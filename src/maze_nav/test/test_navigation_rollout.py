import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from maze_nav.costmap import CostmapConfig, inflate_obstacles
from maze_nav.follower import (
    STATE_BLOCKED_STOP,
    STATE_GOAL_REACHED,
    STATE_STUCK_RECOVERY,
    STATE_WATCHDOG_STOP,
    FollowerConfig,
    PathFollower,
    wrap_angle,
)
from maze_nav.planner import (
    GridSpec,
    astar,
    cells_to_world_path,
    is_cell_free,
    limit_path_stride,
    path_length,
    world_to_grid,
)
from maze_nav.map_io import load_map_yaml


@dataclass(frozen=True)
class RolloutResult:
    history: list
    elapsed_s: float
    distance_m: float
    planned_length_m: float
    rotate_only_s: float
    final_pose: tuple


def _polyline_length(points):
    if len(points) < 2:
        return 0.0
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )


def _plan_path(grid, spec, start_pose, goal_pose, stride_cells=1):
    start = world_to_grid(start_pose[0], start_pose[1], spec)
    goal = world_to_grid(goal_pose[0], goal_pose[1], spec)
    path_cells = astar(grid, start, goal, allow_diagonal=True)

    assert path_cells, f'planner failed from {start} to {goal}'
    assert path_cells[0] == start
    assert path_cells[-1] == goal

    path_cells = limit_path_stride(path_cells, max_stride_cells=stride_cells)
    path_xy = cells_to_world_path(path_cells, spec)
    path_xy[0] = (start_pose[0], start_pose[1])
    path_xy[-1] = (goal_pose[0], goal_pose[1])
    return path_cells, path_xy


def _front_clearance(grid, spec, pose, max_range_m=2.0):
    x, y, yaw = pose
    step = spec.resolution * 0.5
    samples = int(max_range_m / step)
    for i in range(1, samples + 1):
        distance = i * step
        cell = world_to_grid(
            x + math.cos(yaw) * distance,
            y + math.sin(yaw) * distance,
            spec,
        )
        if not is_cell_free(grid, cell):
            return distance
    return max_range_m


def _assert_footprint_free(grid, spec, pose, radius_m=0.11):
    x, y, _ = pose
    samples = [(0.0, 0.0)]
    for angle in np.linspace(0.0, 2.0 * math.pi, 16, endpoint=False):
        samples.append((math.cos(angle) * radius_m, math.sin(angle) * radius_m))

    for dx, dy in samples:
        cell = world_to_grid(x + dx, y + dy, spec)
        assert is_cell_free(grid, cell), f'collision at pose={pose} cell={cell}'


def _rollout_to_goal(
    plan_grid,
    spec,
    start_pose,
    goal_pose,
    max_time_s,
    collision_grid=None,
    config=None,
):
    if collision_grid is None:
        collision_grid = plan_grid
    path_cells, path_xy = _plan_path(plan_grid, spec, start_pose, goal_pose)
    cfg = config or FollowerConfig()
    follower = PathFollower(cfg)

    pose = start_pose
    history = []
    distance_m = 0.0
    rotate_only_s = 0.0
    dt = 0.10
    forbidden = {STATE_BLOCKED_STOP, STATE_STUCK_RECOVERY, STATE_WATCHDOG_STOP}
    max_steps = int(max_time_s / dt)

    for step in range(max_steps):
        _assert_footprint_free(collision_grid, spec, pose)
        clearance = _front_clearance(collision_grid, spec, pose)
        cmd = follower.compute(
            pose,
            path_xy,
            goal_yaw=goal_pose[2],
            front_clearance_m=clearance,
        )
        history.append((pose, cmd))
        assert cmd.state not in forbidden, (
            f'unexpected state={cmd.state} pose={pose} '
            f'target={cmd.target_index} heading_error='
            f'{math.degrees(cmd.heading_error_rad):.1f} clearance={clearance:.2f}'
        )
        if cmd.linear < 1e-4 and abs(cmd.angular) > 1e-4:
            rotate_only_s += dt
        if cmd.state == STATE_GOAL_REACHED:
            return RolloutResult(
                history=history,
                elapsed_s=step * dt,
                distance_m=distance_m,
                planned_length_m=path_length(path_cells) * spec.resolution,
                rotate_only_s=rotate_only_s,
                final_pose=pose,
            )

        x, y, yaw = pose
        nx = x + math.cos(yaw) * cmd.linear * dt
        ny = y + math.sin(yaw) * cmd.linear * dt
        distance_m += math.hypot(nx - x, ny - y)
        pose = (nx, ny, wrap_angle(yaw + cmd.angular * dt))

    last_pose, last_cmd = history[-1]
    distance_to_goal = math.hypot(last_pose[0] - goal_pose[0], last_pose[1] - goal_pose[1])
    raise AssertionError(
        f'goal not reached in {max_time_s:.1f}s: distance={distance_to_goal:.3f} '
        f'state={last_cmd.state} pose={last_pose} '
        f'heading_error={math.degrees(last_cmd.heading_error_rad):.1f}'
    )


def _open_grid():
    return np.zeros((90, 90), dtype=np.int8), GridSpec(
        resolution=0.05,
        origin_x=-2.25,
        origin_y=-2.25,
    )


def _assert_reaches_efficiently(result, start, goal, max_time, max_efficiency):
    straight = math.hypot(goal[0] - start[0], goal[1] - start[1])
    final_error = math.hypot(result.final_pose[0] - goal[0], result.final_pose[1] - goal[1])

    assert result.elapsed_s <= max_time
    assert final_error <= 0.11
    if straight > 1e-6:
        assert result.distance_m <= straight * max_efficiency + 0.08
    assert result.rotate_only_s <= max(7.0, result.elapsed_s * 0.45)


def test_open_map_reaches_straight_goal_fast_without_overshoot():
    grid, spec = _open_grid()
    start = (0.0, 0.0, 0.0)
    goal = (1.0, 0.0, 0.0)

    result = _rollout_to_goal(grid, spec, start, goal, max_time_s=24.0)

    xs = [pose[0] for pose, _ in result.history]
    assert max(xs) <= goal[0] + 0.04
    _assert_reaches_efficiently(result, start, goal, max_time=24.0, max_efficiency=1.12)


def test_open_map_turns_then_drives_when_goal_is_behind():
    grid, spec = _open_grid()
    start = (0.0, 0.0, 0.0)
    goal = (-1.0, 0.0, math.pi)

    result = _rollout_to_goal(grid, spec, start, goal, max_time_s=40.0)

    _assert_reaches_efficiently(result, start, goal, max_time=40.0, max_efficiency=1.20)


def test_open_map_handles_perpendicular_goal_without_cone_spin():
    grid, spec = _open_grid()
    start = (0.0, 0.0, 0.0)
    goal = (0.0, 1.0, math.pi / 2.0)

    result = _rollout_to_goal(grid, spec, start, goal, max_time_s=34.0)

    _assert_reaches_efficiently(result, start, goal, max_time=34.0, max_efficiency=1.22)


def test_open_map_handles_diagonal_goal_efficiently():
    grid, spec = _open_grid()
    start = (-0.8, -0.6, math.radians(20.0))
    goal = (0.9, 0.75, math.radians(45.0))

    result = _rollout_to_goal(grid, spec, start, goal, max_time_s=58.0)

    _assert_reaches_efficiently(result, start, goal, max_time=58.0, max_efficiency=1.25)


def test_open_map_short_goal_does_not_overshoot_and_hunt():
    grid, spec = _open_grid()
    start = (0.0, 0.0, 0.0)
    goal = (0.28, 0.0, 0.0)

    result = _rollout_to_goal(grid, spec, start, goal, max_time_s=12.0)

    xs = [pose[0] for pose, _ in result.history]
    assert max(xs) <= goal[0] + 0.035
    _assert_reaches_efficiently(result, start, goal, max_time=12.0, max_efficiency=1.15)


def test_open_map_randomized_goal_set_reaches_with_reasonable_time():
    grid, spec = _open_grid()
    cases = [
        ((-1.2, -1.0, 0.0), (-0.2, -1.0, 0.0)),
        ((-1.0, -0.3, math.pi), (0.8, -0.3, 0.0)),
        ((0.9, -0.8, math.radians(170.0)), (-0.8, 0.6, 0.0)),
        ((-0.2, 1.1, math.radians(-90.0)), (1.0, 0.2, 0.0)),
        ((1.1, 1.0, math.radians(135.0)), (-1.0, -0.4, 0.0)),
        ((0.2, -1.2, math.radians(80.0)), (0.2, 1.0, 0.0)),
    ]

    for start, goal in cases:
        straight = math.hypot(goal[0] - start[0], goal[1] - start[1])
        budget = 14.0 + straight / 0.038
        result = _rollout_to_goal(grid, spec, start, goal, max_time_s=budget)
        _assert_reaches_efficiently(
            result,
            start,
            goal,
            max_time=budget,
            max_efficiency=1.35,
        )


def test_wall_gap_is_crossed_without_collision_or_big_detour():
    raw_grid = np.zeros((55, 70), dtype=np.int8)
    raw_grid[:, 34] = 100
    raw_grid[20:35, 34] = 0
    spec = GridSpec(resolution=0.05, origin_x=0.0, origin_y=0.0)
    plan_grid = inflate_obstacles(
        raw_grid,
        spec.resolution,
        CostmapConfig(inflation_radius_m=0.18, unknown_as_obstacle=False),
    )
    start = (0.35, 1.35, 0.0)
    goal = (3.10, 1.35, 0.0)

    result = _rollout_to_goal(
        plan_grid,
        spec,
        start,
        goal,
        max_time_s=75.0,
        collision_grid=raw_grid,
    )

    assert any(pose[0] > 1.80 for pose, _ in result.history)
    assert result.distance_m <= result.planned_length_m * 1.35 + 0.15


def test_l_corridor_turns_corner_without_false_front_block():
    raw_grid = np.full((65, 65), 100, dtype=np.int8)
    raw_grid[8:25, 5:54] = 0
    raw_grid[8:57, 36:54] = 0
    spec = GridSpec(resolution=0.05, origin_x=0.0, origin_y=0.0)
    plan_grid = inflate_obstacles(
        raw_grid,
        spec.resolution,
        CostmapConfig(inflation_radius_m=0.18, unknown_as_obstacle=False),
    )
    start = (0.45, 0.85, 0.0)
    goal = (2.25, 2.55, math.pi / 2.0)

    result = _rollout_to_goal(
        plan_grid,
        spec,
        start,
        goal,
        max_time_s=55.0,
        collision_grid=raw_grid,
    )

    assert result.final_pose[1] > 2.45
    assert result.distance_m <= result.planned_length_m * 1.45 + 0.20


def test_s_corridor_follows_multiple_turns_without_corner_cutting():
    raw_grid = np.full((80, 90), 100, dtype=np.int8)
    raw_grid[8:24, 6:70] = 0
    raw_grid[8:55, 54:70] = 0
    raw_grid[40:55, 20:70] = 0
    raw_grid[40:72, 20:36] = 0
    spec = GridSpec(resolution=0.05, origin_x=0.0, origin_y=0.0)
    plan_grid = inflate_obstacles(
        raw_grid,
        spec.resolution,
        CostmapConfig(inflation_radius_m=0.18, unknown_as_obstacle=False),
    )
    start = (0.45, 0.80, 0.0)
    goal = (1.35, 3.25, math.pi / 2.0)

    result = _rollout_to_goal(
        plan_grid,
        spec,
        start,
        goal,
        max_time_s=130.0,
        collision_grid=raw_grid,
    )

    assert result.final_pose[1] > 3.15
    assert result.distance_m <= result.planned_length_m * 1.55 + 0.25


def test_blocked_straight_path_stops_instead_of_driving_into_wall():
    grid, spec = _open_grid()
    path = [(0.0, 0.0), (1.0, 0.0)]
    follower = PathFollower(FollowerConfig())

    cmd = follower.compute(
        pose=(0.0, 0.0, 0.0),
        path_xy=path,
        front_clearance_m=0.12,
    )

    assert cmd.state == STATE_BLOCKED_STOP
    assert cmd.linear == 0.0
    assert cmd.angular == 0.0


def _load_real_tuned_costmap():
    repo_root = Path(__file__).resolve().parents[3]
    occupancy, info = load_map_yaml(repo_root / 'results' / 'parte_a' / 'casa_map_tuned.yaml')
    costmap = inflate_obstacles(
        occupancy,
        info.resolution,
        CostmapConfig(inflation_radius_m=0.18, unknown_as_obstacle=True),
    )
    spec = GridSpec(
        resolution=info.resolution,
        origin_x=info.origin_x,
        origin_y=info.origin_y,
    )
    return occupancy, costmap, spec


def test_real_parte_a_tuned_map_reaches_short_spawn_goal():
    occupancy, costmap, spec = _load_real_tuned_costmap()
    start = (0.0, 0.0, 0.0)
    goal = (0.8, 0.0, 0.0)

    result = _rollout_to_goal(
        costmap,
        spec,
        start,
        goal,
        max_time_s=18.0,
        collision_grid=occupancy,
    )

    _assert_reaches_efficiently(result, start, goal, max_time=18.0, max_efficiency=1.25)


def test_real_parte_a_tuned_map_reaches_perpendicular_spawn_goal():
    occupancy, costmap, spec = _load_real_tuned_costmap()
    start = (0.0, 0.0, 0.0)
    goal = (0.0, 1.0, math.pi / 2.0)

    result = _rollout_to_goal(
        costmap,
        spec,
        start,
        goal,
        max_time_s=38.0,
        collision_grid=occupancy,
    )

    _assert_reaches_efficiently(result, start, goal, max_time=38.0, max_efficiency=1.35)
