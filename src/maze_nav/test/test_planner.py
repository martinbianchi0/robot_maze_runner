import numpy as np

from maze_nav.planner import (
    GridSpec,
    astar,
    cells_to_world_path,
    is_segment_free,
    limit_path_stride,
    nearest_free_cell,
    simplify_path,
    world_to_grid,
)


def test_world_to_grid_and_cells_to_world_are_consistent():
    spec = GridSpec(resolution=0.5, origin_x=-1.0, origin_y=-1.0)

    assert world_to_grid(0.0, 0.0, spec) == (2, 2)
    assert cells_to_world_path([(2, 2)], spec) == [(0.25, 0.25)]


def test_astar_routes_around_wall_gap_without_crossing_obstacles():
    grid = np.zeros((7, 7), dtype=np.int8)
    grid[:, 3] = 100
    grid[3, 3] = 0

    path = astar(grid, (1, 3), (5, 3), allow_diagonal=True)

    assert path[0] == (1, 3)
    assert path[-1] == (5, 3)
    assert (3, 3) in path
    assert all(grid[gy, gx] == 0 for gx, gy in path)


def test_astar_refuses_blocked_start_or_goal():
    grid = np.zeros((3, 3), dtype=np.int8)
    grid[0, 0] = 100

    assert astar(grid, (0, 0), (2, 2)) == []
    assert astar(grid, (1, 1), (0, 0)) == []


def test_collision_segment_detects_wall():
    grid = np.zeros((5, 5), dtype=np.int8)
    grid[2, 2] = 100

    assert not is_segment_free(grid, (0, 2), (4, 2))
    assert is_segment_free(grid, (0, 0), (4, 0))


def test_nearest_free_cell_finds_adjacent_free_space():
    grid = np.zeros((5, 5), dtype=np.int8)
    grid[2, 2] = 100

    assert nearest_free_cell(grid, (2, 2), max_radius=2) in {
        (1, 2), (3, 2), (2, 1), (2, 3), (1, 1), (1, 3), (3, 1), (3, 3)
    }


def test_simplify_path_preserves_collision_free_turning_point():
    grid = np.zeros((6, 6), dtype=np.int8)
    grid[2, 2] = 100
    path = [(0, 2), (1, 2), (1, 3), (2, 3), (3, 3), (4, 3)]

    simplified = simplify_path(grid, path)

    assert simplified[0] == path[0]
    assert simplified[-1] == path[-1]
    for a, b in zip(simplified, simplified[1:]):
        assert is_segment_free(grid, a, b)


def test_limit_path_stride_keeps_dense_intermediate_waypoints():
    path = [(i, 0) for i in range(12)]

    limited = limit_path_stride(path, max_stride_cells=3)

    assert limited[0] == path[0]
    assert limited[-1] == path[-1]
    assert len(limited) > 2
    assert all(
        max(abs(b[0] - a[0]), abs(b[1] - a[1])) <= 3
        for a, b in zip(limited, limited[1:])
    )
