import math

import pytest

from maze_nav.follower import (
    STATE_BLOCKED_STOP,
    STATE_GOAL_REACHED,
    STATE_ROTATE_TO_PATH,
    STATE_STUCK_RECOVERY,
    STATE_TRACK_PATH,
    STATE_WATCHDOG_STOP,
    FollowerConfig,
    PathFollower,
)


def follower():
    return PathFollower(
        FollowerConfig(
            max_linear_mps=0.06,
            max_angular_rps=0.30,
            obstacle_stop_distance_m=0.35,
        )
    )


def test_default_follower_is_conservative_by_construction():
    cfg = FollowerConfig()

    assert cfg.max_linear_mps <= 0.06
    assert cfg.max_angular_rps <= 0.30
    assert cfg.emergency_stop_distance_m < cfg.obstacle_stop_distance_m


def test_follower_tracks_forward_path():
    cmd = follower().compute(
        pose=(0.0, 0.0, 0.0),
        path_xy=[(0.0, 0.0), (0.3, 0.0), (1.0, 0.0)],
        front_clearance_m=2.0,
    )

    assert cmd.state == STATE_TRACK_PATH
    assert cmd.linear > 0.0
    assert abs(cmd.angular) < 1e-6


def test_follower_rotates_without_advancing_when_heading_is_large():
    cmd = follower().compute(
        pose=(0.0, 0.0, 0.0),
        path_xy=[(0.0, 0.0), (0.0, 1.0)],
        front_clearance_m=2.0,
    )

    assert cmd.state == STATE_ROTATE_TO_PATH
    assert cmd.linear == 0.0
    assert cmd.angular > 0.0


def test_follower_uses_shortest_large_rotate_even_when_side_is_close():
    cmd = follower().compute(
        pose=(0.0, 0.0, 0.0),
        path_xy=[(0.0, 0.0), (-0.6, 0.6)],
        front_clearance_m=0.8,
        front_emergency_m=0.8,
        avoidance_turn=-1,
    )

    assert cmd.state == STATE_ROTATE_TO_PATH
    assert cmd.linear == 0.0
    assert cmd.angular > 0.0


def test_follower_latches_large_rotate_direction_despite_side_scan_flicker():
    ctl = follower()
    pose = (0.8797, 0.3599, math.radians(77.32))
    path = [(0.55, 0.10)]
    avoidance_sequence = [-1, 0, -1, 0, -1, 0]
    angular_signs = []
    heading_errors = []

    for avoidance_turn in avoidance_sequence:
        cmd = ctl.compute(
            pose=pose,
            path_xy=path,
            front_clearance_m=0.49,
            front_emergency_m=0.47,
            avoidance_turn=avoidance_turn,
        )
        angular_signs.append(1 if cmd.angular > 0.0 else -1)
        heading_errors.append(abs(cmd.heading_error_rad))
        pose = (
            pose[0],
            pose[1],
            math.atan2(
                math.sin(pose[2] + cmd.angular * 0.10),
                math.cos(pose[2] + cmd.angular * 0.10),
            ),
        )

    assert set(angular_signs) == {1}
    assert heading_errors[-1] < heading_errors[0]


def test_follower_does_not_alternate_max_turns_on_cone_bug_replay():
    ctl = follower()
    path = [(-0.34, 2.178)]
    pose = (-0.307, 0.184, math.radians(179.5))
    dt = 0.10
    angular_signs = []
    heading_errors = []

    for _ in range(40):
        cmd = ctl.compute(
            pose=pose,
            path_xy=path,
            front_clearance_m=2.0,
        )
        heading_errors.append(abs(cmd.heading_error_rad))
        if abs(cmd.angular) > 1e-6:
            angular_signs.append(1 if cmd.angular > 0.0 else -1)
        pose = (
            pose[0],
            pose[1],
            math.atan2(
                math.sin(pose[2] + cmd.angular * dt),
                math.cos(pose[2] + cmd.angular * dt),
            ),
        )

    assert angular_signs
    assert set(angular_signs[:20]) == {-1}
    assert heading_errors[-1] < heading_errors[0]
    assert cmd.state in {STATE_TRACK_PATH, STATE_ROTATE_TO_PATH}


def test_follower_stops_for_front_obstacle():
    cmd = follower().compute(
        pose=(0.0, 0.0, 0.0),
        path_xy=[(0.0, 0.0), (1.0, 0.0)],
        front_clearance_m=0.12,
    )

    assert cmd.state == STATE_BLOCKED_STOP
    assert cmd.linear == 0.0
    assert cmd.angular == 0.0


def test_follower_allows_in_place_rotation_with_front_obstacle_when_heading_large():
    cmd = follower().compute(
        pose=(0.0, 0.0, 0.0),
        path_xy=[(0.0, 0.0), (0.0, 1.0)],
        front_clearance_m=0.14,
        front_emergency_m=0.13,
    )

    assert cmd.state == STATE_ROTATE_TO_PATH
    assert cmd.linear == 0.0
    assert cmd.angular > 0.0


def test_follower_keeps_rotating_with_front_obstacle_until_more_aligned():
    ctl = follower()
    ctl.compute(
        pose=(0.0, 0.0, 0.0),
        path_xy=[(0.0, 0.0), (0.0, 1.0)],
        front_clearance_m=0.50,
        front_emergency_m=0.50,
    )
    cmd = ctl.compute(
        pose=(0.0, 0.0, math.radians(57.0)),
        path_xy=[(0.0, 0.0), (0.0, 1.0)],
        front_clearance_m=0.14,
        front_emergency_m=0.13,
    )

    assert math.degrees(abs(cmd.heading_error_rad)) == pytest.approx(33.0, abs=0.1)
    assert cmd.state == STATE_ROTATE_TO_PATH
    assert cmd.linear == 0.0
    assert cmd.angular > 0.0


def test_follower_recovers_from_soft_front_obstacle_toward_open_side():
    cmd = follower().compute(
        pose=(0.0, 0.0, 0.0),
        path_xy=[(0.0, 0.0), (1.0, 0.0)],
        front_clearance_m=0.30,
        front_emergency_m=0.30,
        avoidance_turn=-1,
    )

    assert cmd.state == STATE_STUCK_RECOVERY
    assert cmd.linear == 0.0
    assert cmd.angular < 0.0


def test_follower_recovery_turns_without_creeping_forward():
    ctl = follower()
    path = [(0.0, 0.0), (1.0, 0.0)]

    ctl.compute(
        pose=(0.0, 0.0, 0.0),
        path_xy=path,
        front_clearance_m=0.30,
        front_emergency_m=0.30,
        avoidance_turn=1,
    )
    cmd = ctl.compute(
        pose=(0.0, 0.0, 0.0),
        path_xy=path,
        front_clearance_m=0.60,
        front_emergency_m=0.60,
        avoidance_turn=1,
    )

    assert cmd.state == STATE_STUCK_RECOVERY
    assert cmd.linear == 0.0
    assert cmd.angular > 0.0


def test_follower_uses_emergency_min_but_tracks_with_robust_clearance():
    ctl = follower()

    cmd = ctl.compute(
        pose=(0.0, 0.0, 0.0),
        path_xy=[(0.0, 0.0), (1.0, 0.0)],
        front_clearance_m=1.2,
        front_emergency_m=0.22,
    )
    assert cmd.state == STATE_TRACK_PATH
    assert cmd.linear > 0.0

    cmd = ctl.compute(
        pose=(0.0, 0.0, 0.0),
        path_xy=[(0.0, 0.0), (1.0, 0.0)],
        front_clearance_m=1.2,
        front_emergency_m=0.12,
    )
    assert cmd.state == STATE_TRACK_PATH
    assert cmd.linear > 0.0

    cmd = ctl.compute(
        pose=(0.0, 0.0, 0.0),
        path_xy=[(0.0, 0.0), (1.0, 0.0)],
        front_clearance_m=0.25,
        front_emergency_m=0.12,
    )
    assert cmd.state == STATE_BLOCKED_STOP
    assert cmd.linear == 0.0


def test_follower_watchdog_stops_on_stale_scan_or_pose():
    cmd_scan = follower().compute(
        pose=(0.0, 0.0, 0.0),
        path_xy=[(0.0, 0.0), (1.0, 0.0)],
        front_clearance_m=2.0,
        scan_age_s=2.0,
    )
    cmd_pose = follower().compute(
        pose=(0.0, 0.0, 0.0),
        path_xy=[(0.0, 0.0), (1.0, 0.0)],
        front_clearance_m=2.0,
        pose_age_s=2.0,
    )

    assert cmd_scan.state == STATE_WATCHDOG_STOP
    assert cmd_pose.state == STATE_WATCHDOG_STOP


def test_follower_reports_goal_reached_when_position_and_yaw_match():
    cmd = follower().compute(
        pose=(0.02, 0.0, 0.05),
        path_xy=[(0.0, 0.0)],
        goal_yaw=0.0,
        front_clearance_m=2.0,
    )

    assert cmd.state == STATE_GOAL_REACHED
    assert cmd.linear == 0.0
    assert cmd.angular == 0.0
