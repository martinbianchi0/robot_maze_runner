import json
import math

import pytest


def test_nav_node_imports_without_path_alias_collision():
    pytest.importorskip('rclpy')
    import maze_nav.nav_node as nav_node

    assert nav_node.FilePath('x').name == 'x'
    assert nav_node.PathMsg.__name__ == 'Path'


def test_nav_node_constructs_with_tiny_map(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from maze_nav.nav_node import MazeNavNode

    pgm = tmp_path / 'tiny.pgm'
    pgm.write_bytes(b'P5\n3 3\n255\n' + bytes([254] * 9))
    yaml = tmp_path / 'tiny.yaml'
    yaml.write_text(
        f'image: {pgm.name}\n'
        'mode: trinary\n'
        'resolution: 0.05\n'
        'origin: [0.0, 0.0, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n',
        encoding='utf-8',
    )

    rclpy.init(args=[
        '--ros-args',
        '-p',
        f'map_yaml:={yaml}',
        '-p',
        'mode:=safe_drive',
        '-p',
        'use_sim_time:=false',
    ])
    node = None
    try:
        node = MazeNavNode()
        assert node.costmap.shape == (3, 3)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_nav_node_accepts_live_map_topic():
    rclpy = pytest.importorskip('rclpy')
    from nav_msgs.msg import OccupancyGrid
    from maze_nav.nav_node import MazeNavNode

    rclpy.init(args=[
        '--ros-args',
        '-p',
        'mode:=goal',
        '-p',
        'map_source:=topic',
        '-p',
        'publish_loaded_map:=false',
        '-p',
        'use_sim_time:=false',
    ])
    node = None
    try:
        node = MazeNavNode()
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = 0.05
        msg.info.width = 4
        msg.info.height = 3
        msg.info.origin.position.x = -0.1
        msg.info.origin.position.y = -0.1
        msg.info.origin.orientation.w = 1.0
        msg.data = [0] * 12

        node._on_map(msg)

        assert node.costmap.shape == (3, 4)
        assert node.grid_spec.origin_x == -0.1
        assert node.grid_spec.origin_y == -0.1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_nav_node_diagnostic_payload_explains_stop_context(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from maze_nav.nav_node import MazeNavNode

    pgm = tmp_path / 'tiny.pgm'
    pgm.write_bytes(b'P5\n3 3\n255\n' + bytes([254] * 9))
    yaml = tmp_path / 'tiny.yaml'
    yaml.write_text(
        f'image: {pgm.name}\n'
        'mode: trinary\n'
        'resolution: 0.05\n'
        'origin: [0.0, 0.0, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n',
        encoding='utf-8',
    )

    rclpy.init(args=[
        '--ros-args',
        '-p',
        f'map_yaml:={yaml}',
        '-p',
        'mode:=goal',
        '-p',
        'use_sim_time:=false',
    ])
    node = None
    try:
        node = MazeNavNode()
        node.raw_pose = (0.1, 0.2, 0.3)
        node.goal_pose = (0.4, 0.6, 0.0)
        node.front_clearance_m = 0.31
        node.front_emergency_m = 0.21
        node.path_xy = [(0.1, 0.2), (0.4, 0.6)]

        payload = node._diagnostic_payload(
            12.3456,
            'BLOCKED_STOP',
            pose=node.raw_pose,
            linear=0.0,
            angular=0.0,
            target_index=1,
            heading_error_rad=0.2,
            scan_age_s=0.12,
            pose_age_s=0.05,
            reason='follow_path',
        )

        encoded = json.dumps(payload)
        decoded = json.loads(encoded)
        assert decoded['state'] == 'BLOCKED_STOP'
        assert decoded['reason'] == 'follow_path'
        assert decoded['front_clearance_m'] == 0.31
        assert decoded['front_emergency_m'] == 0.21
        assert decoded['goal_distance_m'] > 0.0
        assert decoded['target_index'] == 1
        assert decoded['heading_error_deg'] == pytest.approx(11.46)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_nav_node_scan_overlay_marks_dynamic_obstacle(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from sensor_msgs.msg import LaserScan
    from maze_nav.nav_node import MazeNavNode
    from maze_nav.planner import world_to_grid

    pgm = tmp_path / 'tiny.pgm'
    pgm.write_bytes(b'P5\n40 40\n255\n' + bytes([254] * 1600))
    yaml = tmp_path / 'tiny.yaml'
    yaml.write_text(
        f'image: {pgm.name}\n'
        'mode: trinary\n'
        'resolution: 0.05\n'
        'origin: [-1.0, -1.0, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n',
        encoding='utf-8',
    )

    rclpy.init(args=[
        '--ros-args',
        '-p',
        f'map_yaml:={yaml}',
        '-p',
        'mode:=goal',
        '-p',
        'use_sim_time:=false',
        '-p',
        'scan_obstacle_inflation_radius_m:=0.05',
        '-p',
        'scan_obstacle_min_cluster_size:=1',
        '-p',
        'use_scan_obstacle_overlay:=true',
    ])
    node = None
    try:
        node = MazeNavNode()
        scan = LaserScan()
        scan.angle_min = 0.0
        scan.angle_increment = 1.0
        scan.range_min = 0.05
        scan.range_max = 3.5
        scan.ranges = [0.5]

        node._on_scan(scan)
        node._update_scan_obstacle_overlay((0.0, 0.0, 0.0), 10.0)

        gx, gy = world_to_grid(0.5, 0.0, node.grid_spec)
        assert node.static_costmap[gy, gx] == 0
        assert node.costmap[gy, gx] >= 50
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_nav_node_scan_overlay_ignores_isolated_speckle_but_keeps_cluster(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from sensor_msgs.msg import LaserScan
    from maze_nav.nav_node import MazeNavNode
    from maze_nav.planner import world_to_grid

    pgm = tmp_path / 'tiny.pgm'
    pgm.write_bytes(b'P5\n40 40\n255\n' + bytes([254] * 1600))
    yaml = tmp_path / 'tiny.yaml'
    yaml.write_text(
        f'image: {pgm.name}\n'
        'mode: trinary\n'
        'resolution: 0.05\n'
        'origin: [-1.0, -1.0, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n',
        encoding='utf-8',
    )

    rclpy.init(args=[
        '--ros-args',
        '-p',
        f'map_yaml:={yaml}',
        '-p',
        'mode:=goal',
        '-p',
        'use_sim_time:=false',
        '-p',
        'scan_obstacle_inflation_radius_m:=0.05',
        '-p',
        'scan_obstacle_min_cluster_size:=3',
        '-p',
        'use_scan_obstacle_overlay:=true',
    ])
    node = None
    try:
        node = MazeNavNode()
        scan = LaserScan()
        scan.angle_min = -0.02
        scan.angle_increment = 0.02
        scan.range_min = 0.05
        scan.range_max = 3.5
        scan.ranges = [math.inf, 0.5, math.inf]

        node._on_scan(scan)
        node._update_scan_obstacle_overlay((0.0, 0.0, 0.0), 10.0)

        gx, gy = world_to_grid(0.5, 0.0, node.grid_spec)
        assert node.costmap[gy, gx] == 0
        assert node.scan_overlay_cells == 0

        scan.ranges = [0.50, 0.51, 0.49]
        node._on_scan(scan)
        node._update_scan_obstacle_overlay((0.0, 0.0, 0.0), 11.0)

        assert node.costmap[gy, gx] >= 50
        assert node.scan_overlay_cells >= 2
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_nav_node_scan_overlay_replans_when_dynamic_obstacle_hits_path(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from sensor_msgs.msg import LaserScan
    from maze_nav.nav_node import MazeNavNode

    pgm = tmp_path / 'tiny.pgm'
    pgm.write_bytes(b'P5\n60 60\n255\n' + bytes([254] * 3600))
    yaml = tmp_path / 'tiny.yaml'
    yaml.write_text(
        f'image: {pgm.name}\n'
        'mode: trinary\n'
        'resolution: 0.05\n'
        'origin: [-1.5, -1.5, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n',
        encoding='utf-8',
    )

    rclpy.init(args=[
        '--ros-args',
        '-p',
        f'map_yaml:={yaml}',
        '-p',
        'mode:=goal',
        '-p',
        'use_sim_time:=false',
        '-p',
        'scan_obstacle_inflation_radius_m:=0.08',
        '-p',
        'scan_obstacle_min_cluster_size:=3',
        '-p',
        'scan_obstacle_replan_period_s:=0.5',
        '-p',
        'use_scan_obstacle_overlay:=true',
    ])
    node = None
    try:
        node = MazeNavNode()
        node.goal_pose = (1.0, 0.0, 0.0)
        node.path_xy = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)]
        node.need_replan = False
        node.front_clearance_m = 1.0

        scan = LaserScan()
        scan.angle_min = -0.02
        scan.angle_increment = 0.02
        scan.range_min = 0.05
        scan.range_max = 3.5
        scan.ranges = [0.50, 0.51, 0.49]

        node._on_scan(scan)
        node._update_scan_obstacle_overlay((0.0, 0.0, 0.0), 10.0)

        assert node._path_hits_costmap()
        assert node.need_replan
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_nav_node_progress_watchdog_detects_wandering_without_goal_progress(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from maze_nav.nav_node import MazeNavNode

    pgm = tmp_path / 'tiny.pgm'
    pgm.write_bytes(b'P5\n80 80\n255\n' + bytes([254] * 6400))
    yaml = tmp_path / 'tiny.yaml'
    yaml.write_text(
        f'image: {pgm.name}\n'
        'mode: trinary\n'
        'resolution: 0.05\n'
        'origin: [-2.0, -2.0, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n',
        encoding='utf-8',
    )

    rclpy.init(args=[
        '--ros-args',
        '-p',
        f'map_yaml:={yaml}',
        '-p',
        'mode:=goal',
        '-p',
        'use_sim_time:=false',
        '-p',
        'goal_progress_timeout_s:=3.0',
        '-p',
        'goal_progress_min_motion_m:=0.20',
    ])
    node = None
    try:
        node = MazeNavNode()
        node.goal_pose = (1.0, 0.0, 0.0)
        node._reset_goal_progress(10.0, (0.0, 0.0, 0.0), target_index=5)

        assert not node._goal_progress_blocked(12.0, (-0.1, 0.3, 0.0), 5)
        assert node._goal_progress_blocked(13.2, (-0.12, 0.35, 0.0), 5)
        assert not node._goal_progress_blocked(13.3, (-0.12, 0.35, 0.0), 7)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_nav_node_regression_watchdog_detects_moving_away_from_goal(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from maze_nav.nav_node import MazeNavNode

    pgm = tmp_path / 'tiny.pgm'
    pgm.write_bytes(b'P5\n80 80\n255\n' + bytes([254] * 6400))
    yaml = tmp_path / 'tiny.yaml'
    yaml.write_text(
        f'image: {pgm.name}\n'
        'mode: trinary\n'
        'resolution: 0.05\n'
        'origin: [-2.0, -2.0, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n',
        encoding='utf-8',
    )

    rclpy.init(args=[
        '--ros-args',
        '-p',
        f'map_yaml:={yaml}',
        '-p',
        'mode:=goal',
        '-p',
        'use_sim_time:=false',
        '-p',
        'goal_regression_timeout_s:=3.0',
        '-p',
        'goal_regression_min_motion_m:=0.20',
    ])
    node = None
    try:
        node = MazeNavNode()
        node.goal_pose = (1.0, 0.0, 0.0)
        node._reset_goal_regression(10.0, (0.0, 0.0, 0.0))

        assert not node._goal_regression_blocked(12.0, (-0.10, 0.20, 0.0))
        assert node._goal_regression_blocked(13.2, (-0.12, 0.35, 0.0))

        node._plan((0.0, 0.0, 0.0))
        assert node.regression_last_improvement_time == 10.0

        assert not node._goal_regression_blocked(13.3, (0.25, 0.02, 0.0))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_nav_node_latches_blocked_stop_until_new_goal(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import PoseStamped
    from maze_nav.nav_node import MazeNavNode

    pgm = tmp_path / 'tiny.pgm'
    pgm.write_bytes(b'P5\n20 20\n255\n' + bytes([254] * 400))
    yaml = tmp_path / 'tiny.yaml'
    yaml.write_text(
        f'image: {pgm.name}\n'
        'mode: trinary\n'
        'resolution: 0.05\n'
        'origin: [-0.5, -0.5, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n',
        encoding='utf-8',
    )

    rclpy.init(args=[
        '--ros-args',
        '-p',
        f'map_yaml:={yaml}',
        '-p',
        'mode:=goal',
        '-p',
        'use_sim_time:=false',
    ])
    node = None
    try:
        node = MazeNavNode()
        node.raw_pose = (0.0, 0.0, 0.0)
        node.raw_pose_time = 10.0
        node.goal_pose = (0.2, 0.0, 0.0)
        node.blocked_latched = True
        node.blocked_latched_reason = 'no_progress'

        node._run_goal_nav(10.1)

        assert node.state == 'BLOCKED_STOP'
        assert node.blocked_latched

        msg = PoseStamped()
        msg.pose.position.x = 0.1
        msg.pose.orientation.w = 1.0
        node._on_goal_pose(msg)

        assert not node.blocked_latched
        assert node.blocked_latched_reason == ''
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
