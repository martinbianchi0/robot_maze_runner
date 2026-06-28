"""Nodo ROS 2 de navegacion base para Parte B."""

import json
import math
import time
from pathlib import Path as FilePath

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from maze_nav.costmap import CostmapConfig, inflate_obstacles
from maze_nav.follower import (
    STATE_BLOCKED_STOP,
    STATE_GOAL_REACHED,
    STATE_IDLE,
    STATE_ROTATE_TO_PATH,
    STATE_STUCK_RECOVERY,
    STATE_TRACK_PATH,
    STATE_WATCHDOG_STOP,
    FollowerConfig,
    PathFollower,
)
from maze_nav.map_io import MapInfo, load_map_yaml
from maze_nav.planner import (
    GridSpec,
    astar,
    cells_to_world_path,
    limit_path_stride,
    nearest_free_cell,
    world_to_grid,
)
from maze_nav.safe_drive import SafeDriveConfig, SafeDriveController, analyze_scan


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quaternion_from_yaw(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose_pose(a, b):
    ax, ay, ath = a
    bx, by, bth = b
    c = math.cos(ath)
    s = math.sin(ath)
    return (
        ax + c * bx - s * by,
        ay + s * bx + c * by,
        wrap_angle(ath + bth),
    )


def inverse_pose(pose):
    x, y, th = pose
    c = math.cos(th)
    s = math.sin(th)
    return (-c * x - s * y, s * x - c * y, wrap_angle(-th))


def relative_pose(origin, current):
    return compose_pose(inverse_pose(origin), current)


class MazeNavNode(Node):
    def __init__(self):
        super().__init__('maze_nav_base')

        self.declare_parameter('mode', 'safe_drive')  # safe_drive | goal | auto
        self.declare_parameter('map_source', 'yaml')  # yaml | topic
        self.declare_parameter('map_yaml', 'results/parte_a/casa_map_tuned.yaml')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('publish_loaded_map', True)
        self.declare_parameter('replan_on_map_update', False)
        self.declare_parameter('pose_topic', '/odom')
        self.declare_parameter('pose_topic_type', 'odometry')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('require_initial_pose', False)
        self.declare_parameter('inflation_radius_m', 0.18)
        self.declare_parameter('unknown_as_obstacle', True)
        self.declare_parameter('allow_unknown_planning', False)
        self.declare_parameter('goal_search_radius_cells', 18)
        self.declare_parameter('path_stride_cells', 1)
        self.declare_parameter('lookahead_m', 0.20)
        self.declare_parameter('max_linear_mps', 0.06)
        self.declare_parameter('max_angular_rps', 0.30)
        self.declare_parameter('goal_tolerance_m', 0.10)
        self.declare_parameter('blocked_goal_tolerance_m', 0.12)
        self.declare_parameter('yaw_tolerance_rad', 0.35)
        self.declare_parameter('align_final_yaw', False)
        self.declare_parameter('obstacle_stop_distance_m', 0.30)
        self.declare_parameter('emergency_stop_distance_m', 0.16)
        self.declare_parameter('watchdog_timeout_s', 0.80)
        self.declare_parameter('planner_hz', 10.0)
        self.declare_parameter('safe_front_clear_m', 0.65)
        self.declare_parameter('safe_front_slow_m', 0.45)
        self.declare_parameter('safe_front_stop_m', 0.32)
        self.declare_parameter('diagnostic_log_period_s', 1.0)
        self.declare_parameter('use_scan_obstacle_overlay', True)
        self.declare_parameter('scan_obstacle_max_range_m', 1.2)
        self.declare_parameter('scan_obstacle_stride', 2)
        self.declare_parameter('scan_obstacle_min_cluster_size', 3)
        self.declare_parameter('scan_obstacle_cluster_tolerance_m', 0.25)
        self.declare_parameter('scan_obstacle_inflation_radius_m', 0.22)
        self.declare_parameter('scan_obstacle_replan_period_s', 1.0)
        self.declare_parameter('goal_progress_timeout_s', 5.0)
        self.declare_parameter('goal_progress_epsilon_m', 0.03)
        self.declare_parameter('goal_progress_min_motion_m', 0.25)
        self.declare_parameter('goal_progress_target_step', 2)
        self.declare_parameter('goal_regression_timeout_s', 12.0)
        self.declare_parameter('goal_regression_epsilon_m', 0.04)
        self.declare_parameter('goal_regression_min_motion_m', 0.45)

        self.mode = str(self.get_parameter('mode').value)
        self.map_source = str(self.get_parameter('map_source').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.publish_loaded_map = bool(self.get_parameter('publish_loaded_map').value)
        self.replan_on_map_update = bool(self.get_parameter('replan_on_map_update').value)
        self.allow_unknown = bool(self.get_parameter('allow_unknown_planning').value)
        self.goal_search_radius = int(self.get_parameter('goal_search_radius_cells').value)
        self.path_stride_cells = int(self.get_parameter('path_stride_cells').value)
        self.require_initial_pose = bool(self.get_parameter('require_initial_pose').value)
        self.use_scan_obstacle_overlay = bool(
            self.get_parameter('use_scan_obstacle_overlay').value
        )

        self.occupancy = None
        self.map_info = None
        self.grid_spec = None
        self.static_costmap = None
        self.costmap = None
        map_yaml = str(self.get_parameter('map_yaml').value).strip()
        if self.map_source not in ('yaml', 'topic'):
            raise ValueError(f'map_source invalido: {self.map_source!r}')
        if self.map_source == 'yaml' and map_yaml:
            try:
                occupancy, map_info = load_map_yaml(FilePath(map_yaml))
                self._set_map(occupancy, map_info, mark_replan=False)
            except Exception as exc:
                if self.mode in ('goal', 'auto'):
                    raise
                self.get_logger().warn(f'safe_drive sin mapa navegable: {exc}')

        qos_latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = None
        if self.map_source == 'yaml' and self.publish_loaded_map:
            self.map_pub = self.create_publisher(
                OccupancyGrid,
                str(self.get_parameter('map_topic').value),
                qos_latched,
            )
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/global_costmap', qos_latched)
        self.path_pub = self.create_publisher(PathMsg, '/planned_path', qos_latched)
        self.state_pub = self.create_publisher(String, '/nav_state', 10)
        self.debug_pub = self.create_publisher(String, '/nav_debug', 10)
        self.cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            10,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('initial_pose_topic').value),
            self._on_initial_pose,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_pose_topic').value),
            self._on_goal_pose,
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan,
            10,
        )
        if self.map_source == 'topic':
            self.create_subscription(
                OccupancyGrid,
                str(self.get_parameter('map_topic').value),
                self._on_map,
                qos_latched,
            )
        pose_topic = str(self.get_parameter('pose_topic').value)
        pose_type = str(self.get_parameter('pose_topic_type').value)
        if pose_type == 'pose_stamped':
            self.create_subscription(PoseStamped, pose_topic, self._on_pose_stamped, 10)
        else:
            self.create_subscription(Odometry, pose_topic, self._on_odom, 10)

        watchdog = float(self.get_parameter('watchdog_timeout_s').value)
        self.follower = PathFollower(
            FollowerConfig(
                lookahead_m=float(self.get_parameter('lookahead_m').value),
                max_linear_mps=float(self.get_parameter('max_linear_mps').value),
                max_angular_rps=float(self.get_parameter('max_angular_rps').value),
                goal_tolerance_m=float(self.get_parameter('goal_tolerance_m').value),
                blocked_goal_tolerance_m=float(
                    self.get_parameter('blocked_goal_tolerance_m').value
                ),
                yaw_tolerance_rad=float(self.get_parameter('yaw_tolerance_rad').value),
                obstacle_stop_distance_m=float(
                    self.get_parameter('obstacle_stop_distance_m').value
                ),
                emergency_stop_distance_m=float(
                    self.get_parameter('emergency_stop_distance_m').value
                ),
                watchdog_timeout_s=watchdog,
                align_final_yaw=bool(self.get_parameter('align_final_yaw').value),
            )
        )
        self.safe_drive = SafeDriveController(
            SafeDriveConfig(
                max_linear_mps=min(float(self.get_parameter('max_linear_mps').value), 0.05),
                max_angular_rps=min(float(self.get_parameter('max_angular_rps').value), 0.25),
                front_clear_m=float(self.get_parameter('safe_front_clear_m').value),
                front_slow_m=float(self.get_parameter('safe_front_slow_m').value),
                front_stop_m=float(self.get_parameter('safe_front_stop_m').value),
                watchdog_timeout_s=watchdog,
            )
        )

        self.raw_pose = None
        self.raw_pose_time = None
        self.anchor_raw_pose = None
        self.anchor_map_pose = None
        self.goal_pose = None
        self.path_xy = []
        self.need_replan = False
        self.scan_sectors = None
        self.front_clearance_m = math.inf
        self.front_emergency_m = math.inf
        self.last_scan_msg = None
        self.last_scan_time = None
        self.last_map_pub = 0.0
        self.last_costmap_pub = 0.0
        self.last_scan_overlay_time = 0.0
        self.last_scan_replan_time = 0.0
        self.scan_overlay_cells = 0
        self.state = STATE_IDLE
        self.blocked_latched = False
        self.blocked_latched_reason = ''
        self.last_debug_log_time = 0.0
        self.last_debug_state = None
        self.last_timer_wall_time = None
        self.loop_period_ms = None
        self.last_plan_duration_ms = None
        self.last_scan_overlay_duration_ms = None
        self.diagnostic_log_period_s = float(
            self.get_parameter('diagnostic_log_period_s').value
        )
        self.progress_best_goal_distance = math.inf
        self.progress_last_time = 0.0
        self.progress_last_pose = None
        self.progress_last_target_index = -1
        self.regression_best_goal_distance = math.inf
        self.regression_last_improvement_time = 0.0
        self.regression_last_improvement_pose = None

        self._publish_loaded_map()
        self._publish_costmap()
        timer_period = 1.0 / float(self.get_parameter('planner_hz').value)
        self.timer = self.create_timer(timer_period, self._on_timer)
        self.get_logger().info(
            f'maze_nav_base ready: mode={self.mode} map_source={self.map_source} '
            f'map={map_yaml or "none"} '
            f'pose_topic={pose_topic}'
        )

    def _node_time_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_state(self, state):
        self.state = state
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def _finite_or_none(self, value):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value):
            return None
        return value

    def _pose_payload(self, pose):
        if pose is None:
            return None
        return {
            'x': round(float(pose[0]), 4),
            'y': round(float(pose[1]), 4),
            'yaw_deg': round(math.degrees(float(pose[2])), 2),
        }

    def _diagnostic_payload(
        self,
        now,
        state,
        linear=0.0,
        angular=0.0,
        pose=None,
        target_index=None,
        heading_error_rad=None,
        scan_age_s=None,
        pose_age_s=None,
        reason='',
    ):
        goal_distance = None
        if pose is not None and self.goal_pose is not None:
            goal_distance = math.hypot(
                self.goal_pose[0] - pose[0],
                self.goal_pose[1] - pose[1],
            )
        sectors = None
        if self.scan_sectors is not None:
            sectors = {
                'front_m': self._finite_or_none(self.scan_sectors.front),
                'front_min_m': self._finite_or_none(self.scan_sectors.front_min),
                'left_m': self._finite_or_none(self.scan_sectors.left),
                'right_m': self._finite_or_none(self.scan_sectors.right),
                'valid_count': int(self.scan_sectors.valid_count),
            }
        return {
            'stamp_s': round(float(now), 3),
            'mode': self.mode,
            'state': state,
            'reason': reason,
            'cmd': {
                'linear_x': round(float(linear), 4),
                'angular_z': round(float(angular), 4),
            },
            'pose': self._pose_payload(pose),
            'raw_pose': self._pose_payload(self.raw_pose),
            'goal': self._pose_payload(self.goal_pose),
            'goal_distance_m': (
                round(goal_distance, 4) if goal_distance is not None else None
            ),
            'path_waypoints': len(self.path_xy),
            'target_index': target_index,
            'heading_error_deg': (
                round(math.degrees(heading_error_rad), 2)
                if heading_error_rad is not None
                else None
            ),
            'front_clearance_m': self._finite_or_none(self.front_clearance_m),
            'front_emergency_m': self._finite_or_none(self.front_emergency_m),
            'scan_overlay_cells': int(self.scan_overlay_cells),
            'scan_age_s': self._finite_or_none(scan_age_s),
            'pose_age_s': self._finite_or_none(pose_age_s),
            'scan_sectors': sectors,
            'map_source': self.map_source,
            'perf': {
                'last_plan_duration_ms': self._finite_or_none(
                    self.last_plan_duration_ms
                ),
                'last_scan_overlay_duration_ms': self._finite_or_none(
                    self.last_scan_overlay_duration_ms
                ),
                'loop_period_ms': self._finite_or_none(self.loop_period_ms),
            },
        }

    def _publish_diagnostics(
        self,
        now,
        state,
        linear=0.0,
        angular=0.0,
        pose=None,
        target_index=None,
        heading_error_rad=None,
        scan_age_s=None,
        pose_age_s=None,
        reason='',
    ):
        payload = self._diagnostic_payload(
            now,
            state,
            linear=linear,
            angular=angular,
            pose=pose,
            target_index=target_index,
            heading_error_rad=heading_error_rad,
            scan_age_s=scan_age_s,
            pose_age_s=pose_age_s,
            reason=reason,
        )
        text = json.dumps(payload, sort_keys=True)
        msg = String()
        msg.data = text
        self.debug_pub.publish(msg)

        state_changed = state != self.last_debug_state
        period_elapsed = now - self.last_debug_log_time >= self.diagnostic_log_period_s
        if state_changed or period_elapsed:
            self.get_logger().info(f'nav_debug {text}')
            self.last_debug_log_time = now
            self.last_debug_state = state

    def _pose_from_msg(self, pose_msg):
        p = pose_msg.position
        return (float(p.x), float(p.y), yaw_from_quaternion(pose_msg.orientation))

    def _on_odom(self, msg):
        self.raw_pose = self._pose_from_msg(msg.pose.pose)
        self.raw_pose_time = self._node_time_s()

    def _on_pose_stamped(self, msg):
        self.raw_pose = self._pose_from_msg(msg.pose)
        self.raw_pose_time = self._node_time_s()

    def _on_initial_pose(self, msg):
        if self.raw_pose is None:
            self.get_logger().warn('initialpose recibido antes de pose_topic; espero pose')
            return
        self.anchor_raw_pose = self.raw_pose
        self.anchor_map_pose = self._pose_from_msg(msg.pose.pose)
        self.need_replan = self.goal_pose is not None
        self.follower.reset()
        self.get_logger().info(
            f'initialpose=({self.anchor_map_pose[0]:.2f}, '
            f'{self.anchor_map_pose[1]:.2f}, '
            f'{math.degrees(self.anchor_map_pose[2]):.1f} deg)'
        )

    def _on_goal_pose(self, msg):
        new_goal = self._pose_from_msg(msg.pose)
        if self.goal_pose is not None:
            same_xy = math.hypot(
                new_goal[0] - self.goal_pose[0],
                new_goal[1] - self.goal_pose[1],
            ) < 1e-3
            same_yaw = abs(wrap_angle(new_goal[2] - self.goal_pose[2])) < 1e-3
            if same_xy and same_yaw:
                return
        self.goal_pose = new_goal
        self.need_replan = True
        self.blocked_latched = False
        self.blocked_latched_reason = ''
        self.follower.reset()
        self._reset_goal_progress()
        self._reset_goal_regression()
        self.get_logger().info(
            f'goal=({self.goal_pose[0]:.2f}, {self.goal_pose[1]:.2f}, '
            f'{math.degrees(self.goal_pose[2]):.1f} deg)'
        )

    def _on_scan(self, msg):
        self.scan_sectors = analyze_scan(
            msg.ranges,
            float(msg.angle_min),
            float(msg.angle_increment),
            float(msg.range_min),
            float(msg.range_max),
        )
        self.front_clearance_m = self.scan_sectors.front
        self.front_emergency_m = self.scan_sectors.front_min
        self.last_scan_msg = msg
        self.last_scan_time = self._node_time_s()

    def _on_map(self, msg):
        if msg.info.width == 0 or msg.info.height == 0:
            self.get_logger().warn('map vacio recibido; se ignora')
            return
        data = np.asarray(msg.data, dtype=np.int16)
        expected = int(msg.info.width * msg.info.height)
        if data.size != expected:
            self.get_logger().warn(
                f'map con data invalida: {data.size} != {expected}; se ignora'
            )
            return
        occupancy = np.clip(data.reshape((msg.info.height, msg.info.width)), -1, 100).astype(
            np.int8
        )
        map_info = MapInfo(
            width=int(msg.info.width),
            height=int(msg.info.height),
            resolution=float(msg.info.resolution),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            origin_yaw=yaw_from_quaternion(msg.info.origin.orientation),
            frame_id=msg.header.frame_id or self.map_frame,
        )
        self.map_frame = map_info.frame_id
        self._set_map(
            occupancy,
            map_info,
            mark_replan=self.replan_on_map_update or not self.path_xy,
        )

    def _set_map(self, occupancy, map_info, mark_replan=True):
        self.occupancy = np.asarray(occupancy, dtype=np.int8)
        self.map_info = map_info
        self.grid_spec = GridSpec(
            resolution=self.map_info.resolution,
            origin_x=self.map_info.origin_x,
            origin_y=self.map_info.origin_y,
        )
        self.static_costmap = inflate_obstacles(
            self.occupancy,
            self.map_info.resolution,
            CostmapConfig(
                inflation_radius_m=float(
                    self.get_parameter('inflation_radius_m').value
                ),
                unknown_as_obstacle=bool(
                    self.get_parameter('unknown_as_obstacle').value
                ),
            ),
        )
        self.costmap = np.array(self.static_costmap, copy=True)
        if mark_replan and self.goal_pose is not None:
            self.need_replan = True

    def _update_scan_obstacle_overlay(self, pose, now):
        perf_start = time.perf_counter()
        if (
            not self.use_scan_obstacle_overlay
            or self.static_costmap is None
            or self.grid_spec is None
            or self.last_scan_msg is None
            or self.map_info is None
        ):
            return
        if now - self.last_scan_overlay_time < 0.25:
            return

        msg = self.last_scan_msg
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        if ranges.size == 0:
            return

        angle_min = float(msg.angle_min)
        angle_increment = float(msg.angle_increment)
        range_min = float(msg.range_min)
        range_max = min(
            float(msg.range_max),
            float(self.get_parameter('scan_obstacle_max_range_m').value),
        )
        stride = max(1, int(self.get_parameter('scan_obstacle_stride').value))
        min_cluster_size = max(
            1,
            int(self.get_parameter('scan_obstacle_min_cluster_size').value),
        )
        cluster_tolerance = max(
            0.0,
            float(self.get_parameter('scan_obstacle_cluster_tolerance_m').value),
        )

        overlay = np.zeros_like(self.static_costmap, dtype=np.int8)
        x, y, yaw = pose
        valid_indices = []
        for i in range(ranges.size):
            distance = float(ranges[i])
            if not math.isfinite(distance) or distance < range_min or distance > range_max:
                continue
            valid_indices.append(i)

        clusters = []
        current = []
        for i in valid_indices:
            if not current:
                current = [i]
                continue
            prev = current[-1]
            contiguous = i == prev + 1
            similar_range = abs(float(ranges[i]) - float(ranges[prev])) <= cluster_tolerance
            if contiguous and similar_range:
                current.append(i)
            else:
                if len(current) >= min_cluster_size:
                    clusters.append(current)
                current = [i]
        if len(current) >= min_cluster_size:
            clusters.append(current)

        marked_cells = 0
        for cluster in clusters:
            for offset, i in enumerate(cluster):
                if offset % stride != 0 and offset != len(cluster) - 1:
                    continue
                distance = float(ranges[i])
                beam_angle = yaw + angle_min + i * angle_increment
                gx, gy = world_to_grid(
                    x + math.cos(beam_angle) * distance,
                    y + math.sin(beam_angle) * distance,
                    self.grid_spec,
                )
                if 0 <= gy < overlay.shape[0] and 0 <= gx < overlay.shape[1]:
                    overlay[gy, gx] = 100
                    marked_cells += 1

        inflated_overlay = inflate_obstacles(
            overlay,
            self.map_info.resolution,
            CostmapConfig(
                inflation_radius_m=float(
                    self.get_parameter('scan_obstacle_inflation_radius_m').value
                ),
                unknown_as_obstacle=False,
            ),
        )
        self.costmap = np.array(self.static_costmap, dtype=np.int8, copy=True)
        self.costmap[inflated_overlay >= 50] = 100
        self.scan_overlay_cells = marked_cells
        self.last_scan_overlay_time = now
        self.last_scan_overlay_duration_ms = (time.perf_counter() - perf_start) * 1000.0

        replan_period = float(self.get_parameter('scan_obstacle_replan_period_s').value)
        should_replan = (
            self.goal_pose is not None
            and now - self.last_scan_replan_time >= replan_period
            and (
                self.front_clearance_m
                < float(self.get_parameter('obstacle_stop_distance_m').value)
                or self._path_hits_costmap()
            )
        )
        if should_replan:
            self.need_replan = True
            self.last_scan_replan_time = now

    def _path_hits_costmap(self):
        if self.costmap is None or self.grid_spec is None or not self.path_xy:
            return False
        for x, y in self.path_xy:
            gx, gy = world_to_grid(x, y, self.grid_spec)
            if not (0 <= gy < self.costmap.shape[0] and 0 <= gx < self.costmap.shape[1]):
                return True
            if int(self.costmap[gy, gx]) >= 50:
                return True
        return False

    def _estimated_pose(self):
        if self.raw_pose is None:
            return None
        if self.anchor_raw_pose is None:
            if self.require_initial_pose:
                return None
            return self.raw_pose
        rel = relative_pose(self.anchor_raw_pose, self.raw_pose)
        return compose_pose(self.anchor_map_pose, rel)

    def _make_grid_msg(self, grid):
        if grid is None or self.map_info is None:
            return None
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = float(self.map_info.resolution)
        msg.info.width = int(self.map_info.width)
        msg.info.height = int(self.map_info.height)
        msg.info.origin.position.x = float(self.map_info.origin_x)
        msg.info.origin.position.y = float(self.map_info.origin_y)
        msg.info.origin.orientation = quaternion_from_yaw(float(self.map_info.origin_yaw))
        msg.data = np.asarray(grid, dtype=np.int8).ravel().astype(int).tolist()
        return msg

    def _publish_loaded_map(self):
        if self.map_pub is None or not self.publish_loaded_map or self.occupancy is None:
            return
        msg = self._make_grid_msg(self.occupancy)
        if msg is None:
            return
        self.map_pub.publish(msg)
        self.last_map_pub = self._node_time_s()

    def _publish_costmap(self):
        if self.costmap is None:
            return
        msg = self._make_grid_msg(self.costmap)
        if msg is None:
            return
        self.costmap_pub.publish(msg)
        self.last_costmap_pub = self._node_time_s()

    def _publish_path(self, path_xy, goal_yaw):
        msg = PathMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for i, (x, y) in enumerate(path_xy):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            if i + 1 < len(path_xy):
                nx, ny = path_xy[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = goal_yaw
            pose.pose.orientation = quaternion_from_yaw(yaw)
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def _plan(self, pose):
        perf_start = time.perf_counter()
        if self.costmap is None or self.grid_spec is None:
            self.get_logger().warn('goal nav requiere mapa/costmap')
            self.last_plan_duration_ms = (time.perf_counter() - perf_start) * 1000.0
            return False
        if self.goal_pose is None:
            self.last_plan_duration_ms = (time.perf_counter() - perf_start) * 1000.0
            return False
        start = world_to_grid(pose[0], pose[1], self.grid_spec)
        goal = world_to_grid(self.goal_pose[0], self.goal_pose[1], self.grid_spec)
        start = nearest_free_cell(
            self.costmap,
            start,
            max_radius=self.goal_search_radius,
            allow_unknown=self.allow_unknown,
        )
        goal = nearest_free_cell(
            self.costmap,
            goal,
            max_radius=self.goal_search_radius,
            allow_unknown=self.allow_unknown,
        )
        if start is None or goal is None:
            self.get_logger().warn('start/goal fuera de espacio libre')
            self.path_xy = []
            self._publish_path([], 0.0)
            self.last_plan_duration_ms = (time.perf_counter() - perf_start) * 1000.0
            return False

        path_cells = astar(self.costmap, start, goal, allow_unknown=self.allow_unknown)
        if not path_cells:
            self.get_logger().warn(f'A* no encontro camino start={start} goal={goal}')
            self.path_xy = []
            self._publish_path([], 0.0)
            self.last_plan_duration_ms = (time.perf_counter() - perf_start) * 1000.0
            return False

        path_cells = limit_path_stride(path_cells, self.path_stride_cells)
        self.path_xy = cells_to_world_path(path_cells, self.grid_spec)
        self.path_xy[-1] = (self.goal_pose[0], self.goal_pose[1])
        self._publish_path(self.path_xy, self.goal_pose[2])
        self.need_replan = False
        self.follower.reset()
        self._reset_goal_progress()
        self.last_plan_duration_ms = (time.perf_counter() - perf_start) * 1000.0
        self.get_logger().info(f'planned_path: {len(self.path_xy)} waypoints')
        return True

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_cmd(self, linear, angular):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)

    def _avoidance_turn(self):
        if self.scan_sectors is None:
            return 0
        left = self.scan_sectors.left
        right = self.scan_sectors.right
        if not math.isfinite(left):
            left = 3.5
        if not math.isfinite(right):
            right = 3.5
        if min(left, right) > 0.35:
            return 0
        if abs(left - right) < 0.05:
            return 0
        return 1 if left > right else -1

    def _reset_goal_progress(self, now=None, pose=None, target_index=-1):
        self.progress_best_goal_distance = math.inf
        if pose is not None and self.goal_pose is not None:
            self.progress_best_goal_distance = math.hypot(
                self.goal_pose[0] - pose[0],
                self.goal_pose[1] - pose[1],
            )
        self.progress_last_time = 0.0 if now is None else float(now)
        self.progress_last_pose = pose
        self.progress_last_target_index = int(target_index)

    def _reset_goal_regression(self, now=None, pose=None):
        self.regression_best_goal_distance = math.inf
        if pose is not None and self.goal_pose is not None:
            self.regression_best_goal_distance = math.hypot(
                self.goal_pose[0] - pose[0],
                self.goal_pose[1] - pose[1],
            )
        self.regression_last_improvement_time = 0.0 if now is None else float(now)
        self.regression_last_improvement_pose = pose

    def _goal_regression_blocked(self, now, pose):
        if self.goal_pose is None:
            self._reset_goal_regression()
            return False

        distance_to_goal = math.hypot(
            self.goal_pose[0] - pose[0],
            self.goal_pose[1] - pose[1],
        )
        if (
            self.regression_last_improvement_pose is None
            or self.regression_last_improvement_time <= 0.0
        ):
            self._reset_goal_regression(now, pose)
            return False

        epsilon = float(self.get_parameter('goal_regression_epsilon_m').value)
        if distance_to_goal <= self.regression_best_goal_distance - epsilon:
            self.regression_best_goal_distance = distance_to_goal
            self.regression_last_improvement_time = float(now)
            self.regression_last_improvement_pose = pose
            return False

        if distance_to_goal <= float(self.get_parameter('goal_tolerance_m').value) + epsilon:
            return False

        timeout_s = float(self.get_parameter('goal_regression_timeout_s').value)
        min_motion = float(self.get_parameter('goal_regression_min_motion_m').value)
        moved_since_best = math.hypot(
            pose[0] - self.regression_last_improvement_pose[0],
            pose[1] - self.regression_last_improvement_pose[1],
        )
        return now - self.regression_last_improvement_time >= timeout_s and moved_since_best >= min_motion

    def _goal_progress_blocked(self, now, pose, target_index, require_motion=True):
        if self.goal_pose is None:
            self._reset_goal_progress()
            return False

        distance_to_goal = math.hypot(
            self.goal_pose[0] - pose[0],
            self.goal_pose[1] - pose[1],
        )
        if self.progress_last_pose is None or self.progress_last_time <= 0.0:
            self._reset_goal_progress(now, pose, target_index)
            return False

        epsilon = float(self.get_parameter('goal_progress_epsilon_m').value)
        target_step = int(self.get_parameter('goal_progress_target_step').value)
        improved_goal = distance_to_goal <= self.progress_best_goal_distance - epsilon
        advanced_path = target_index >= self.progress_last_target_index + target_step
        if improved_goal or advanced_path:
            self.progress_best_goal_distance = min(
                self.progress_best_goal_distance,
                distance_to_goal,
            )
            self.progress_last_time = float(now)
            self.progress_last_pose = pose
            self.progress_last_target_index = max(
                self.progress_last_target_index,
                int(target_index),
            )
            return False

        timeout_s = float(self.get_parameter('goal_progress_timeout_s').value)
        min_motion = float(self.get_parameter('goal_progress_min_motion_m').value)
        moved_since_progress = math.hypot(
            pose[0] - self.progress_last_pose[0],
            pose[1] - self.progress_last_pose[1],
        )
        return (
            now - self.progress_last_time >= timeout_s
            and (not require_motion or moved_since_progress >= min_motion)
        )

    def _run_safe_drive(self, now):
        if self.scan_sectors is None or self.last_scan_time is None:
            self._publish_stop()
            self._set_state(STATE_WATCHDOG_STOP)
            self._publish_diagnostics(
                now,
                STATE_WATCHDOG_STOP,
                pose=self._estimated_pose(),
                scan_age_s=None,
                reason='no_scan',
            )
            return
        scan_age = now - self.last_scan_time
        cmd = self.safe_drive.compute(self.scan_sectors, scan_age_s=scan_age)
        self._publish_cmd(cmd.linear, cmd.angular)
        self._set_state(cmd.state)
        self._publish_diagnostics(
            now,
            cmd.state,
            linear=cmd.linear,
            angular=cmd.angular,
            pose=self._estimated_pose(),
            scan_age_s=scan_age,
            reason='safe_drive',
        )

    def _run_goal_nav(self, now):
        pose = self._estimated_pose()
        if pose is None:
            self._publish_stop()
            self._set_state(STATE_IDLE)
            pose_age = math.inf if self.raw_pose_time is None else now - self.raw_pose_time
            self._publish_diagnostics(
                now,
                STATE_IDLE,
                pose=None,
                pose_age_s=pose_age,
                reason='no_pose',
            )
            return
        if self.goal_pose is None:
            self._publish_stop()
            self._set_state(STATE_IDLE)
            pose_age = math.inf if self.raw_pose_time is None else now - self.raw_pose_time
            scan_age = math.inf if self.last_scan_time is None else now - self.last_scan_time
            self._publish_diagnostics(
                now,
                STATE_IDLE,
                pose=pose,
                scan_age_s=scan_age,
                pose_age_s=pose_age,
                reason='no_goal',
            )
            return
        self._update_scan_obstacle_overlay(pose, now)
        if self.need_replan:
            if not self._plan(pose):
                self.blocked_latched = True
                self.blocked_latched_reason = 'plan_failed'
                self._publish_stop()
                self._set_state(STATE_BLOCKED_STOP)
                pose_age = math.inf if self.raw_pose_time is None else now - self.raw_pose_time
                scan_age = math.inf if self.last_scan_time is None else now - self.last_scan_time
                self._publish_diagnostics(
                    now,
                    STATE_BLOCKED_STOP,
                    pose=pose,
                    scan_age_s=scan_age,
                    pose_age_s=pose_age,
                    reason='plan_failed',
                )
                return

        if self.blocked_latched:
            self._publish_stop()
            self._set_state(STATE_BLOCKED_STOP)
            pose_age = math.inf if self.raw_pose_time is None else now - self.raw_pose_time
            scan_age = math.inf if self.last_scan_time is None else now - self.last_scan_time
            self._publish_diagnostics(
                now,
                STATE_BLOCKED_STOP,
                pose=pose,
                scan_age_s=scan_age,
                pose_age_s=pose_age,
                reason=self.blocked_latched_reason or 'blocked_latched',
            )
            return

        pose_age = math.inf if self.raw_pose_time is None else now - self.raw_pose_time
        scan_age = math.inf if self.last_scan_time is None else now - self.last_scan_time
        cmd = self.follower.compute(
            pose,
            self.path_xy,
            goal_yaw=self.goal_pose[2],
            front_clearance_m=self.front_clearance_m,
            front_emergency_m=self.front_emergency_m,
            avoidance_turn=self._avoidance_turn(),
            pose_age_s=pose_age,
            scan_age_s=scan_age,
        )
        progress_watch_states = (STATE_TRACK_PATH, STATE_STUCK_RECOVERY)
        regression_watch_states = (
            STATE_TRACK_PATH,
            STATE_STUCK_RECOVERY,
            STATE_ROTATE_TO_PATH,
        )
        if cmd.state in progress_watch_states and self._goal_progress_blocked(
            now,
            pose,
            int(cmd.target_index),
            require_motion=cmd.state == STATE_TRACK_PATH,
        ):
            self.blocked_latched = True
            self.blocked_latched_reason = 'no_progress'
            self._publish_stop()
            self._set_state(STATE_BLOCKED_STOP)
            self._publish_diagnostics(
                now,
                STATE_BLOCKED_STOP,
                linear=0.0,
                angular=0.0,
                pose=pose,
                target_index=int(cmd.target_index),
                heading_error_rad=cmd.heading_error_rad,
                scan_age_s=scan_age,
                pose_age_s=pose_age,
                reason='no_progress',
            )
            return
        if cmd.state in regression_watch_states and self._goal_regression_blocked(now, pose):
            self.blocked_latched = True
            self.blocked_latched_reason = 'goal_regression'
            self._publish_stop()
            self._set_state(STATE_BLOCKED_STOP)
            self._publish_diagnostics(
                now,
                STATE_BLOCKED_STOP,
                linear=0.0,
                angular=0.0,
                pose=pose,
                target_index=int(cmd.target_index),
                heading_error_rad=cmd.heading_error_rad,
                scan_age_s=scan_age,
                pose_age_s=pose_age,
                reason='goal_regression',
            )
            return

        self._publish_cmd(cmd.linear, cmd.angular)
        self._set_state(cmd.state)
        self._publish_diagnostics(
            now,
            cmd.state,
            linear=cmd.linear,
            angular=cmd.angular,
            pose=pose,
            target_index=int(cmd.target_index),
            heading_error_rad=cmd.heading_error_rad,
            scan_age_s=scan_age,
            pose_age_s=pose_age,
            reason='follow_path',
        )
        if cmd.state in (STATE_GOAL_REACHED, STATE_BLOCKED_STOP):
            self._publish_stop()

    def _on_timer(self):
        wall_now = time.perf_counter()
        if self.last_timer_wall_time is not None:
            self.loop_period_ms = (wall_now - self.last_timer_wall_time) * 1000.0
        self.last_timer_wall_time = wall_now

        now = self._node_time_s()
        if (
            self.publish_loaded_map
            and self.occupancy is not None
            and now - self.last_map_pub > 1.0
        ):
            self._publish_loaded_map()
        if self.costmap is not None and now - self.last_costmap_pub > 1.0:
            self._publish_costmap()

        if self.mode == 'safe_drive':
            self._run_safe_drive(now)
            return
        if self.mode == 'auto' and self.goal_pose is None:
            self._run_safe_drive(now)
            return
        self._run_goal_nav(now)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MazeNavNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            if rclpy.ok():
                try:
                    node._publish_stop()
                except Exception:
                    pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
