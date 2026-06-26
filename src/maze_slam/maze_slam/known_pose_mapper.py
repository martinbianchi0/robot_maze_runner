"""ROS 2 node that builds an occupancy grid from /calc_odom and /scan."""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan

from maze_slam.grid_mapping import OccupancyGridMapper


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return yaw from a quaternion."""
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + (float(stamp.nanosec) * 1e-9)


class KnownPoseMapperNode(Node):
    """Build and publish a map using the current odometry as known pose."""

    def __init__(self) -> None:
        super().__init__("known_pose_mapper")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/calc_odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("path_topic", "/known_pose_path")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("width_m", 12.0)
        self.declare_parameter("height_m", 12.0)
        self.declare_parameter("origin_x", -6.0)
        self.declare_parameter("origin_y", -6.0)
        self.declare_parameter("occupied_probability", 0.70)
        self.declare_parameter("free_probability", 0.40)
        self.declare_parameter("occupied_threshold", 0.65)
        self.declare_parameter("free_threshold", 0.35)
        self.declare_parameter("min_log_odds", -5.0)
        self.declare_parameter("max_log_odds", 5.0)
        self.declare_parameter("ray_stride", 1)
        self.declare_parameter("max_range", 0.0)
        self.declare_parameter("hit_range_margin", 0.02)
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("odom_timeout_sec", 0.5)
        self.declare_parameter("max_path_poses", 2500)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.map_topic = self.get_parameter("map_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.occupied_threshold = float(self.get_parameter("occupied_threshold").value)
        self.free_threshold = float(self.get_parameter("free_threshold").value)
        self.ray_stride = int(self.get_parameter("ray_stride").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.hit_range_margin = float(self.get_parameter("hit_range_margin").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.odom_timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        self.max_path_poses = int(self.get_parameter("max_path_poses").value)

        self.mapper = OccupancyGridMapper(
            width_m=float(self.get_parameter("width_m").value),
            height_m=float(self.get_parameter("height_m").value),
            resolution=float(self.get_parameter("resolution").value),
            origin_x=float(self.get_parameter("origin_x").value),
            origin_y=float(self.get_parameter("origin_y").value),
            occupied_probability=float(self.get_parameter("occupied_probability").value),
            free_probability=float(self.get_parameter("free_probability").value),
            min_log_odds=float(self.get_parameter("min_log_odds").value),
            max_log_odds=float(self.get_parameter("max_log_odds").value),
        )

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        default_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self.path_pub = self.create_publisher(Path, self.path_topic, map_qos)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, default_qos)

        self.last_pose: Optional[Tuple[float, float, float]] = None
        self.last_odom_stamp_sec: Optional[float] = None
        self.last_odom_received_monotonic: Optional[float] = None
        self.last_warn_monotonic = 0.0
        self.last_info_monotonic = 0.0
        self.last_publish_nanoseconds: Optional[int] = None
        self.publish_period_nanoseconds = (
            int(1e9 / self.publish_hz) if self.publish_hz > 0.0 else 0
        )
        self.scan_metadata: Optional[Tuple[int, float, float]] = None
        self.local_cosines = np.array([], dtype=np.float32)
        self.local_sines = np.array([], dtype=np.float32)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame

        self.get_logger().info(
            "known_pose_mapper ready: scan=%s odom=%s map=%s path=%s size=%dx%d res=%.3f stride=%d"
            % (
                self.scan_topic,
                self.odom_topic,
                self.map_topic,
                self.path_topic,
                self.mapper.width,
                self.mapper.height,
                self.mapper.resolution,
                self.ray_stride,
            )
        )

    def odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self.last_pose = (pose.position.x, pose.position.y, yaw)
        self.last_odom_stamp_sec = stamp_to_seconds(msg.header.stamp)
        self.last_odom_received_monotonic = time.monotonic()

    def scan_callback(self, msg: LaserScan) -> None:
        if self.last_pose is None:
            self.warn_throttled("Skipping scan: no /calc_odom message received yet")
            return
        if self.odom_is_stale(msg):
            self.warn_throttled("Skipping scan: latest /calc_odom is stale")
            return

        self.ensure_scan_trig_cache(msg)
        stats = self.mapper.update_scan(
            self.last_pose,
            msg.ranges,
            self.local_cosines,
            self.local_sines,
            msg.range_min,
            msg.range_max,
            self.max_range,
            ray_stride=self.ray_stride,
            hit_range_margin=self.hit_range_margin,
        )
        self.append_path_pose(msg)

        now = self.get_clock().now()
        if self.should_publish(now.nanoseconds):
            self.publish_map_and_path(msg)
            self.last_publish_nanoseconds = now.nanoseconds

        if time.monotonic() - self.last_info_monotonic > 5.0:
            self.last_info_monotonic = time.monotonic()
            self.get_logger().info(
                "map update: beams=%d hits=%d free_cells=%d"
                % (stats.beams_used, stats.hit_updates, stats.free_updates)
            )

    def odom_is_stale(self, scan_msg: LaserScan) -> bool:
        if self.last_odom_received_monotonic is None:
            return True
        scan_stamp_sec = stamp_to_seconds(scan_msg.header.stamp)
        if scan_stamp_sec > 0.0 and self.last_odom_stamp_sec is not None:
            return abs(scan_stamp_sec - self.last_odom_stamp_sec) > self.odom_timeout_sec
        return (time.monotonic() - self.last_odom_received_monotonic) > self.odom_timeout_sec

    def ensure_scan_trig_cache(self, msg: LaserScan) -> None:
        metadata = (len(msg.ranges), float(msg.angle_min), float(msg.angle_increment))
        if metadata == self.scan_metadata:
            return
        self.scan_metadata = metadata
        angles = msg.angle_min + (np.arange(len(msg.ranges), dtype=np.float32) * msg.angle_increment)
        self.local_cosines = np.cos(angles).astype(np.float32)
        self.local_sines = np.sin(angles).astype(np.float32)
        self.get_logger().info("cached %d laser beam angles" % len(msg.ranges))

    def should_publish(self, now_nanoseconds: int) -> bool:
        if self.publish_period_nanoseconds <= 0:
            return True
        if self.last_publish_nanoseconds is None:
            return True
        return (now_nanoseconds - self.last_publish_nanoseconds) >= self.publish_period_nanoseconds

    def append_path_pose(self, scan_msg: LaserScan) -> None:
        if self.last_pose is None:
            return
        pose_msg = PoseStamped()
        pose_msg.header.stamp = scan_msg.header.stamp
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose.position.x = self.last_pose[0]
        pose_msg.pose.position.y = self.last_pose[1]
        pose_msg.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(self.last_pose[2])
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.path_msg.poses.append(pose_msg)
        if self.max_path_poses > 0 and len(self.path_msg.poses) > self.max_path_poses:
            self.path_msg.poses = self.path_msg.poses[-self.max_path_poses :]

    def publish_map_and_path(self, scan_msg: LaserScan) -> None:
        map_msg = OccupancyGrid()
        map_msg.header.stamp = scan_msg.header.stamp
        map_msg.header.frame_id = self.map_frame
        map_msg.info.resolution = self.mapper.resolution
        map_msg.info.width = self.mapper.width
        map_msg.info.height = self.mapper.height
        map_msg.info.origin.position.x = self.mapper.origin_x
        map_msg.info.origin.position.y = self.mapper.origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = self.mapper.to_occupancy_grid_data(
            occupied_probability_threshold=self.occupied_threshold,
            free_probability_threshold=self.free_threshold,
        ).ravel().tolist()
        self.map_pub.publish(map_msg)

        self.path_msg.header.stamp = scan_msg.header.stamp
        self.path_msg.header.frame_id = self.map_frame
        self.path_pub.publish(self.path_msg)

    def warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_warn_monotonic > 2.0:
            self.last_warn_monotonic = now
            self.get_logger().warning(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KnownPoseMapperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
