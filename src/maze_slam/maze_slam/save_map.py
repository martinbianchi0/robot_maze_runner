"""Save a nav_msgs/OccupancyGrid as PGM/YAML map files."""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Tuple

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from maze_slam.known_pose_mapper import quaternion_to_yaw


def occupancy_grid_to_image(map_msg: OccupancyGrid) -> np.ndarray:
    """Convert OccupancyGrid data to a map_server-style grayscale image."""
    data = np.asarray(map_msg.data, dtype=np.int16).reshape(
        (map_msg.info.height, map_msg.info.width)
    )
    image = np.full(data.shape, 205, dtype=np.uint8)
    image[data == 0] = 254
    image[data >= 65] = 0
    return np.flipud(image)


def write_pgm(path: str, image: np.ndarray) -> None:
    with open(path, "wb") as handle:
        header = "P5\n%d %d\n255\n" % (image.shape[1], image.shape[0])
        handle.write(header.encode("ascii"))
        handle.write(image.tobytes())


def write_yaml(path: str, pgm_filename: str, map_msg: OccupancyGrid) -> None:
    origin = map_msg.info.origin
    yaw = quaternion_to_yaw(
        origin.orientation.x,
        origin.orientation.y,
        origin.orientation.z,
        origin.orientation.w,
    )
    content = (
        "image: %s\n"
        "mode: trinary\n"
        "resolution: %.12g\n"
        "origin: [%.12g, %.12g, %.12g]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
        % (
            pgm_filename,
            map_msg.info.resolution,
            origin.position.x,
            origin.position.y,
            yaw,
        )
    )
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(content)


def save_map(map_msg: OccupancyGrid, output_prefix: str) -> Tuple[str, str]:
    output_dir = os.path.dirname(output_prefix)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    pgm_path = output_prefix + ".pgm"
    yaml_path = output_prefix + ".yaml"
    image = occupancy_grid_to_image(map_msg)
    write_pgm(pgm_path, image)
    write_yaml(yaml_path, os.path.basename(pgm_path), map_msg)
    return pgm_path, yaml_path


class MapSaverNode(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("maze_slam_map_saver")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_msg = None
        self.create_subscription(OccupancyGrid, topic, self.map_callback, qos)

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Save an OccupancyGrid map.")
    parser.add_argument("--output-prefix", required=True)
    parser.add_argument("--topic", default="/map")
    parser.add_argument("--timeout", type=float, default=10.0)
    return parser.parse_known_args(argv)


def main(argv=None) -> int:
    args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init(args=ros_args)
    node = MapSaverNode(args.topic)
    try:
        deadline = time.monotonic() + args.timeout
        while node.map_msg is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.map_msg is None:
            node.get_logger().error("Timed out waiting for map on %s" % args.topic)
            return 1
        pgm_path, yaml_path = save_map(node.map_msg, args.output_prefix)
        node.get_logger().info("Saved map: %s" % pgm_path)
        node.get_logger().info("Saved metadata: %s" % yaml_path)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
