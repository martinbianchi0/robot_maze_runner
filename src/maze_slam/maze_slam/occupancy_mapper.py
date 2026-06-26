#!/usr/bin/env python3
"""Etapa 1 - Mapeo con pose conocida (/calc_odom + /scan).

Gate de viabilidad: si con esta pose y el inverse sensor model el mapa
de custom_casa ya sale deforme, hay que arreglar frames/raycasting antes
de meter particulas.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose

from maze_slam.utils import (
    yaw_from_quaternion, logodds_to_occupancy, update_map_from_scan,
)


class OccupancyMapper(Node):

    def __init__(self):
        super().__init__('occupancy_mapper')

        # Parametros
        self.declare_parameter('map_size_m', 16.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('max_range', 3.5)
        self.declare_parameter('odom_topic', '/calc_odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_period_s', 1.0)

        size_m = float(self.get_parameter('map_size_m').value)
        self.res = float(self.get_parameter('resolution').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.map_frame = self.get_parameter('map_frame').value
        odom_topic = self.get_parameter('odom_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        map_topic = self.get_parameter('map_topic').value
        pub_period = float(self.get_parameter('publish_period_s').value)

        self.n = int(round(size_m / self.res))
        self.origin_x = -size_m / 2.0
        self.origin_y = -size_m / 2.0
        self.L = np.zeros((self.n, self.n), dtype=np.float32)

        self.pose = None  # (x, y, yaw) ultimo /calc_odom
        self.scan_angles = None
        self.scan_count = 0

        self.create_subscription(Odometry, odom_topic, self.cb_odom, 50)
        self.create_subscription(LaserScan, scan_topic, self.cb_scan,
                                 qos_profile_sensor_data)
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, map_topic, map_qos)
        self.create_timer(pub_period, self.publish_map)

        self.get_logger().info(
            f'occupancy_mapper: grid {self.n}x{self.n} @ {self.res} m, '
            f'origen=({self.origin_x:.1f},{self.origin_y:.1f}), '
            f'odom={odom_topic}, scan={scan_topic}'
        )

    def cb_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose = (x, y, yaw)

    def cb_scan(self, msg: LaserScan):
        if self.pose is None:
            return
        if self.scan_angles is None or len(self.scan_angles) != len(msg.ranges):
            self.scan_angles = np.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
        x, y, yaw = self.pose
        update_map_from_scan(self.L, x, y, yaw,
                             np.asarray(msg.ranges, dtype=np.float64),
                             self.scan_angles, self.max_range,
                             self.origin_x, self.origin_y, self.res)
        self.scan_count += 1

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.res
        msg.info.width = self.n
        msg.info.height = self.n
        msg.info.origin = Pose()
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        data = logodds_to_occupancy(self.L)
        msg.data = data.flatten().tolist()
        self.map_pub.publish(msg)
        self.get_logger().info(
            f'map publicado | scans={self.scan_count} | conocidas={(data != -1).sum()}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
