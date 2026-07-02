#!/usr/bin/env python3
"""Mock de percepcion para el test de mision C5.

Publica cone_detections + un LaserScan sintetico que ubican el cono en un punto
de mundo FIJO (target), calculado desde la pose actual (/amcl_pose). Permite
validar la FSM de mision sin percepcion real, incluyendo el caso 'cono detras de
pared' (target sobre un obstaculo del mapa: el LIDAR-fusion cae sobre la pared ->
la FSM lo rechaza).

Uso: python scripts/mock_cone_publisher.py --ros-args \
        -p target_x:=0.71 -p target_y:=-1.12
"""
import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                'src', 'maze_perception'))
from maze_perception.detections import ConeDetection, ConeDetections  # noqa: E402


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class MockCone(Node):

    def __init__(self):
        super().__init__('mock_cone_publisher')
        self.tx = float(self.declare_parameter('target_x', 0.0).value)
        self.ty = float(self.declare_parameter('target_y', 0.0).value)
        self.area = int(self.declare_parameter('area_px', 2500).value)
        self.n = int(self.declare_parameter('n_beams', 360).value)
        det_topic = self.declare_parameter('detections_topic', 'cone_detections').value
        scan_topic = self.declare_parameter('scan_topic', '/scan').value
        pose_topic = self.declare_parameter('pose_topic', '/amcl_pose').value

        self.pose = None
        self.det_pub = self.create_publisher(String, det_topic, 10)
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self._on_pose, 10)
        self.create_timer(0.1, self._step)
        self.get_logger().info(f'mock_cone en target=({self.tx:.2f},{self.ty:.2f})')

    def _on_pose(self, msg):
        p = msg.pose.pose
        yaw = math.atan2(2 * (p.orientation.w * p.orientation.z),
                         1 - 2 * p.orientation.z * p.orientation.z)
        self.pose = (p.position.x, p.position.y, yaw)

    def _step(self):
        if self.pose is None:
            return
        px, py, pyaw = self.pose
        dist = math.hypot(self.tx - px, self.ty - py)
        bearing = wrap(math.atan2(self.ty - py, self.tx - px) - pyaw)
        now = self.get_clock().now().to_msg()

        det = ConeDetections(
            stamp_s=now.sec + now.nanosec * 1e-9, frame_id='camera',
            image_width=250, image_height=250,
            detections=[ConeDetection(bearing_rad=bearing, u=125, v=125,
                                      area_px=self.area, confidence=0.9, color='red')])
        self.det_pub.publish(String(data=det.to_json()))

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2 * math.pi / self.n
        scan.range_min = 0.1
        scan.range_max = 12.0
        ranges = [12.0] * self.n
        idx = int(round((bearing - scan.angle_min) / scan.angle_increment))
        for k in range(-2, 3):
            ranges[(idx + k) % self.n] = dist
        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = MockCone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
