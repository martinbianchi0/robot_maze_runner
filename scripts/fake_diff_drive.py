#!/usr/bin/env python3
"""Mini-sim cinematico (unicycle) para cerrar el lazo de navegacion sin Gazebo.

Integra /cmd_vel y publica /amcl_pose (la pose que consume el navigator de
toma-2). NO usa /scan: el navigator planifica y sigue sobre el mapa estatico (la
evitacion por LIDAR se valida aparte). Sirve para probar el contrato
/goal_pose -> /nav_state -> REACHED (etapa C4) de forma reproducible.

Uso: python scripts/fake_diff_drive.py --ros-args -p init_x:=2.5 -p init_y:=-2.0
"""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node


class FakeDiffDrive(Node):

    def __init__(self):
        super().__init__('fake_diff_drive')
        self.x = float(self.declare_parameter('init_x', 0.0).value)
        self.y = float(self.declare_parameter('init_y', 0.0).value)
        self.yaw = float(self.declare_parameter('init_yaw', 0.0).value)
        rate = float(self.declare_parameter('rate', 30.0).value)
        cmd_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        pose_topic = self.declare_parameter('pose_topic', '/amcl_pose').value

        self.v = 0.0
        self.w = 0.0
        self.dt = 1.0 / rate
        self.create_subscription(Twist, cmd_topic, self._on_cmd, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)
        self.create_timer(self.dt, self._step)
        self.get_logger().info(
            f'fake_diff_drive en ({self.x:.2f},{self.y:.2f}) '
            f'{cmd_topic} -> {pose_topic}')

    def _on_cmd(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def _step(self):
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw = (self.yaw + self.w * self.dt + math.pi) % (2 * math.pi) - math.pi
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeDiffDrive()
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
