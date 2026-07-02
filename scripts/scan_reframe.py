#!/usr/bin/env python3
"""Shim de reencuadre del LaserScan para el LIDAR real (Parte C).

El RPLIDAR del TB4 real esta montado a +90 deg respecto de base (ver
docs/decisiones/PARTE_C_ESTIMACION_CONO.md), pero el localizer/navigator de
toma-2 asumen el scan alineado con el frente. Este nodo republica el scan con
`angle_min`/`angle_max` corridos por `yaw_offset` (default +pi/2), sin tocar
maze_nav: el scan reencuadrado se puede alimentar al localizer para que la
correccion MCL matchee el mapa. Es la alternativa no invasiva al fix propio
(agregar scan_yaw_offset al localizer), a coordinar con el equipo.

Uso: python scripts/scan_reframe.py --ros-args \
        -p in_topic:=/scan_raw -p out_topic:=/scan -p yaw_offset:=1.5708
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanReframe(Node):

    def __init__(self):
        super().__init__('scan_reframe')
        in_topic = self.declare_parameter('in_topic', '/scan_raw').value
        self.out_topic = self.declare_parameter('out_topic', '/scan').value
        self.yaw_offset = float(self.declare_parameter('yaw_offset', 1.5707963).value)
        self.pub = self.create_publisher(LaserScan, self.out_topic, qos_profile_sensor_data)
        self.create_subscription(LaserScan, in_topic, self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f'scan_reframe {in_topic} -> {self.out_topic} (yaw_offset={self.yaw_offset:.4f})')

    def _on_scan(self, msg: LaserScan):
        msg.angle_min += self.yaw_offset
        msg.angle_max += self.yaw_offset
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanReframe()
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
