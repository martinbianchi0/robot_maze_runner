#!/usr/bin/env python3
"""Publica markers de RViz para la mision C5: paredes del mapa, robot, cono, goal
y estado. Las paredes van como CUBE_LIST de markers para NO usar el display Map de
RViz (que tiene un bug de GLSL en macOS). Todo en frame 'map'.

Uso: python scripts/rviz_markers.py --ros-args -p cone_x:=4.04 -p cone_y:=-2.17
"""
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'maze_nav'))

import rclpy  # noqa: E402
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import ColorRGBA, String  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

from maze_nav.nav_utils import load_map  # noqa: E402


def color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


class RvizMarkers(Node):

    def __init__(self):
        super().__init__('rviz_markers')
        self.cone_x = float(self.declare_parameter('cone_x', 0.0).value)
        self.cone_y = float(self.declare_parameter('cone_y', 0.0).value)
        map_yaml = self.declare_parameter('map_yaml', 'maps/maze_slam.yaml').value

        m = load_map(os.path.join(ROOT, map_yaml))
        occ, res = m['occ'], m['res']
        ox, oy = m['origin']
        ys, xs = np.where(occ == 100)
        self.res = res
        self.wall_pts = [Point(x=float(ox + (gx + 0.5) * res),
                               y=float(oy + (gy + 0.5) * res), z=0.0)
                         for gx, gy in zip(xs.tolist(), ys.tolist())]

        self.pose = None
        self.goal = None
        self.state = ''
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 20)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self.create_subscription(String, '/mission_state', self._on_state, 10)
        self.pub = self.create_publisher(MarkerArray, '/viz_markers', 1)
        self.create_timer(0.1, self._fast)
        self.create_timer(0.5, self._slow)
        self.get_logger().info(
            f'rviz_markers: {len(self.wall_pts)} celdas de pared, cono en '
            f'({self.cone_x:.2f},{self.cone_y:.2f})')

    def _on_pose(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y,
                     math.atan2(2 * p.orientation.w * p.orientation.z,
                                1 - 2 * p.orientation.z ** 2))

    def _on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def _on_state(self, msg):
        self.state = msg.data

    def _base(self, ns, mid, mtype):
        mk = Marker()
        mk.header.frame_id = 'map'
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = ns
        mk.id = mid
        mk.type = mtype
        mk.action = Marker.ADD
        mk.pose.orientation.w = 1.0
        return mk

    def _slow(self):
        arr = MarkerArray()
        walls = self._base('walls', 0, Marker.CUBE_LIST)
        walls.scale.x = walls.scale.y = self.res
        walls.scale.z = 0.1
        walls.color = color(0.25, 0.25, 0.28)
        walls.points = self.wall_pts
        arr.markers.append(walls)

        cone = self._base('cono', 1, Marker.CYLINDER)
        cone.pose.position.x = self.cone_x
        cone.pose.position.y = self.cone_y
        cone.pose.position.z = 0.15
        cone.scale.x = cone.scale.y = 0.16
        cone.scale.z = 0.30
        cone.color = color(1.0, 0.45, 0.0)
        arr.markers.append(cone)
        self.pub.publish(arr)

    def _fast(self):
        arr = MarkerArray()
        if self.pose is not None:
            x, y, yaw = self.pose
            robot = self._base('robot', 2, Marker.ARROW)
            robot.pose.position.x = x
            robot.pose.position.y = y
            robot.pose.position.z = 0.05
            robot.pose.orientation.z = math.sin(yaw / 2)
            robot.pose.orientation.w = math.cos(yaw / 2)
            robot.scale.x = 0.35
            robot.scale.y = robot.scale.z = 0.08
            robot.color = color(0.1, 0.3, 1.0)
            arr.markers.append(robot)

            txt = self._base('estado', 4, Marker.TEXT_VIEW_FACING)
            txt.pose.position.x = x
            txt.pose.position.y = y
            txt.pose.position.z = 0.6
            txt.scale.z = 0.3
            txt.color = color(1.0, 1.0, 1.0)
            txt.text = self.state
            arr.markers.append(txt)
        if self.goal is not None:
            goal = self._base('goal', 3, Marker.SPHERE)
            goal.pose.position.x = self.goal[0]
            goal.pose.position.y = self.goal[1]
            goal.pose.position.z = 0.1
            goal.scale.x = goal.scale.y = goal.scale.z = 0.18
            goal.color = color(0.1, 0.9, 0.1)
            arr.markers.append(goal)
        if arr.markers:
            self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = RvizMarkers()
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
