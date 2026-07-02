#!/usr/bin/env python3
"""Markers RViz para demo/replay del laboratorio.

Funciona tanto en vivo como con rosbag play --clock. No usa mensajes custom:
consume estados/debug como std_msgs/String y publica MarkerArray.
"""
from __future__ import annotations

import argparse
import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    msg = ColorRGBA()
    msg.r = float(r)
    msg.g = float(g)
    msg.b = float(b)
    msg.a = float(a)
    return msg


def point(x: float, y: float, z: float = 0.0) -> Point:
    msg = Point()
    msg.x = float(x)
    msg.y = float(y)
    msg.z = float(z)
    return msg


class LabVizMarkers(Node):
    def __init__(self, ns: str):
        super().__init__('lab_viz_markers')
        self.ns = ns.strip('/')
        self.pose: Optional[tuple[float, float, float]] = None
        self.traj: list[tuple[float, float]] = []
        self.goal: Optional[tuple[float, float]] = None
        self.path: list[tuple[float, float]] = []
        self.nav_state = '?'
        self.mission_state = '?'
        self.nav_debug = {}
        self.best_cone_bearing: Optional[float] = None
        self.best_cone_confidence: Optional[float] = None

        self.pub = self.create_publisher(MarkerArray, '/lab_viz/markers', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 30)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(String, '/nav_state', self.on_nav_state, 20)
        self.create_subscription(String, '/mission_state', self.on_mission_state, 20)
        self.create_subscription(String, '/nav_debug', self.on_nav_debug, 20)
        self.create_subscription(String, '/cone_detections', self.on_cone_detections, 20)
        self.create_timer(0.2, self.publish_markers)
        self.get_logger().info(f'lab_viz_markers listo ns={self.ns or "(global)"}')

    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation)
        self.pose = (float(p.position.x), float(p.position.y), yaw)
        self.traj.append((self.pose[0], self.pose[1]))
        if len(self.traj) > 5000:
            self.traj = self.traj[-5000:]

    def on_goal(self, msg: PoseStamped) -> None:
        self.goal = (float(msg.pose.position.x), float(msg.pose.position.y))

    def on_plan(self, msg: Path) -> None:
        self.path = [(float(ps.pose.position.x), float(ps.pose.position.y))
                     for ps in msg.poses]

    def on_nav_state(self, msg: String) -> None:
        self.nav_state = msg.data

    def on_mission_state(self, msg: String) -> None:
        self.mission_state = msg.data

    def on_nav_debug(self, msg: String) -> None:
        try:
            self.nav_debug = json.loads(msg.data)
        except json.JSONDecodeError:
            self.nav_debug = {'reason': msg.data}

    def on_cone_detections(self, msg: String) -> None:
        try:
            raw = json.loads(msg.data)
            detections = raw.get('detections', [])
            if not detections:
                self.best_cone_bearing = None
                self.best_cone_confidence = None
                return
            best = max(detections, key=lambda d: (
                float(d.get('confidence', 0.0)),
                float(d.get('area_px', 0.0)),
            ))
            self.best_cone_bearing = float(best.get('bearing_rad', 0.0))
            self.best_cone_confidence = float(best.get('confidence', 0.0))
        except (json.JSONDecodeError, TypeError, ValueError):
            self.best_cone_bearing = None
            self.best_cone_confidence = None

    def base_marker(self, marker_id: int, marker_type: int, ns: str) -> Marker:
        msg = Marker()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = ns
        msg.id = marker_id
        msg.type = marker_type
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        return msg

    def line_marker(self, marker_id: int, ns: str, pts, rgba: ColorRGBA,
                    width: float, z: float = 0.04) -> Marker:
        msg = self.base_marker(marker_id, Marker.LINE_STRIP, ns)
        msg.scale.x = float(width)
        msg.color = rgba
        msg.points = [point(x, y, z) for x, y in pts]
        return msg

    def publish_markers(self) -> None:
        arr = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        arr.markers.append(delete)

        if len(self.traj) >= 2:
            arr.markers.append(self.line_marker(
                1, 'trajectory', self.traj, color(0.05, 0.35, 1.0, 1.0), 0.035, 0.05))

        if self.path:
            arr.markers.append(self.line_marker(
                2, 'plan', self.path, color(1.0, 0.0, 0.75, 1.0), 0.025, 0.08))

        if self.goal is not None:
            msg = self.base_marker(3, Marker.SPHERE, 'goal')
            msg.pose.position.x = self.goal[0]
            msg.pose.position.y = self.goal[1]
            msg.pose.position.z = 0.18
            msg.scale.x = 0.22
            msg.scale.y = 0.22
            msg.scale.z = 0.22
            msg.color = color(0.0, 0.8, 0.2, 0.9)
            arr.markers.append(msg)

        if self.pose is not None:
            x, y, yaw = self.pose
            reason = str(self.nav_debug.get('reason', self.nav_debug.get('note', '?')))
            clearance = self.nav_debug.get('forward_clearance_m')
            rec = self.nav_debug.get('recovery_attempts', 0)
            rec_max = self.nav_debug.get('max_recovery_attempts', 0)
            dyn = self.nav_debug.get('dyn_obstacle_cells', 0)
            try:
                clearance_text = f'{float(clearance):.2f}m'
            except (TypeError, ValueError):
                clearance_text = '?'
            text = (
                f'NAV={self.nav_state}\n'
                f'MISSION={self.mission_state}\n'
                f'reason={reason}\n'
                f'clearance={clearance_text} rec={rec}/{rec_max} dyn={dyn}'
            )
            msg = self.base_marker(4, Marker.TEXT_VIEW_FACING, 'status_text')
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = 0.72
            msg.scale.z = 0.22
            msg.text = text
            danger = any(token in reason for token in (
                'blocked', 'recovery', 'front_blocked', 'blocked_max_recovery_attempts'))
            msg.color = color(1.0, 0.15, 0.05, 1.0) if danger else color(1.0, 1.0, 1.0, 1.0)
            arr.markers.append(msg)

            robot = self.base_marker(5, Marker.ARROW, 'pose_arrow')
            robot.pose.position.x = x
            robot.pose.position.y = y
            robot.pose.position.z = 0.12
            robot.pose.orientation.z = math.sin(yaw / 2.0)
            robot.pose.orientation.w = math.cos(yaw / 2.0)
            robot.scale.x = 0.38
            robot.scale.y = 0.08
            robot.scale.z = 0.08
            robot.color = color(0.0, 0.9, 1.0, 0.9)
            arr.markers.append(robot)

            if self.best_cone_bearing is not None:
                bearing = yaw + self.best_cone_bearing
                ray = [(x, y), (x + 0.80 * math.cos(bearing), y + 0.80 * math.sin(bearing))]
                arr.markers.append(self.line_marker(
                    6, 'cone_ray', ray, color(1.0, 0.45, 0.0, 0.95), 0.035, 0.16))

        self.pub.publish(arr)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ns', default='tb4_0')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = LabVizMarkers(args.ns)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
