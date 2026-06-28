#!/usr/bin/env python3
"""Cliente ROS para validar safe_drive en una simulacion viva."""

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def pose_xy_from_odom(msg):
    p = msg.pose.pose.position
    return (float(p.x), float(p.y))


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class SafeDriveSmokeClient:
    def __init__(self, node):
        self.node = node
        self.odom_xy = None
        self.state = None
        self.state_since = time.monotonic()
        self.latest_debug = None
        self.latest_cmd = None
        self.min_front_emergency = math.inf

        node.create_subscription(Odometry, '/odom', self._on_odom, 10)
        node.create_subscription(String, '/nav_state', self._on_state, 10)
        node.create_subscription(String, '/nav_debug', self._on_debug, 10)
        node.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)

    def _on_odom(self, msg):
        self.odom_xy = pose_xy_from_odom(msg)

    def _on_state(self, msg):
        if msg.data != self.state:
            self.state = msg.data
            self.state_since = time.monotonic()

    def _on_debug(self, msg):
        try:
            self.latest_debug = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_debug = {'raw': msg.data}
        front = self.latest_debug.get('front_emergency_m')
        if front is not None and math.isfinite(float(front)):
            self.min_front_emergency = min(self.min_front_emergency, float(front))

    def _on_cmd(self, msg):
        self.latest_cmd = (float(msg.linear.x), float(msg.angular.z))

    def wait_for_odom(self, timeout_s=20.0):
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.odom_xy is not None:
                return True
        return False

    def has_valid_scan_debug(self):
        if not self.latest_debug:
            return False
        if self.latest_debug.get('scan_sectors') is None:
            return False
        front = self.latest_debug.get('front_emergency_m')
        return front is not None and math.isfinite(float(front))

    def wait_until_ready(self, timeout_s):
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.odom_xy is not None and self.has_valid_scan_debug():
                return True
        return False

    def run(
        self,
        duration_s,
        min_move_m,
        min_front_emergency_m,
        bad_state_s,
        ready_timeout_s,
    ):
        if not self.wait_until_ready(ready_timeout_s):
            return False, {
                'result': 'NOT_READY',
                'odom_ready': self.odom_xy is not None,
                'scan_ready': self.has_valid_scan_debug(),
                'state': self.state,
                'cmd': self.latest_cmd,
                'debug': self.latest_debug,
            }

        start_xy = self.odom_xy
        start_time = time.monotonic()
        last_print = 0.0

        while time.monotonic() - start_time < duration_s:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            now = time.monotonic()
            elapsed = now - start_time
            moved = distance(start_xy, self.odom_xy) if self.odom_xy else 0.0

            if elapsed - last_print >= 2.0:
                last_print = elapsed
                print(
                    'progress safe_drive t={:.1f}s state={} moved={:.3f} '
                    'cmd={} debug={}'.format(
                        elapsed,
                        self.state,
                        moved,
                        self.latest_cmd,
                        json.dumps(self.latest_debug, sort_keys=True)
                        if self.latest_debug else None,
                    ),
                    flush=True,
                )

            if self.min_front_emergency < min_front_emergency_m:
                return False, {
                    'result': 'FRONT_EMERGENCY_TOO_CLOSE',
                    'elapsed_s': round(elapsed, 2),
                    'moved_m': round(moved, 3),
                    'min_front_emergency_m': round(self.min_front_emergency, 3),
                    'state': self.state,
                    'cmd': self.latest_cmd,
                    'debug': self.latest_debug,
                }

            if self.state in {'WATCHDOG_STOP', 'BLOCKED_STOP'}:
                if now - self.state_since >= bad_state_s:
                    return False, {
                        'result': self.state,
                        'elapsed_s': round(elapsed, 2),
                        'state_duration_s': round(now - self.state_since, 2),
                        'moved_m': round(moved, 3),
                        'cmd': self.latest_cmd,
                        'debug': self.latest_debug,
                    }

        moved = distance(start_xy, self.odom_xy) if self.odom_xy else 0.0
        ok = moved >= min_move_m
        return ok, {
            'result': 'SAFE_DRIVE_OK' if ok else 'NOT_ENOUGH_MOTION',
            'duration_s': round(duration_s, 2),
            'moved_m': round(moved, 3),
            'min_front_emergency_m': round(self.min_front_emergency, 3),
            'state': self.state,
            'cmd': self.latest_cmd,
            'debug': self.latest_debug,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration-s', type=float, default=30.0)
    parser.add_argument('--min-move-m', type=float, default=0.25)
    parser.add_argument('--min-front-emergency-m', type=float, default=0.13)
    parser.add_argument('--bad-state-s', type=float, default=4.0)
    parser.add_argument('--ready-timeout-s', type=float, default=25.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('maze_nav_smoke_safe_drive_client')
    client = SafeDriveSmokeClient(node)
    try:
        ok, result = client.run(
            args.duration_s,
            args.min_move_m,
            args.min_front_emergency_m,
            args.bad_state_s,
            args.ready_timeout_s,
        )
        print('summary ' + json.dumps(result, sort_keys=True), flush=True)
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
