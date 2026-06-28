#!/usr/bin/env python3
"""Cliente ROS para validar navegacion a goals en una simulacion viva."""

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def pose_xy_from_odom(msg):
    p = msg.pose.pose.position
    return (float(p.x), float(p.y))


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def path_length_m(msg):
    points = [
        (float(pose.pose.position.x), float(pose.pose.position.y))
        for pose in msg.poses
    ]
    if len(points) < 2:
        return 0.0
    return sum(distance(a, b) for a, b in zip(points, points[1:]))


def parse_goal(text):
    parts = [float(part.strip()) for part in text.split(',')]
    if len(parts) == 2:
        parts.append(0.0)
    if len(parts) != 3:
        raise argparse.ArgumentTypeError('goal debe ser x,y o x,y,yaw_deg')
    return (parts[0], parts[1], math.radians(parts[2]))


def parse_goal_spec(text):
    expected = 'reached'
    raw_goal = text
    if text.startswith('blocked:'):
        expected = 'blocked'
        raw_goal = text[len('blocked:'):]
    return {
        'raw': text,
        'expected': expected,
        'goal': parse_goal(raw_goal),
    }


class SmokeClient:
    def __init__(
        self,
        node,
        timeout_s,
        block_fail_s,
        blocked_max_move_m,
        reached_max_efficiency,
        reached_extra_m,
    ):
        self.node = node
        self.timeout_s = timeout_s
        self.block_fail_s = block_fail_s
        self.blocked_max_move_m = blocked_max_move_m
        self.reached_max_efficiency = reached_max_efficiency
        self.reached_extra_m = reached_extra_m
        self.odom_xy = None
        self.state = None
        self.latest_debug = None
        self.latest_cmd = None
        self.path_len = 0
        self.latest_path_length_m = 0.0
        self.state_since = time.monotonic()

        self.goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
        node.create_subscription(Odometry, '/odom', self._on_odom, 10)
        node.create_subscription(String, '/nav_state', self._on_state, 10)
        node.create_subscription(String, '/nav_debug', self._on_debug, 10)
        node.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        node.create_subscription(Path, '/planned_path', self._on_path, 10)

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

    def _on_cmd(self, msg):
        self.latest_cmd = (float(msg.linear.x), float(msg.angular.z))

    def _on_path(self, msg):
        self.path_len = len(msg.poses)
        self.latest_path_length_m = path_length_m(msg)

    def wait_for_odom(self, timeout_s=20.0):
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.odom_xy is not None:
                return True
        return False

    def publish_goal(self, goal):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        qx, qy, qz, qw = yaw_to_quaternion(goal[2])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        for _ in range(10):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.goal_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def run_goal(self, goal_spec, index):
        if not self.wait_for_odom():
            return False, {'error': 'no_odom'}

        goal = goal_spec['goal']
        expected = goal_spec['expected']
        start_xy = self.odom_xy
        self.state = None
        self.state_since = time.monotonic()
        self.path_len = 0
        self.latest_path_length_m = 0.0
        self.publish_goal(goal)

        goal_xy = (goal[0], goal[1])
        start_time = time.monotonic()
        last_print = 0.0

        while time.monotonic() - start_time < self.timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            now = time.monotonic()
            elapsed = now - start_time
            moved = distance(start_xy, self.odom_xy) if self.odom_xy else 0.0
            goal_error = distance(self.odom_xy, goal_xy) if self.odom_xy else math.inf

            if elapsed - last_print >= 2.0:
                last_print = elapsed
                print(
                    'progress goal={} t={:.1f}s state={} moved={:.3f} '
                    'goal_error={:.3f} path={} debug={}'.format(
                        index,
                        elapsed,
                        self.state,
                        moved,
                        goal_error,
                        self.path_len,
                        json.dumps(self.latest_debug, sort_keys=True)
                        if self.latest_debug else None,
                    ),
                    flush=True,
                )

            if self.state == 'GOAL_REACHED':
                max_reasonable_move = (
                    self.latest_path_length_m * self.reached_max_efficiency
                    + self.reached_extra_m
                )
                efficient = (
                    self.latest_path_length_m <= 1e-6
                    or moved <= max_reasonable_move
                )
                ok = expected == 'reached' and efficient
                return ok, {
                    'goal': index,
                    'result': 'GOAL_REACHED' if efficient else 'INEFFICIENT_REACHED',
                    'expected': expected,
                    'elapsed_s': round(elapsed, 2),
                    'moved_m': round(moved, 3),
                    'goal_error_m': round(goal_error, 3),
                    'path_waypoints': self.path_len,
                    'planned_length_m': round(self.latest_path_length_m, 3),
                    'max_reasonable_move_m': round(max_reasonable_move, 3),
                    'debug': self.latest_debug,
                }

            if expected == 'blocked' and moved > self.blocked_max_move_m:
                return False, {
                    'goal': index,
                    'result': 'BLOCKED_MOVED_TOO_FAR',
                    'expected': expected,
                    'elapsed_s': round(elapsed, 2),
                    'moved_m': round(moved, 3),
                    'goal_error_m': round(goal_error, 3),
                    'path_waypoints': self.path_len,
                    'planned_length_m': round(self.latest_path_length_m, 3),
                    'state': self.state,
                    'cmd': self.latest_cmd,
                    'debug': self.latest_debug,
                }

            if self.state in {'BLOCKED_STOP', 'WATCHDOG_STOP', 'STUCK_RECOVERY'}:
                if expected == 'blocked' and self.state == 'STUCK_RECOVERY':
                    continue
                if now - self.state_since >= self.block_fail_s:
                    safely_blocked = (
                        expected == 'blocked'
                        and self.state == 'BLOCKED_STOP'
                        and self.latest_cmd is not None
                        and abs(self.latest_cmd[0]) < 1e-4
                        and abs(self.latest_cmd[1]) < 1e-4
                    )
                    return safely_blocked, {
                        'goal': index,
                        'result': self.state,
                        'expected': expected,
                        'elapsed_s': round(elapsed, 2),
                        'state_duration_s': round(now - self.state_since, 2),
                        'moved_m': round(moved, 3),
                        'goal_error_m': round(goal_error, 3),
                        'path_waypoints': self.path_len,
                        'planned_length_m': round(self.latest_path_length_m, 3),
                        'cmd': self.latest_cmd,
                        'debug': self.latest_debug,
                    }

        return False, {
            'goal': index,
            'result': 'TIMEOUT',
            'expected': expected,
            'elapsed_s': round(self.timeout_s, 2),
            'odom_xy': self.odom_xy,
            'path_waypoints': self.path_len,
            'planned_length_m': round(self.latest_path_length_m, 3),
            'state': self.state,
            'cmd': self.latest_cmd,
            'debug': self.latest_debug,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal', action='append', type=parse_goal_spec, required=True)
    parser.add_argument('--timeout-s', type=float, default=70.0)
    parser.add_argument('--block-fail-s', type=float, default=5.0)
    parser.add_argument('--blocked-max-move-m', type=float, default=0.70)
    parser.add_argument('--reached-max-efficiency', type=float, default=1.65)
    parser.add_argument('--reached-extra-m', type=float, default=0.20)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('maze_nav_smoke_goal_client')
    client = SmokeClient(
        node,
        args.timeout_s,
        args.block_fail_s,
        args.blocked_max_move_m,
        args.reached_max_efficiency,
        args.reached_extra_m,
    )

    all_ok = True
    results = []
    try:
        for idx, goal_spec in enumerate(args.goal, start=1):
            ok, result = client.run_goal(goal_spec, idx)
            results.append(result)
            print('result ' + json.dumps(result, sort_keys=True), flush=True)
            all_ok = all_ok and ok
            if not ok:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('summary ' + json.dumps(results, sort_keys=True), flush=True)
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
