#!/usr/bin/env python3
"""Cliente de smoke test C4: valida el contrato de navegacion de toma-2.

Publica un /goal_pose, monitorea /nav_state y /amcl_pose, y verifica que el
navigator llegue a REACHED con la pose final cerca del goal. Imprime la secuencia
de estados observada (contrato) y termina con exit 0 (PASS) / 1 (FAIL).

Uso: python scripts/goal_nav_client.py GOAL_X GOAL_Y [timeout_s] [tol_m]
"""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

GOAL_X = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
GOAL_Y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
TIMEOUT = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
TOL = float(sys.argv[4]) if len(sys.argv) > 4 else 0.25


class GoalNavClient(Node):

    def __init__(self):
        super().__init__('goal_nav_client')
        self.state = None
        self.pose = None
        self.states_seen = []
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(String, '/nav_state', self._on_state, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10)

    def _on_state(self, msg):
        if not self.states_seen or self.states_seen[-1] != msg.data:
            self.states_seen.append(msg.data)
        self.state = msg.data

    def _on_pose(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def send_goal(self):
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = GOAL_X
        g.pose.position.y = GOAL_Y
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)


def main():
    rclpy.init()
    node = GoalNavClient()

    t0 = time.monotonic()
    while time.monotonic() - t0 < 8.0 and node.pose is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.pose is None:
        print('[goal_nav] no llego /amcl_pose; abortando')
        print('[goal_nav] RESULT: FAIL')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    for _ in range(5):
        node.send_goal()
        rclpy.spin_once(node, timeout_sec=0.1)

    reached = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.state == 'REACHED':
            reached = True
            break

    dist = math.hypot(node.pose[0] - GOAL_X, node.pose[1] - GOAL_Y)
    ok = reached and dist <= TOL
    print('[goal_nav] ' + json.dumps({
        'reached': reached, 'final_state': node.state,
        'dist_to_goal_m': round(dist, 3), 'goal': [GOAL_X, GOAL_Y],
        'final_pose': [round(node.pose[0], 2), round(node.pose[1], 2)],
        'states': node.states_seen}))
    print('[goal_nav] RESULT:', 'PASS' if ok else 'FAIL')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
