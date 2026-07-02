#!/usr/bin/env bash
# Smoke sintético para el caso "silla/patas": un obstáculo frontal persiste,
# el navigator no debe entrar en loop adelante/atrás indefinido y debe terminar
# detenido con una razón clara en /nav_debug.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

PYTHONPATH="$ROOT_DIR/src/maze_nav:${PYTHONPATH:-}" python3 - <<'PY'
import json
import math
import time

import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan

from maze_nav.navigator import Navigator


class Sink:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def make_front_blocked_scan(distance):
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_increment = 2.0 * math.pi / 360.0
    scan.range_min = 0.05
    scan.range_max = 8.0
    ranges = [8.0] * 360
    for i in range(360):
        ang = scan.angle_min + i * scan.angle_increment
        if abs(ang) < 0.12:
            ranges[i] = distance
    scan.ranges = ranges
    return scan


def main():
    rclpy.init()
    node = Navigator()
    try:
        cmd_sink = Sink()
        debug_sink = Sink()
        node.cmd_pub = cmd_sink
        node.debug_pub = debug_sink
        node.state_pub = Sink()
        node.path_pub = Sink()

        # Acelerar el smoke sin cambiar defaults del nodo real.
        node.recovery_hold_s = 0.001
        node.recovery_backoff_s = 0.030
        node.recovery_backoff_speed = -0.05
        node.max_recovery_attempts = 2
        node.front_blocked_confirmations = 1

        width, height, res = 120, 80, 0.05
        occ = np.zeros((height, width), dtype=np.int8)
        occ[0, :] = 100
        occ[-1, :] = 100
        occ[:, 0] = 100
        occ[:, -1] = 100
        node.map = {
            'occ': occ,
            'res': res,
            'origin': (-1.0, -2.0),
            'H': height,
            'W': width,
        }
        node._build_costmap()
        node.pose = (0.0, 0.0, 0.0)
        node.goal = (2.0, 0.0, 0.0)
        node.path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        node.scan = make_front_blocked_scan(0.16)
        node.state = 'FOLLOWING'

        node.control_loop()
        assert node.state == 'RECOVERY', node.state

        time.sleep(0.005)
        node.control_loop()
        time.sleep(0.040)
        node.control_loop()
        assert node.state == 'RECOVERY', node.state

        time.sleep(0.005)
        node.control_loop()
        time.sleep(0.040)
        node.control_loop()

        linear_cmds = [float(m.linear.x) for m in cmd_sink.messages]
        forward = [v for v in linear_cmds if v > 1e-6]
        backoff = [v for v in linear_cmds if v < -1e-6]
        assert not forward, f'comandos hacia adelante durante bloqueo: {forward}'
        assert len(backoff) <= node.max_recovery_attempts, backoff
        assert node.state == 'IDLE', node.state
        assert node.blocked_reason == 'blocked_max_recovery_attempts', node.blocked_reason
        assert node.recovery_attempts == node.max_recovery_attempts
        assert len(node.dyn_obstacles) > 0

        debug = [json.loads(m.data) for m in debug_sink.messages]
        assert debug[-1]['state'] == 'IDLE', debug[-1]
        assert debug[-1]['reason'] == 'blocked_max_recovery_attempts', debug[-1]
        assert debug[-1]['recovery_attempts'] == node.max_recovery_attempts, debug[-1]
        print('[smoke_chair_obstacle] OK')
        print(f'[smoke_chair_obstacle] cmd_linear={linear_cmds}')
        print(f'[smoke_chair_obstacle] dyn_obstacle_cells={len(node.dyn_obstacles)}')
        print(f'[smoke_chair_obstacle] final_debug={debug[-1]}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
PY
