#!/usr/bin/env python3
"""Monitor/assert del test de mision C5.

Verifica el INVARIANTE de seguridad: todo /goal_pose emitido cae en celda libre
del mapa inflado (ningun goal atraviesa una pared). Ademas chequea el resultado
esperado: 'done' (la mision llega a DONE) o 'reject' (el cono es inalcanzable ->
NO se llega a DONE y no se emite ningun goal hacia la pared).

Uso: python scripts/mission_monitor.py EXPECTED DURATION MAP_YAML [inflation_m]
  EXPECTED: done | reject
"""
import json
import math
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'maze_nav'))
sys.path.insert(0, os.path.join(ROOT, 'src', 'maze_mission'))

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from maze_nav.nav_utils import load_map  # noqa: E402
from maze_mission.occupancy import GridSpec, inflate_occupancy, is_cell_free, world_to_grid  # noqa: E402

EXPECTED = sys.argv[1] if len(sys.argv) > 1 else 'done'
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
MAP = sys.argv[3] if len(sys.argv) > 3 else 'maps/maze_slam.yaml'
INFL = float(sys.argv[4]) if len(sys.argv) > 4 else 0.26


class Monitor(Node):

    def __init__(self, grid, spec):
        super().__init__('mission_monitor')
        self.grid = grid
        self.spec = spec
        self.states = []
        self.goals = []
        self.all_free = True
        self.done = False
        self.failure = False
        self.create_subscription(String, '/mission_state', self._on_state, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)

    def _on_state(self, msg):
        if not self.states or self.states[-1] != msg.data:
            self.states.append(msg.data)
        if msg.data == 'DONE':
            self.done = True
        elif msg.data == 'FAILURE':
            self.failure = True

    def _on_goal(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        free = is_cell_free(self.grid, world_to_grid(x, y, self.spec))
        self.goals.append((round(x, 2), round(y, 2), bool(free)))
        if not free:
            self.all_free = False


def main():
    m = load_map(os.path.join(ROOT, MAP))
    spec = GridSpec(m['res'], m['origin'][0], m['origin'][1])
    grid = inflate_occupancy(m['occ'], int(round(INFL / m['res'])))

    rclpy.init()
    node = Monitor(grid, spec)
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < DURATION:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.done or node.failure:
            # dejar correr un poco mas para captar goals tardios, luego cortar
            if node.done and EXPECTED == 'done':
                break

    n_bad = sum(1 for g in node.goals if not g[2])
    if EXPECTED == 'done':
        ok = node.all_free and node.done
    else:  # reject: nunca DONE y ningun goal en pared
        ok = node.all_free and not node.done

    print('[monitor] ' + json.dumps({
        'expected': EXPECTED, 'done': node.done, 'failure': node.failure,
        'n_goals': len(node.goals), 'goals_en_pared': n_bad,
        'all_goals_free': node.all_free, 'states': node.states}))
    print('[monitor] RESULT:', 'PASS' if ok else 'FAIL')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
