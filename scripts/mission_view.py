#!/usr/bin/env python3
"""Vista top-down de una corrida de mision (graba y renderiza). Visual de C5.

Se suscribe a /amcl_pose (trayectoria del robot), /goal_pose (goals emitidos) y
/mission_state, y al terminar dibuja todo sobre el mapa del laberinto: paredes,
trayectoria del robot (coloreada por tiempo), goals, y el cono (naranja). Permite
VER como el robot interactua con el cono sin Gazebo (percepcion mockeada).

Uso: python scripts/mission_view.py CONE_X CONE_Y DURATION SCENARIO [map.yaml]
Salida -> results/parte_c/C5/mission_view_<scenario>.png
"""
import math
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'maze_nav'))

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from maze_nav.nav_utils import load_map  # noqa: E402

CONE_X = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
CONE_Y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
DURATION = float(sys.argv[3]) if len(sys.argv) > 3 else 40.0
SCEN = sys.argv[4] if len(sys.argv) > 4 else 'reachable'
MAP = sys.argv[5] if len(sys.argv) > 5 else 'maps/maze_slam.yaml'
OUT = os.path.join(ROOT, 'results', 'parte_c', 'C5')
os.makedirs(OUT, exist_ok=True)


class View(Node):

    def __init__(self):
        super().__init__('mission_view')
        self.traj = []
        self.goals = []
        self.states = []
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 20)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self.create_subscription(String, '/mission_state', self._on_state, 10)

    def _on_pose(self, msg):
        p = msg.pose.pose.position
        self.traj.append((p.x, p.y))

    def _on_goal(self, msg):
        self.goals.append((msg.pose.position.x, msg.pose.position.y))

    def _on_state(self, msg):
        if not self.states or self.states[-1] != msg.data:
            self.states.append(msg.data)


def render(node):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    m = load_map(os.path.join(ROOT, MAP))
    occ, res = m['occ'], m['res']
    ox, oy = m['origin']
    extent = [ox, ox + occ.shape[1] * res, oy, oy + occ.shape[0] * res]

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(occ == 100, origin='lower', cmap='Greys', extent=extent, alpha=0.9)
    ax.imshow(occ == -1, origin='lower', cmap='Greys', extent=extent, alpha=0.15)

    traj = np.array(node.traj) if node.traj else np.empty((0, 2))
    if len(traj):
        ax.scatter(traj[:, 0], traj[:, 1], c=np.arange(len(traj)), cmap='viridis',
                   s=10, label='robot (tiempo)')
        ax.plot(traj[0, 0], traj[0, 1], 'ks', ms=10, label='start')
        ax.plot(traj[-1, 0], traj[-1, 1], 'b>', ms=12, label='robot final')
    for gx, gy in node.goals:
        ax.plot(gx, gy, 'gX', ms=13, mew=2)
    if node.goals:
        ax.plot([], [], 'gX', ms=10, label=f'goals emitidos ({len(node.goals)})')
    # cono
    ax.plot(CONE_X, CONE_Y, marker='^', color='darkorange', ms=18, mec='k',
            label='cono rojo')

    final = node.states[-1] if node.states else '?'
    zoom = 3.0
    cxs = traj[:, 0].tolist() + [CONE_X] if len(traj) else [CONE_X]
    cys = traj[:, 1].tolist() + [CONE_Y] if len(traj) else [CONE_Y]
    ax.set_xlim(min(cxs) - zoom, max(cxs) + zoom)
    ax.set_ylim(min(cys) - zoom, max(cys) + zoom)
    ax.set_aspect('equal')
    ax.set_title(f'Mision C5 - escenario {SCEN}\nestado final: {final}   '
                 f'goals: {len(node.goals)}')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(alpha=0.2)
    out = os.path.join(OUT, f'mission_view_{SCEN}.png')
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f'[view] estados={node.states}')
    print(f'[view] trayectoria={len(node.traj)} poses, goals={len(node.goals)} -> {out}')


def main():
    rclpy.init()
    node = View()
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < DURATION:
        rclpy.spin_once(node, timeout_sec=0.1)
    render(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
