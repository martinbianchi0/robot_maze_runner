#!/usr/bin/env python3
"""Valida el reencuadre del scan (LIDAR +90) por consistencia scan<->odom (C3).

Un solo scan NO distingue el offset del LIDAR (una rotacion global del scan es
degenerada con un cambio de yaw del robot). El offset SI se observa al atarlo al
movimiento de la odometria: odom dice "adelante = el heading del robot"; un offset
equivocado hace que el scan sea inconsistente con ese movimiento.

Metodo (offline, sin MCL): (1) localizar globalmente el primer scan por fuerza
bruta contra el likelihood field del mapa, por offset; (2) dead-reckon con la
odometria sobre una ventana de scans y evaluar, en cada paso, el match del scan a
la pose dead-reckoned. El offset correcto mantiene el match alto; el equivocado
diverge (la pose se va para donde no corresponde) y el match colapsa.

Uso: python scripts/scan_match_validate.py [bag] [skip_scans] [window]
Evidencia -> results/parte_c/C3/
"""
import json
import math
import os
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'maze_nav'))

import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

from maze_nav.nav_utils import load_map, odom_deltas, wrap_angle  # noqa: E402

BAG = sys.argv[1] if len(sys.argv) > 1 else 'rosbags/laberinto'
SKIP = int(sys.argv[2]) if len(sys.argv) > 2 else 200
WINDOW = int(sys.argv[3]) if len(sys.argv) > 3 else 25
OUT = os.path.join(ROOT, 'results', 'parte_c', 'C3')
os.makedirs(OUT, exist_ok=True)

SIGMA = 0.35
MAX_BEAMS = 72
LIDAR_DX = -0.04
OFFSETS = {'0': 0.0, '+90': math.pi / 2, '-90': -math.pi / 2}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def beams(scan):
    ranges = np.asarray(scan.ranges, dtype=np.float32)
    step = max(1, len(ranges) // MAX_BEAMS)
    idx = np.arange(0, len(ranges), step)
    r = ranges[idx]
    ang = scan.angle_min + idx * scan.angle_increment
    rmax = scan.range_max if scan.range_max > 0 else 12.0
    ok = np.isfinite(r) & (r > scan.range_min) & (r < rmax)
    return r[ok], ang[ok]


def collect(bag, skip, window, min_step=0.05, target_motion=2.0):
    """Junta (scan, odom) desde 'skip', saltando tramos estaticos: solo guarda un
    scan si el robot se movio >= min_step desde el ultimo guardado, hasta acumular
    'target_motion' metros o 'window' scans. Asi la ventana abarca movimiento real
    (necesario para que el offset del LIDAR sea observable)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    odom = None
    seen = 0
    out = []
    total = 0.0
    last = None
    while reader.has_next() and len(out) < window and total < target_motion:
        topic, data, _ = reader.read_next()
        if topic.endswith('/odom'):
            od = deserialize_message(data, get_message(tmap[topic]))
            p = od.pose.pose
            odom = (p.position.x, p.position.y, yaw_of(p.orientation))
        elif topic.endswith('/scan'):
            seen += 1
            if seen < skip or odom is None:
                continue
            if last is not None and math.hypot(odom[0] - last[0], odom[1] - last[1]) < min_step:
                continue
            if last is not None:
                total += math.hypot(odom[0] - last[0], odom[1] - last[1])
            last = odom
            out.append((deserialize_message(data, get_message(tmap[topic])), odom))
    return out


class Matcher:
    def __init__(self, yaml_path):
        m = load_map(yaml_path)
        self.occ, self.res = m['occ'], m['res']
        self.ox, self.oy = m['origin']
        self.H, self.W = m['H'], m['W']
        self.field = distance_transform_edt(self.occ != 100).astype(np.float32) * self.res

    def likelihood(self, px, py, pyaw, r, beam, off):
        """Match de un scan a una pose (px,py,pyaw escalares o arrays)."""
        px = np.atleast_1d(px)[:, None]
        py = np.atleast_1d(py)[:, None]
        pyaw = np.atleast_1d(pyaw)[:, None]
        sx = px + np.cos(pyaw) * LIDAR_DX
        sy = py + np.sin(pyaw) * LIDAR_DX
        wa = pyaw + (beam + off)[None, :]
        ex = sx + r[None, :] * np.cos(wa)
        ey = sy + r[None, :] * np.sin(wa)
        gx = np.clip(((ex - self.ox) / self.res).astype(np.int32), 0, self.W - 1)
        gy = np.clip(((ey - self.oy) / self.res).astype(np.int32), 0, self.H - 1)
        d = self.field[gy, gx]
        return np.mean(np.exp(-(d * d) / (2 * SIGMA * SIGMA)), axis=1)

    def global_match(self, r, beam, off):
        ys, xs = np.where(self.occ == 0)
        sub = np.linspace(0, len(xs) - 1, min(len(xs), 600)).astype(int)
        cxw = self.ox + (xs[sub] + 0.5) * self.res
        cyw = self.oy + (ys[sub] + 0.5) * self.res
        yaws = np.arange(0.0, 2 * math.pi, math.radians(10))
        px = np.repeat(cxw, len(yaws))
        py = np.repeat(cyw, len(yaws))
        pyaw = np.tile(yaws, len(cxw))
        score = self.likelihood(px, py, pyaw, r, beam, off)
        b = int(np.argmax(score))
        return (float(px[b]), float(py[b]), float(pyaw[b])), float(score[b])


def main():
    mm = Matcher(os.path.join(ROOT, 'maps', 'maze_slam.yaml'))
    data = collect(BAG, SKIP, WINDOW)
    if len(data) < 5:
        print('muy pocos scans'); return
    print(f'{len(data)} scans desde #{SKIP}; odom recorrido='
          f'{math.hypot(data[-1][1][0]-data[0][1][0], data[-1][1][1]-data[0][1][1]):.2f} m')

    curves = {}
    poses = {}
    for name, off in OFFSETS.items():
        r0, b0 = beams(data[0][0])
        pose, sc0 = mm.global_match(r0, b0, off)
        x, y, yaw = pose
        prev_odom = data[0][1]
        lik = [sc0]
        for scan, odom in data[1:]:
            rot1, trans, rot2 = odom_deltas(prev_odom, odom)
            x += trans * math.cos(yaw + rot1)
            y += trans * math.sin(yaw + rot1)
            yaw = wrap_angle(yaw + rot1 + rot2)
            prev_odom = odom
            r, bb = beams(scan)
            lik.append(float(mm.likelihood(x, y, yaw, r, bb, off)[0]))
        curves[name] = lik
        poses[name] = (x, y, yaw)
        print(f'  offset {name:>3}: match inicial={lik[0]:.3f}  final={lik[-1]:.3f}  '
              f'medio={np.mean(lik):.3f}')

    best = max(curves, key=lambda k: np.mean(curves[k]))
    print(f'\nMEJOR offset por consistencia scan<->odom: {best} '
          f'(match medio={np.mean(curves[best]):.3f})')
    _plot(curves)
    with open(os.path.join(OUT, 'scan_match.json'), 'w') as f:
        json.dump({'bag': BAG, 'skip': SKIP, 'window': len(data), 'best_offset': best,
                   'mean_match': {k: float(np.mean(v)) for k, v in curves.items()},
                   'curves': curves, 'final_pose': {k: list(v) for k, v in poses.items()}},
                  f, indent=2)
    print(f'Evidencia -> {OUT}')


def _plot(curves):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 5))
    for name, lik in curves.items():
        ax.plot(lik, marker='o', ms=3, label=f'offset {name}deg (medio {np.mean(lik):.2f})')
    ax.set_xlabel('paso (scan dead-reckoned con odom)')
    ax.set_ylabel('match scan<->mapa (1=perfecto)')
    ax.set_title('Consistencia scan<->odom por offset del LIDAR\n'
                 '(el offset correcto mantiene el match; el equivocado diverge)')
    ax.grid(alpha=0.3); ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'scan_odom_consistency.png'), dpi=110)
    plt.close(fig)


if __name__ == '__main__':
    main()
