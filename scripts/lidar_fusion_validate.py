#!/usr/bin/env python3
"""Validacion offline de la estrategia LIDAR-fusion (Parte C, etapa C1).

Lee un rosbag (por defecto laberinto_conos), corre el detector de cono rojo sobre
cada frame de la camara, estima la posicion del cono con LIDAR-fusion y compara
distintas configuraciones de geometria (offset del LIDAR y modo de rango) para
verificar EMPIRICAMENTE que el offset -90deg (LIDAR montado a +90) es el correcto.

Metrica: como el cono es estatico, sus estimaciones de mundo (frame odom) deben
AGRUPARSE. Se reporta, por configuracion, el cluster mas denso (ratio de inliers
a 0.3m + dispersion). La geometria correcta agrupa mucho mas que las erroneas.

Uso (dentro de rosenv, desde la raiz del repo):
  python scripts/lidar_fusion_validate.py [bag] [max_detections] [img_stride]

Evidencia -> results/parte_c/C1/lidar_fusion/
"""
import math
import os
import sys
from collections import Counter, defaultdict

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'maze_perception'))
sys.path.insert(0, os.path.join(ROOT, 'src', 'maze_mission'))

import cv2  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

from maze_perception.blob_extractor import BlobFilter, extract_blobs  # noqa: E402
from maze_perception.hsv_segmenter import RedHSVThresholds, segment_red  # noqa: E402
from maze_mission.cone_goal_estimator import cone_world_from_lidar  # noqa: E402

BAG = sys.argv[1] if len(sys.argv) > 1 else 'rosbags/laberinto_conos'
MAX_DET = int(sys.argv[2]) if len(sys.argv) > 2 else 500
IMG_STRIDE = int(sys.argv[3]) if len(sys.argv) > 3 else 2
OUT = os.path.join(ROOT, 'results', 'parte_c', 'C1', 'lidar_fusion')
os.makedirs(OUT, exist_ok=True)

OFFSETS = {'-90': -math.pi / 2, '0': 0.0, '+90': math.pi / 2}
MODES = ['nearest', 'median']
PRIMARY = ('-90', 'nearest')
PLAUSIBLE = (0.20, 5.0)   # rango metrico plausible para un cono en el laberinto

THR = RedHSVThresholds()
BLOBF = BlobFilter()


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def image_to_bgr(msg):
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)
    enc = msg.encoding.lower()
    if enc == 'rgb8':
        return np.ascontiguousarray(buf[:, :w * 3].reshape(h, w, 3)[:, :, ::-1])
    return np.ascontiguousarray(buf[:, :w * 3].reshape(h, w, 3))   # bgr8


def _pearson(xs, ys):
    if len(xs) < 5:
        return float('nan')
    x, y = np.asarray(xs, float), np.asarray(ys, float)
    if x.std() == 0 or y.std() == 0:
        return float('nan')
    return float(np.corrcoef(x, y)[0, 1])


def densest_cluster(points, radius=0.3, bin_size=0.15):
    if len(points) < 3:
        return None
    pts = np.asarray(points, dtype=float)
    keys = np.floor(pts / bin_size).astype(int)
    best = Counter(map(tuple, keys)).most_common(1)[0][0]
    center = (np.array(best) + 0.5) * bin_size
    for _ in range(3):   # refinar el centro sobre los inliers
        d = np.hypot(pts[:, 0] - center[0], pts[:, 1] - center[1])
        inl = pts[d <= radius]
        if len(inl) == 0:
            break
        center = inl.mean(axis=0)
    d = np.hypot(pts[:, 0] - center[0], pts[:, 1] - center[1])
    inl = pts[d <= radius]
    di = np.hypot(inl[:, 0] - center[0], inl[:, 1] - center[1]) if len(inl) else np.array([0.0])
    return {'n': len(pts), 'inliers': int(len(inl)), 'ratio': len(inl) / len(pts),
            'cx': float(center[0]), 'cy': float(center[1]),
            'mad': float(np.median(di)), 'p95': float(np.percentile(di, 95))}


def main():
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    fx = cx = None
    scan = scan_t = None
    pose = pose_t = None
    img_idx = 0
    n_det = 0
    n_msg = 0
    # worlds[cfg] = lista de (wx, wy, r); rows = detalle de la config primaria
    worlds = defaultdict(list)
    rows = []
    frames_for_png = []

    print(f'Leyendo {BAG} (max_det={MAX_DET}, img_stride={IMG_STRIDE}) ...')
    while reader.has_next() and n_det < MAX_DET:
        topic, data, _ = reader.read_next()
        n_msg += 1
        if n_msg % 20000 == 0:
            print(f'  ... {n_msg} msgs, {n_det} detecciones')

        if topic.endswith('camera_info') and fx is None:
            ci = deserialize_message(data, get_message(type_map[topic]))
            if ci.k[0] > 0:
                fx, cx = float(ci.k[0]), float(ci.k[2])
        elif topic.endswith('/scan'):
            scan = deserialize_message(data, get_message(type_map[topic]))
            scan_t = stamp_s(scan.header)
        elif topic.endswith('/odom'):
            od = deserialize_message(data, get_message(type_map[topic]))
            p = od.pose.pose
            pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
            pose_t = stamp_s(od.header)
        elif topic.endswith('image_raw'):
            img_idx += 1
            if img_idx % IMG_STRIDE or fx is None or scan is None or pose is None:
                continue
            img = deserialize_message(data, get_message(type_map[topic]))
            it = stamp_s(img.header)
            if abs(it - scan_t) > 0.2 or abs(it - pose_t) > 0.2:
                continue
            bgr = image_to_bgr(img)
            blobs = extract_blobs(segment_red(bgr, THR), BLOBF)
            if not blobs:
                continue
            b = blobs[0]                       # el mas grande
            bearing = math.atan2(cx - b.u, fx)
            ranges = list(scan.ranges)
            got_primary = None
            for oname, off in OFFSETS.items():
                for mode in MODES:
                    out = cone_world_from_lidar(
                        bearing, pose, ranges, scan.angle_min, scan.angle_increment,
                        lidar_yaw_offset=off, range_mode=mode)
                    if out is None:
                        continue
                    wx, wy, r = out
                    worlds[(oname, mode)].append((wx, wy, r, b.area_px))
                    if (oname, mode) == PRIMARY:
                        got_primary = (wx, wy, r)
            if got_primary is not None:
                wx, wy, r = got_primary
                rows.append((it, b.u, b.v, b.area_px, math.degrees(bearing),
                             pose[0], pose[1], math.degrees(pose[2]), r, wx, wy))
                n_det += 1
                if len(frames_for_png) < 8 and n_det % 25 == 1:
                    frames_for_png.append((bgr.copy(), b, r, bearing))

    print(f'\nProcesados {n_msg} msgs; {n_det} detecciones rojas con rango.')

    # diversidad de puntos de vista (si el robot casi no se movio, el clustering
    # no discrimina; la correlacion area<->rango si).
    if rows:
        rx = np.array([r[5] for r in rows]); ry = np.array([r[6] for r in rows])
        print(f'Spread de pose del robot: x[{rx.min():.2f},{rx.max():.2f}] '
              f'y[{ry.min():.2f},{ry.max():.2f}] std=({rx.std():.2f},{ry.std():.2f})\n')

    # --- comparacion de configuraciones ---
    print('Config (offset,mode):   n  corr(sqrt(area),1/r)  ratio  MAD   r_med   (corr = discriminador)')
    summary = {}
    for cfg in [(o, m) for o in OFFSETS for m in MODES]:
        good = [(w[0], w[1], w[2], w[3]) for w in worlds[cfg]
                if PLAUSIBLE[0] <= w[2] <= PLAUSIBLE[1]]
        pts = [(g[0], g[1]) for g in good]
        rmed = float(np.median([w[2] for w in worlds[cfg]])) if worlds[cfg] else float('nan')
        corr = _pearson([math.sqrt(g[3]) for g in good], [1.0 / g[2] for g in good])
        cl = densest_cluster(pts)
        summary[f'{cfg[0]}/{cfg[1]}'] = {'cluster': cl, 'range_median': rmed,
                                         'n_plausible': len(pts), 'area_range_corr': corr}
        mark = '  <== PRIMARIA' if cfg == PRIMARY else ''
        if cl:
            print(f'  {cfg[0]:>3}/{cfg[1]:<7}: {cl["n"]:4d}   {corr:+.2f}                {cl["ratio"]:.2f}  '
                  f'{cl["mad"]:.3f}  r_med={rmed:.2f}{mark}')
        else:
            print(f'  {cfg[0]:>3}/{cfg[1]:<7}: sin datos suficientes')

    # --- evidencia ---
    _save_csv(rows)
    _save_scatter(worlds, rows)
    _save_frames(frames_for_png)
    _save_summary(summary, n_det, n_msg)
    print(f'\nEvidencia guardada en {OUT}')


def _save_csv(rows):
    import csv
    with open(os.path.join(OUT, 'detections.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'u', 'v', 'area_px', 'bearing_deg', 'robot_x', 'robot_y',
                    'robot_yaw_deg', 'range_m', 'cone_wx', 'cone_wy'])
        w.writerows(rows)


def _save_scatter(worlds, rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharex=True, sharey=True)
    for ax, oname in zip(axes, ['-90', '0', '+90']):
        pts = np.array([(w[0], w[1]) for w in worlds[(oname, 'nearest')]
                        if PLAUSIBLE[0] <= w[2] <= PLAUSIBLE[1]] or [(0, 0)])
        ax.scatter(pts[:, 0], pts[:, 1], s=6, alpha=0.4, label='cono estimado')
        if rows:
            rob = np.array([(r[5], r[6]) for r in rows])
            ax.plot(rob[:, 0], rob[:, 1], '-', color='gray', lw=0.5, alpha=0.6, label='robot')
        ax.set_title(f'offset {oname}deg (nearest)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc='best', fontsize=8)
    fig.suptitle('LIDAR-fusion: posicion estimada del cono (frame odom) por offset')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'cone_positions_by_offset.png'), dpi=110)
    plt.close(fig)


def _save_frames(frames):
    for i, (bgr, b, r, bearing) in enumerate(frames):
        img = cv2.resize(bgr, (500, 500), interpolation=cv2.INTER_NEAREST)
        sx, sy = 500 / bgr.shape[1], 500 / bgr.shape[0]
        cv2.circle(img, (int(b.u * sx), int(b.v * sy)), 8, (0, 255, 0), 2)
        cv2.putText(img, f'r={r:.2f}m  bearing={math.degrees(bearing):+.0f}deg',
                    (10, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imwrite(os.path.join(OUT, f'frame_{i:02d}.png'), img)


def _save_summary(summary, n_det, n_msg):
    import json
    with open(os.path.join(OUT, 'summary.json'), 'w') as f:
        json.dump({'bag': BAG, 'n_msgs': n_msg, 'n_detections': n_det,
                   'primary': f'{PRIMARY[0]}/{PRIMARY[1]}', 'configs': summary},
                  f, indent=2)


if __name__ == '__main__':
    main()
