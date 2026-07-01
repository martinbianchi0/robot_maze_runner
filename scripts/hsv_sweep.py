#!/usr/bin/env python3
"""Barrido de candidatos de umbral HSV en UNA sola pasada del bag (Parte C, C1).

Para cada frame corre varios sets de umbrales, estima la posicion del cono con
LIDAR-fusion, y evalua cada candidato por el cluster dominante: ratio de inliers
(precision proxy) + cantidad de inliers (recall proxy) + correlacion. Elige el
mejor sin re-leer el bag por candidato.

Uso: python scripts/hsv_sweep.py [bag] [max_frames] [stride]
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
MAX_FRAMES = int(sys.argv[2]) if len(sys.argv) > 2 else 4000
STRIDE = int(sys.argv[3]) if len(sys.argv) > 3 else 6
BLOBF = BlobFilter()

# Candidatos: (nombre, overrides sobre RedHSVThresholds)
CANDIDATES = {
    'A_orig_S120_h10':   dict(low1_s=120, high1_h=10, low2_h=170, low2_s=120),
    'B_tight_S160_h10':  dict(low1_s=160, high1_h=10, low2_h=170, low2_s=160),
    'C_tight_S140_h12':  dict(low1_s=140, high1_h=12, low2_h=168, low2_s=140),
    'D_wide_S160_h25':   dict(low1_s=160, high1_h=25, low2_h=172, low2_s=160),
    'E_strict_S185_h10': dict(low1_s=185, high1_h=10, low2_h=170, low2_s=185),
    'F_tight_S160_h8':   dict(low1_s=160, high1_h=8, low2_h=172, low2_s=160),
}
THR = {name: RedHSVThresholds(**ov) for name, ov in CANDIDATES.items()}


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def to_bgr(msg):
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, np.uint8).reshape(h, msg.step)
    a = buf[:, :w * 3].reshape(h, w, 3)
    return np.ascontiguousarray(a[:, :, ::-1] if msg.encoding.lower() == 'rgb8' else a)


def cluster_stats(recs):
    """recs: lista de (wx, wy, r, area). Devuelve ratio, inliers, mad, corr, r_med."""
    good = [x for x in recs if 0.2 <= x[2] <= 5.0]
    if len(good) < 5:
        return None
    pts = np.array([(g[0], g[1]) for g in good])
    keys = np.floor(pts / 0.15).astype(int)
    best = Counter(map(tuple, keys)).most_common(1)[0][0]
    c = (np.array(best) + 0.5) * 0.15
    for _ in range(3):
        d = np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1])
        inl = pts[d <= 0.3]
        if len(inl):
            c = inl.mean(axis=0)
    d = np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1])
    inl_mask = d <= 0.3
    inl = pts[inl_mask]
    mad = float(np.median(np.hypot(inl[:, 0] - c[0], inl[:, 1] - c[1]))) if len(inl) else 0.0
    sa = np.array([math.sqrt(g[3]) for g in good])
    ir = np.array([1.0 / g[2] for g in good])
    corr = float(np.corrcoef(sa, ir)[0, 1]) if sa.std() and ir.std() else float('nan')
    return {'n': len(good), 'inliers': int(inl_mask.sum()), 'ratio': len(inl) / len(good),
            'mad': mad, 'corr': corr, 'r_med': float(np.median([g[2] for g in good]))}


def main():
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}

    fx = cx = scan = scan_t = pose = pose_t = None
    img_i = n = frames = 0
    worlds = defaultdict(list)
    print(f'Barriendo {len(THR)} candidatos en {BAG} (max_frames={MAX_FRAMES}, stride={STRIDE}) ...')
    while reader.has_next() and frames < MAX_FRAMES:
        topic, data, _ = reader.read_next()
        n += 1
        if topic.endswith('camera_info') and fx is None:
            ci = deserialize_message(data, get_message(tmap[topic]))
            if ci.k[0] > 0:
                fx, cx = float(ci.k[0]), float(ci.k[2])
        elif topic.endswith('/scan'):
            scan = deserialize_message(data, get_message(tmap[topic])); scan_t = stamp_s(scan.header)
        elif topic.endswith('/odom'):
            od = deserialize_message(data, get_message(tmap[topic]))
            p = od.pose.pose; pose = (p.position.x, p.position.y, yaw(p.orientation)); pose_t = stamp_s(od.header)
        elif topic.endswith('image_raw'):
            img_i += 1
            if img_i % STRIDE or fx is None or scan is None or pose is None:
                continue
            img = deserialize_message(data, get_message(tmap[topic]))
            it = stamp_s(img.header)
            if abs(it - scan_t) > 0.2 or abs(it - pose_t) > 0.2:
                continue
            bgr = to_bgr(img)
            ranges = list(scan.ranges)
            frames += 1
            for name, thr in THR.items():
                blobs = extract_blobs(segment_red(bgr, thr), BLOBF)
                if not blobs:
                    continue
                b = blobs[0]
                out = cone_world_from_lidar(math.atan2(cx - b.u, fx), pose, ranges,
                                            scan.angle_min, scan.angle_increment)
                if out is not None:
                    worlds[name].append((out[0], out[1], out[2], b.area_px))

    print(f'\n{frames} frames evaluados.\n')
    print(f'{"candidato":<20} {"n":>4} {"inliers":>7} {"ratio":>6} {"corr":>6} {"MAD":>6} {"r_med":>6}')
    scored = []
    for name in THR:
        st = cluster_stats(worlds[name])
        if not st:
            print(f'{name:<20} sin datos')
            continue
        print(f'{name:<20} {st["n"]:>4} {st["inliers"]:>7} {st["ratio"]:>6.2f} '
              f'{st["corr"]:>+6.2f} {st["mad"]:>6.3f} {st["r_med"]:>6.2f}')
        # score: prioriza inliers (recall) * ratio (precision)
        scored.append((st['inliers'] * st['ratio'], name, st))
    if scored:
        scored.sort(reverse=True)
        best = scored[0]
        print(f'\nMEJOR: {best[1]}  (inliers*ratio={best[0]:.0f}, ratio={best[2]["ratio"]:.2f}, '
              f'inliers={best[2]["inliers"]}, corr={best[2]["corr"]:+.2f})')


if __name__ == '__main__':
    main()
