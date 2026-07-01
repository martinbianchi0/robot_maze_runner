#!/usr/bin/env python3
"""Calibracion de umbrales HSV del cono rojo (Parte C, etapa C1).

Caracteriza el color real del cono usando el cluster de LIDAR-fusion como
pseudo-ground-truth: una deteccion es CONO (inlier) si su posicion de mundo
estimada cae en el cluster dominante; es ESPURIA (outlier) si no. Compara la
distribucion HSV de inliers vs outliers y propone umbrales que capturen el cono
y descarten lo espurio / distractores de otros colores.

Se detecta con una mascara LOOSE (rojo-naranja amplia) para no sesgar la
caracterizacion; los umbrales finos salen del analisis.

Uso: python scripts/hsv_calibrate.py [bag] [max_det] [stride]
Evidencia -> results/parte_c/C1/hsv/
"""
import json
import math
import os
import sys
from collections import Counter

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'maze_perception'))
sys.path.insert(0, os.path.join(ROOT, 'src', 'maze_mission'))

import cv2  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

from maze_perception.blob_extractor import BlobFilter, extract_blobs  # noqa: E402
from maze_mission.cone_goal_estimator import cone_world_from_lidar  # noqa: E402

BAG = sys.argv[1] if len(sys.argv) > 1 else 'rosbags/laberinto_conos'
MAX_DET = int(sys.argv[2]) if len(sys.argv) > 2 else 600
STRIDE = int(sys.argv[3]) if len(sys.argv) > 3 else 6
OUT = os.path.join(ROOT, 'results', 'parte_c', 'C1', 'hsv')
os.makedirs(OUT, exist_ok=True)

BLOBF = BlobFilter()
LO1, HI1 = (0, 60, 40), (25, 255, 255)      # banda loose baja (rojo-naranja)
LO2, HI2 = (160, 60, 40), (180, 255, 255)   # banda loose alta


def loose_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, LO1, HI1) | cv2.inRange(hsv, LO2, HI2)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    return hsv, cv2.morphologyEx(m, cv2.MORPH_OPEN, k)


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def to_bgr(msg):
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, np.uint8).reshape(h, msg.step)
    a = buf[:, :w * 3].reshape(h, w, 3)
    return np.ascontiguousarray(a[:, :, ::-1] if msg.encoding.lower() == 'rgb8' else a)


def densest(points, radius=0.3, bin_size=0.15):
    pts = np.asarray(points, float)
    keys = np.floor(pts / bin_size).astype(int)
    best = Counter(map(tuple, keys)).most_common(1)[0][0]
    c = (np.array(best) + 0.5) * bin_size
    for _ in range(3):
        d = np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1])
        inl = pts[d <= radius]
        if len(inl):
            c = inl.mean(axis=0)
    return c


def main():
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}

    fx = cx = scan = scan_t = pose = pose_t = None
    img_i = n = 0
    dets = []   # (world_xy, hue_signed, sat, val, area, bgr_small?, u, v)
    frames = []
    print(f'Leyendo {BAG} (max_det={MAX_DET}, stride={STRIDE}) ...')
    while reader.has_next() and len(dets) < MAX_DET:
        topic, data, _ = reader.read_next()
        n += 1
        if n % 30000 == 0:
            print(f'  ... {n} msgs, {len(dets)} detecciones')
        if topic.endswith('camera_info') and fx is None:
            ci = deserialize_message(data, get_message(tmap[topic]))
            if ci.k[0] > 0:
                fx, cx = float(ci.k[0]), float(ci.k[2])
        elif topic.endswith('/scan'):
            scan = deserialize_message(data, get_message(tmap[topic]))
            scan_t = stamp_s(scan.header)
        elif topic.endswith('/odom'):
            od = deserialize_message(data, get_message(tmap[topic]))
            p = od.pose.pose
            pose = (p.position.x, p.position.y, yaw(p.orientation))
            pose_t = stamp_s(od.header)
        elif topic.endswith('image_raw'):
            img_i += 1
            if img_i % STRIDE or fx is None or scan is None or pose is None:
                continue
            img = deserialize_message(data, get_message(tmap[topic]))
            it = stamp_s(img.header)
            if abs(it - scan_t) > 0.2 or abs(it - pose_t) > 0.2:
                continue
            bgr = to_bgr(img)
            hsv, mask = loose_mask(bgr)
            blobs = extract_blobs(mask, BLOBF)
            if not blobs:
                continue
            b = blobs[0]
            sub_m = mask[b.y:b.y + b.h, b.x:b.x + b.w] > 0
            if sub_m.sum() < 10:
                continue
            sub = hsv[b.y:b.y + b.h, b.x:b.x + b.w][sub_m].astype(float)
            hue = sub[:, 0]
            hsig = np.where(hue > 90, hue - 180, hue)
            out = cone_world_from_lidar(math.atan2(cx - b.u, fx), pose,
                                        list(scan.ranges), scan.angle_min,
                                        scan.angle_increment)
            if out is None:
                continue
            dets.append((out[0], out[1], float(np.median(hsig)),
                         float(np.median(sub[:, 1])), float(np.median(sub[:, 2])),
                         b.area_px, out[2]))
            if len(frames) < 12:
                frames.append((bgr.copy(), b, out[2]))

    if len(dets) < 10:
        print('muy pocas detecciones; abortando')
        return
    arr = dets
    worlds = [(d[0], d[1]) for d in arr]
    center = densest(worlds)
    inl, out = [], []
    for d in arr:
        target = inl if math.hypot(d[0] - center[0], d[1] - center[1]) <= 0.3 else out
        target.append(d)

    print(f'\n{len(arr)} detecciones: {len(inl)} inliers (cono), {len(out)} outliers (espurios)')
    print(f'Cono en odom ~ ({center[0]:.2f},{center[1]:.2f})\n')

    def stats(group, name):
        if not group:
            print(f'  {name}: (vacio)')
            return None
        h = np.array([g[2] for g in group]); s = np.array([g[3] for g in group])
        v = np.array([g[4] for g in group])
        pr = lambda x: (float(np.percentile(x, 2)), float(np.percentile(x, 50)), float(np.percentile(x, 98)))
        hh, ss, vv = pr(h), pr(s), pr(v)
        print(f'  {name} (n={len(group)}): H_signed p2/50/98={hh[0]:+.0f}/{hh[1]:+.0f}/{hh[2]:+.0f}  '
              f'S={ss[0]:.0f}/{ss[1]:.0f}/{ss[2]:.0f}  V={vv[0]:.0f}/{vv[1]:.0f}/{vv[2]:.0f}')
        return {'H': hh, 'S': ss, 'V': vv, 'n': len(group)}

    print('Distribucion HSV (H_signed: rojo ~0; naranja >0; magenta <0):')
    si = stats(inl, 'CONO   ')
    so = stats(out, 'ESPURIO')

    # --- proponer umbrales a partir del cono (inliers) ---
    hlo, hhi = math.floor(si['H'][0]), math.ceil(si['H'][2])   # p2/p98 de H_signed
    smin = max(0, int(si['S'][0]) - 10)
    vmin = max(0, int(si['V'][0]) - 10)
    proposal = {
        'low1_h': 0, 'low1_s': smin, 'low1_v': vmin,
        'high1_h': int(max(6, hhi)), 'high1_s': 255, 'high1_v': 255,
        'low2_h': int(180 + hlo) if hlo < 0 else 174, 'low2_s': smin, 'low2_v': vmin,
        'high2_h': 180, 'high2_s': 255, 'high2_v': 255,
    }
    print('\nPropuesta de umbrales HSV (para cone_detector hsv.*):')
    print('  ' + json.dumps(proposal))

    _hist(inl, out)
    _frames(frames)
    with open(os.path.join(OUT, 'hsv_calibration.json'), 'w') as f:
        json.dump({'bag': BAG, 'n': len(arr), 'inliers': len(inl), 'outliers': len(out),
                   'cone_center': [float(center[0]), float(center[1])],
                   'cono_hsv': si, 'espurio_hsv': so, 'proposal': proposal}, f, indent=2)
    print(f'\nEvidencia -> {OUT}')


def _hist(inl, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(15, 4))
    for j, (lbl, idx, rng) in enumerate([('H_signed', 2, (-40, 40)), ('S', 3, (0, 255)), ('V', 4, (0, 255))]):
        if inl:
            ax[j].hist([g[idx] for g in inl], bins=30, range=rng, alpha=0.6, label='cono', color='tab:red')
        if out:
            ax[j].hist([g[idx] for g in out], bins=30, range=rng, alpha=0.6, label='espurio', color='tab:gray')
        ax[j].set_title(lbl); ax[j].legend(); ax[j].grid(alpha=0.3)
    fig.suptitle('HSV del cono (inliers) vs espurios (outliers)')
    fig.tight_layout(); fig.savefig(os.path.join(OUT, 'hsv_hist.png'), dpi=110); plt.close(fig)


def _frames(frames):
    for i, (bgr, b, r) in enumerate(frames):
        img = cv2.resize(bgr, (400, 400), interpolation=cv2.INTER_NEAREST)
        sx, sy = 400 / bgr.shape[1], 400 / bgr.shape[0]
        cv2.rectangle(img, (int(b.x * sx), int(b.y * sy)),
                      (int((b.x + b.w) * sx), int((b.y + b.h) * sy)), (0, 255, 0), 2)
        cv2.putText(img, f'r={r:.2f}m', (8, 388), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imwrite(os.path.join(OUT, f'sample_{i:02d}.png'), img)


if __name__ == '__main__':
    main()
