#!/usr/bin/env python3
"""Tuning offline de hiperparametros del FastSLAM.

Reproduce el bag del laberinto DIRECTO al algoritmo (sin ROS pub/sub ni RViz),
mucho mas rapido que correr slam.sh en vivo. Genera un PGM/PNG del mapa y una
metrica objetiva de consistencia, para comparar configuraciones.

EJEMPLOS
--------
# una corrida con params custom (120s de bag):
python3 shs/tune.py --particles 30 --sigma 0.10 --secs 120

# barrido de un parametro (genera un montaje comparativo):
python3 shs/tune.py --sweep sigma 0.08,0.12,0.20 --secs 120
python3 shs/tune.py --sweep particles 10,25,50

# barrido del ruido de movimiento (alpha = a1,a2,a3,a4):
python3 shs/tune.py --sweep alpha "0.02,0.01,0.03,0.01 0.05,0.02,0.06,0.02"

Salidas en results/tuning/.

METRICA
-------
"consistency" = distancia media (m) de los endpoints del scan a la celda ocupada
mas cercana del mapa final, sobre una muestra de scans de la trayectoria.
Mas BAJO = mejor (los scans caen sobre las paredes -> mapa nitido, poca deriva).
Tambien reporta celdas exploradas (cobertura) y ocupadas (a mas deriva, mas smear).
"""
import argparse
import math
import sqlite3
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE / 'src' / 'maze_slam'))
from maze_slam.fastslam import FastSLAM  # noqa: E402

try:
    from scipy.ndimage import distance_transform_edt
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def find_bag_db3():
    """Devuelve el .db3 a leer. Prioriza la copia LOCAL (no raid5) que arma bag.sh,
    porque el .db3 en la raid5 se corrompe."""
    import os
    cache = os.environ.get('MAZE_BAG_CACHE', '/var/tmp/maze_slam_bag')
    candidates = [
        Path(cache) / 'laberinto' / 'laberinto_0.db3',
        HERE / 'maps' / 'laberinto' / 'laberinto_0.db3',
    ]
    for p in candidates:
        if p.exists():
            return p
    print('ERROR: no encuentro el bag. Corré ./shs/bag.sh una vez (extrae a disco local).',
          file=sys.stderr)
    sys.exit(1)


def load_messages(db3, secs):
    """Lee scan+odom en orden temporal hasta `secs` del bag. Tolera corrupcion."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    scan_cls = get_message('sensor_msgs/msg/LaserScan')
    odom_cls = get_message('nav_msgs/msg/Odometry')
    c = sqlite3.connect(f'file:{db3}?mode=ro', uri=True)
    tid = dict(c.execute("SELECT name, id FROM topics").fetchall())
    # Primer timestamp: lo sacamos del primer chunk legible (MIN(timestamp) global
    # puede pegar la corrupcion del bag original).
    want = (tid['/tb4_0/scan'], tid['/tb4_0/odom'])
    t0 = None
    probe = 1
    while t0 is None and probe < 50000:
        try:
            row = c.execute(
                f"SELECT MIN(timestamp) FROM messages WHERE id BETWEEN {probe} AND {probe + 4999} "
                f"AND topic_id={tid['/tb4_0/scan']}"
            ).fetchone()
            if row and row[0]:
                t0 = row[0]
        except sqlite3.DatabaseError:
            pass
        probe += 5000
    if t0 is None:
        print('ERROR: no pude leer timestamps del bag.', file=sys.stderr)
        sys.exit(1)
    t_end = t0 + int(secs * 1e9)
    out = []
    # Leemos por chunks (rango fijo 1..200000) para sobrevivir paginas corruptas.
    lo_id, hi_id = 1, 200000
    CH = 2000
    i = lo_id
    while i <= hi_id:
        try:
            rows = c.execute(
                f"SELECT timestamp, topic_id, data FROM messages "
                f"WHERE id BETWEEN {i} AND {i + CH - 1} ORDER BY id"
            ).fetchall()
        except sqlite3.DatabaseError:
            i += CH
            continue
        for ts, topic, data in rows:
            if topic not in want or ts > t_end:
                continue
            cls = scan_cls if topic == tid['/tb4_0/scan'] else odom_cls
            try:
                out.append((ts, topic, deserialize_message(bytes(data), cls)))
            except Exception:
                pass
        i += CH
    out.sort(key=lambda r: r[0])
    return out, tid['/tb4_0/scan']


def run_slam(msgs, scan_topic_id, p):
    """Corre el FastSLAM con el dict de params `p`. Devuelve (fs, stats)."""
    fs = FastSLAM(
        n_particles=p['particles'], map_size=p['map_size'], resolution=p['res'],
        alpha=tuple(p['alpha']), sigma_hit=p['sigma'],
        scan_step_map=p['step_map'], scan_step_weight=p['step_weight'],
        occ_threshold=p['occ_thresh'],
        use_scan_match=p.get('scan_match', True),
        sensor_x=-0.040, sensor_y=0.0, sensor_yaw=math.pi / 2.0,
    )
    rng = np.random.default_rng(p['seed'])
    last_odom = None
    n_upd = 0
    traj = []  # (best_pose, scan) para metrica
    for ts, topic, m in msgs:
        if topic != scan_topic_id:
            last_odom = (m.pose.pose.position.x, m.pose.pose.position.y,
                         quat_yaw(m.pose.pose.orientation))
            continue
        if last_odom is None:
            continue
        first = n_upd == 0
        delta = fs.odom_delta(*last_odom, min_trans=p['min_trans'], min_rot=p['min_rot'])
        if delta is None and not first:
            continue
        fs.step(delta, m.ranges, m.angle_min, m.angle_increment, m.range_max, rng)
        b = fs.best()
        traj.append(((b.x, b.y, b.theta), m))
        n_upd += 1

    stats = compute_stats(fs, traj)
    stats['updates'] = n_upd
    return fs, stats


def compute_stats(fs, traj):
    best = fs.best()
    occ = best.log_odds > fs.occ_threshold
    free = best.log_odds < (fs.l_free * 0.5)
    n_occ = int(occ.sum())
    n_free = int(free.sum())
    explored = n_occ + n_free

    # consistency: distancia media endpoint -> celda ocupada mas cercana, sobre
    # una muestra de scans de la trayectoria, usando el mapa final.
    consistency = float('nan')
    if _HAS_SCIPY and occ.any() and traj:
        dist = distance_transform_edt(~occ).astype(np.float32) * fs.res
        sample = traj[::max(1, len(traj) // 60)]  # ~60 scans
        ds = []
        for (bx, by, bth), m in sample:
            lx = bx + fs.sx * math.cos(bth) - fs.sy * math.sin(bth)
            ly = by + fs.sx * math.sin(bth) + fs.sy * math.cos(bth)
            lth = bth + fs.syaw
            ranges = np.asarray(m.ranges, dtype=np.float32)
            idx = np.arange(0, len(ranges), 6)
            rs = ranges[idx]
            ang = m.angle_min + idx * m.angle_increment
            valid = np.isfinite(rs) & (rs > 0.06) & (rs < min(m.range_max, 8.0))
            rs, ang = rs[valid], ang[valid]
            ex = lx + rs * np.cos(lth + ang)
            ey = ly + rs * np.sin(lth + ang)
            gx = ((ex - fs.origin) / fs.res).astype(int)
            gy = ((ey - fs.origin) / fs.res).astype(int)
            ok = (gx >= 0) & (gx < fs.W) & (gy >= 0) & (gy < fs.H)
            if ok.any():
                ds.append(float(dist[gy[ok], gx[ok]].mean()))
        if ds:
            consistency = float(np.mean(ds))

    return {'occupied': n_occ, 'free': n_free, 'explored': explored,
            'consistency_m': consistency}


def render(fs, path_png, label='', stats=None):
    best = fs.best()
    img = np.full((fs.H, fs.W), 205, dtype=np.uint8)
    img[best.log_odds > fs.occ_threshold] = 0
    img[best.log_odds < (fs.l_free * 0.5)] = 254
    img = np.flipud(img)
    ys, xs = np.where(img != 205)
    if len(xs):
        y0, y1, x0, x1 = ys.min(), ys.max(), xs.min(), xs.max()
        img = img[max(0, y0 - 5):y1 + 5, max(0, x0 - 5):x1 + 5]
    im = Image.fromarray(img).convert('RGB')
    im = im.resize((im.width * 3, im.height * 3), Image.NEAREST)
    if label or stats:
        d = ImageDraw.Draw(im)
        txt = label
        if stats:
            txt += (f"\nconsist={stats['consistency_m']:.3f}m"
                    f"\nexpl={stats['explored']} occ={stats['occupied']}")
        d.multiline_text((4, 4), txt, fill=(255, 0, 0))
    im.save(path_png)
    return im


def montage(images, labels, out_png):
    pad = 6
    cw = max(i.width for i in images)
    ch = max(i.height for i in images)
    cols = min(len(images), 3)
    rows = (len(images) + cols - 1) // cols
    canvas = Image.new('RGB', (cols * cw + (cols + 1) * pad,
                               rows * ch + (rows + 1) * pad), (40, 40, 40))
    for k, im in enumerate(images):
        r, c = divmod(k, cols)
        canvas.paste(im, (pad + c * (cw + pad), pad + r * (ch + pad)))
    canvas.save(out_png)


def base_params(args):
    return {
        'particles': args.particles, 'map_size': args.map_size, 'res': args.res,
        'sigma': args.sigma, 'alpha': args.alpha, 'min_trans': args.min_trans,
        'min_rot': args.min_rot, 'step_map': args.step_map,
        'step_weight': args.step_weight, 'occ_thresh': args.occ_thresh,
        'seed': args.seed, 'scan_match': args.scan_match,
    }


def apply_override(p, name, value):
    p = dict(p)
    if name == 'alpha':
        p['alpha'] = [float(x) for x in value.split(',')]
    elif name in ('particles', 'map_size', 'step_map', 'step_weight', 'seed'):
        p[name] = int(value)
    else:
        p[name] = float(value)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--secs', type=float, default=120.0, help='segundos de bag a usar')
    ap.add_argument('--particles', type=int, default=25)
    ap.add_argument('--map-size', dest='map_size', type=int, default=300)
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--sigma', type=float, default=0.12)
    ap.add_argument('--alpha', type=lambda s: [float(x) for x in s.split(',')],
                    default=[0.03, 0.015, 0.04, 0.015])
    ap.add_argument('--min-trans', dest='min_trans', type=float, default=0.04)
    ap.add_argument('--min-rot', dest='min_rot', type=float, default=0.04)
    ap.add_argument('--step-map', dest='step_map', type=int, default=4)
    ap.add_argument('--step-weight', dest='step_weight', type=int, default=4)
    ap.add_argument('--occ-thresh', dest='occ_thresh', type=float, default=0.7)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--scan-match', dest='scan_match', action='store_true', default=True)
    ap.add_argument('--no-scan-match', dest='scan_match', action='store_false')
    ap.add_argument('--sweep', nargs=2, metavar=('PARAM', 'VALUES'),
                    help='varia un parametro. VALUES separados por coma (o por '
                         'espacio para alpha, c/u "a1,a2,a3,a4")')
    args = ap.parse_args()

    outdir = HERE / 'results' / 'tuning'
    outdir.mkdir(parents=True, exist_ok=True)

    db3 = find_bag_db3()
    print(f'Leyendo {args.secs:.0f}s de {db3.name}...', flush=True)
    msgs, scan_tid = load_messages(db3, args.secs)
    print(f'  {len(msgs)} mensajes.', flush=True)

    base = base_params(args)

    if args.sweep:
        name, raw = args.sweep
        values = raw.split() if name == 'alpha' else raw.split(',')
        images, labels = [], []
        print(f'\n{"valor":>22} | {"consist(m)":>10} | {"explored":>8} | {"occ":>6} | {"upd":>4}')
        print('-' * 64)
        for v in values:
            p = apply_override(base, name, v)
            fs, st = run_slam(msgs, scan_tid, p)
            label = f'{name}={v}'
            png = outdir / f'sweep_{name}_{v.replace(",", "_").replace(" ", "")}.png'
            images.append(render(fs, png, label, st))
            labels.append(label)
            print(f'{v:>22} | {st["consistency_m"]:>10.3f} | {st["explored"]:>8} | '
                  f'{st["occupied"]:>6} | {st["updates"]:>4}')
        mont = outdir / f'sweep_{name}.png'
        montage(images, labels, mont)
        print(f'\nMontaje: {mont}')
        print('(consistency mas BAJO = mejor; explored mas alto = mas cobertura)')
    else:
        fs, st = run_slam(msgs, scan_tid, base)
        png = outdir / 'tune_single.png'
        render(fs, png, 'single', st)
        print(f'\nconsistency={st["consistency_m"]:.3f}m  explored={st["explored"]}  '
              f'occupied={st["occupied"]}  updates={st["updates"]}')
        print(f'Mapa: {png}')


if __name__ == '__main__':
    main()
