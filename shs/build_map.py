#!/usr/bin/env python3
"""Genera el MAPA ENTREGABLE de Parte A offline, con la config de maxima calidad.

Procesa TODO el bag sin presion de tiempo real (a diferencia del nodo en vivo, que
con 80 particulas + res 0.03 no llega y descarta scans). Reproduce la calidad de
los PNG de tuning y guarda el mapa en formato ROS (pgm + yaml) listo para Parte B.

Uso:
    python3 shs/build_map.py            # config C (80p, res 0.03), bag completo
    python3 shs/build_map.py --fast     # config liviana (50p, res 0.05) para iterar
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE / 'shs'))
import tune  # noqa: E402


def save_ros_map(fs, stem):
    """Guarda pgm + yaml en formato ROS (map_server) y un png de preview."""
    best = fs.best()
    # Convencion ROS: 0=ocupado(negro), 254=libre(blanco), 205=desconocido.
    img = np.full((fs.H, fs.W), 205, dtype=np.uint8)
    img[best.log_odds > fs.occ_threshold] = 0
    img[best.log_odds < (fs.l_free * 0.5)] = 254
    img = np.flipud(img)  # ROS map_server espera origen abajo-izquierda

    pgm = stem.with_suffix('.pgm')
    yaml = stem.with_suffix('.yaml')
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{fs.W} {fs.H}\n255\n'.encode())
        f.write(img.tobytes())
    with open(yaml, 'w') as f:
        f.write(f'image: {pgm.name}\n')
        f.write(f'resolution: {fs.res}\n')
        f.write(f'origin: [{fs.origin}, {fs.origin}, 0.0]\n')
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.196\n')
    # preview png recortado
    tune.render(fs, str(stem.with_suffix('.png')), 'mapa Parte A')
    return pgm, yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fast', action='store_true', help='config liviana para iterar')
    ap.add_argument('--secs', type=float, default=1400.0)
    ap.add_argument('--out', default='laberinto_slam', help='nombre base en maps/')
    args = ap.parse_args()

    if args.fast:
        p = dict(particles=50, res=0.05, map_size=300, scan_match=True)
    else:
        p = dict(particles=80, res=0.03, map_size=500, scan_match=True)
    p.update(sigma=0.08, alpha=[0.04, 0.02, 0.05, 0.02], min_trans=0.04,
             min_rot=0.04, step_map=4, step_weight=4, occ_thresh=0.7, seed=42)

    db3 = tune.find_bag_db3()
    print(f'Leyendo {args.secs:.0f}s de {db3.name}...', flush=True)
    msgs, scan_tid = tune.load_messages(db3, args.secs)
    print(f'  {len(msgs)} mensajes. Corriendo SLAM (config '
          f'{"liviana" if args.fast else "C maxima calidad"})...', flush=True)

    fs, st = tune.run_slam(msgs, scan_tid, p)

    maps_dir = HERE / 'maps'
    maps_dir.mkdir(exist_ok=True)
    pgm, yaml = save_ros_map(fs, maps_dir / args.out)
    print(f'\nOK. consistency={st["consistency_m"]:.3f}m  explored={st["explored"]}  '
          f'updates={st["updates"]}')
    print(f'Mapa entregable: {pgm}')
    print(f'                 {yaml}')
    print(f'Preview:         {maps_dir / (args.out + ".png")}')


if __name__ == '__main__':
    main()
