#!/usr/bin/env python3
"""Barrido nocturno: grilla sigma x alpha (50 particulas fijas), ~700s de bag.

Genera UN montaje con los 9 mapas etiquetados + una tabla de metricas.
Salida en ~/Downloads/slam_tuning/.
"""
import math
import os
import sys
from pathlib import Path

from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE / 'shs'))
import tune  # noqa: E402

SECS = 700.0
PARTICLES = 50

SIGMAS = [0.08, 0.12, 0.20]
ALPHAS = {
    'low':  [0.02, 0.01, 0.03, 0.01],   # confia mucho en odometria
    'med':  [0.04, 0.02, 0.05, 0.02],   # balance
    'high': [0.08, 0.04, 0.10, 0.04],   # explora mas correcciones
}


def main():
    out = Path.home() / 'Downloads' / 'slam_tuning'
    out.mkdir(parents=True, exist_ok=True)

    db3 = tune.find_bag_db3()
    print(f'Leyendo {SECS:.0f}s de {db3.name}... (una sola vez)', flush=True)
    msgs, scan_tid = tune.load_messages(db3, SECS)
    print(f'  {len(msgs)} mensajes.\n', flush=True)

    base = {
        'particles': PARTICLES, 'map_size': 300, 'res': 0.05,
        'min_trans': 0.04, 'min_rot': 0.04, 'step_map': 4,
        'step_weight': 4, 'occ_thresh': 0.7, 'seed': 42,
    }

    cells = []   # (img, label)
    rows_txt = [f'{"alpha":>6} {"sigma":>6} | {"consist(m)":>10} | {"explored":>8} | {"occ":>6} | {"upd":>4}']
    rows_txt.append('-' * 60)

    for aname, avals in ALPHAS.items():
        for sigma in SIGMAS:
            p = dict(base, alpha=avals, sigma=sigma)
            print(f'>> alpha={aname}{avals} sigma={sigma} ...', flush=True)
            fs, st = tune.run_slam(msgs, scan_tid, p)
            label = f'a={aname} s={sigma}'
            png = out / f'map_a-{aname}_s-{sigma}.png'
            img = tune.render(fs, str(png), label, st)
            cells.append((img, label))
            line = (f'{aname:>6} {sigma:>6.2f} | {st["consistency_m"]:>10.3f} | '
                    f'{st["explored"]:>8} | {st["occupied"]:>6} | {st["updates"]:>4}')
            rows_txt.append(line)
            print('   ' + line, flush=True)

    # Montaje 3x3 (filas=alpha, columnas=sigma)
    imgs = [c[0] for c in cells]
    cw = max(i.width for i in imgs)
    ch = max(i.height for i in imgs)
    pad = 8
    cols, rows = 3, 3
    canvas = Image.new('RGB', (cols * cw + (cols + 1) * pad,
                               rows * ch + (rows + 1) * pad), (35, 35, 35))
    for k, (im, _) in enumerate(cells):
        r, c = divmod(k, cols)
        canvas.paste(im, (pad + c * (cw + pad), pad + r * (ch + pad)))
    montage_path = out / 'GRID_sigma_x_alpha.png'
    canvas.save(montage_path)

    table = '\n'.join(rows_txt)
    (out / 'metrics.txt').write_text(table + '\n')
    print('\n' + table)
    print(f'\nMontaje:  {montage_path}')
    print(f'Tabla:    {out / "metrics.txt"}')
    print('\nFilas = alpha (low/med/high), columnas = sigma (0.08/0.12/0.20).')
    print('consistency mas BAJO = scans caen sobre las paredes (mapa nitido).')


if __name__ == '__main__':
    main()
