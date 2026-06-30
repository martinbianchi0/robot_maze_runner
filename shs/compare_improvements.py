#!/usr/bin/env python3
"""Compara mejoras de calidad de mapa sobre el BAG COMPLETO.

Config base elegida en el tuning: alpha=(0.04,0.02,0.05,0.02), sigma=0.08.
Variantes:
  A) baseline    : 50 part, res 0.05, SIN scan-match   (la que elegiste, bag completo)
  B) +scanmatch  : 50 part, res 0.05, CON scan-match
  C) full mejoras: 80 part, res 0.03, CON scan-match

Salida: ~/Downloads/slam_tuning/IMPROVEMENTS.png + metrics_improvements.txt
"""
import sys
from pathlib import Path

from PIL import Image

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE / 'shs'))
import tune  # noqa: E402

SECS = 1400.0  # bag completo (~1393s)
ALPHA = [0.04, 0.02, 0.05, 0.02]
SIGMA = 0.08

VARIANTS = [
    ('A_baseline_50p_res05_noSM',  dict(particles=50, res=0.05, map_size=300, scan_match=False)),
    ('B_scanmatch_50p_res05',      dict(particles=50, res=0.05, map_size=300, scan_match=True)),
    ('C_full_80p_res03_SM',        dict(particles=80, res=0.03, map_size=500, scan_match=True)),
]


def main():
    out = Path.home() / 'Downloads' / 'slam_tuning'
    out.mkdir(parents=True, exist_ok=True)

    db3 = tune.find_bag_db3()
    print(f'Leyendo {SECS:.0f}s de {db3.name}... (una sola vez)', flush=True)
    msgs, scan_tid = tune.load_messages(db3, SECS)
    print(f'  {len(msgs)} mensajes.\n', flush=True)

    base = dict(min_trans=0.04, min_rot=0.04, step_map=4, step_weight=4,
                occ_thresh=0.7, seed=42, sigma=SIGMA, alpha=ALPHA)

    cells, rows = [], [f'{"variante":>28} | {"consist(m)":>10} | {"explored":>8} | {"occ":>6} | {"upd":>5}',
                       '-' * 70]
    for name, ov in VARIANTS:
        p = dict(base, **ov)
        print(f'>> {name}  ({ov}) ...', flush=True)
        fs, st = tune.run_slam(msgs, scan_tid, p)
        png = out / f'improve_{name}.png'
        cells.append(tune.render(fs, str(png), name, st))
        line = (f'{name:>28} | {st["consistency_m"]:>10.3f} | {st["explored"]:>8} | '
                f'{st["occupied"]:>6} | {st["updates"]:>5}')
        rows.append(line)
        print('   ' + line, flush=True)

    # montaje horizontal
    h = max(i.height for i in cells)
    imgs = [i.resize((int(i.width * h / i.height), h)) for i in cells]
    pad = 8
    W = sum(i.width for i in imgs) + pad * (len(imgs) + 1)
    canvas = Image.new('RGB', (W, h + 2 * pad), (35, 35, 35))
    x = pad
    for im in imgs:
        canvas.paste(im, (x, pad))
        x += im.width + pad
    canvas.save(out / 'IMPROVEMENTS.png')

    table = '\n'.join(rows)
    (out / 'metrics_improvements.txt').write_text(table + '\n')
    print('\n' + table)
    print(f'\nMontaje: {out / "IMPROVEMENTS.png"}')


if __name__ == '__main__':
    main()
