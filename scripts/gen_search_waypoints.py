#!/usr/bin/env python3
"""Genera los waypoints de busqueda del cono sobre un mapa (Parte C).

Reemplaza los placeholders de config/parte_c/waypoints_laberinto.yaml por
waypoints REALES calculados sobre el mapa del laberinto (maps/maze_slam.yaml):

1. Carga el mapa (misma semantica que map_publisher/nav_utils.load_map) y lo
   infla con el criterio del navigator (robot_radius + inflation = 0.26 m).
2. Se queda solo con las celdas libres ALCANZABLES desde la pose de arranque
   (flood-fill): nada de waypoints en bolsones aislados o fuera del laberinto.
3. Farthest-point sampling con distancia GEODESICA (BFS por celdas libres, no
   euclidea: dos pasillos separados por una pared son lejanos aunque esten a
   30 cm): agrega el waypoint mas lejano de los ya elegidos hasta que toda
   celda alcanzable queda a <= coverage_radius de alguno.
4. Ordena la ruta por vecino mas cercano (geodesico) desde el arranque y asigna
   a cada waypoint el yaw de llegada (direccion desde el waypoint anterior).

Todos los waypoints caen en celda libre del mapa inflado por construccion (se
re-valida al final). El giro-scan de 360 en cada waypoint lo hace la mision
(scan_turn_steps), por eso el yaw es solo la orientacion de llegada.

Uso (desde la raiz del repo):
  python3 scripts/gen_search_waypoints.py [--map maps/maze_slam.yaml]
      [--start 2.45 -2.0] [--coverage 1.5] [--max-wp 25]
      [--out config/parte_c/waypoints_laberinto.yaml] [--preview salida.png]

El default de --start es el arranque usado en los smoke de C4/C5; en el
laboratorio, regenerar con la pose real de arranque si difiere.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from collections import deque

import numpy as np
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, 'src', 'maze_mission'))

from maze_mission.occupancy import (GridSpec, grid_to_world, inflate_occupancy,  # noqa: E402
                                    nearest_free_cell, world_to_grid)


def load_map(yaml_path):
    """Mapa map_server -> occ {-1, 0, 100} con y creciente hacia arriba."""
    with open(yaml_path) as handle:
        meta = yaml.safe_load(handle)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)
    img = load_pgm(img_path)
    negate = int(meta.get('negate', 0))
    occ_th = float(meta.get('occupied_thresh', 0.65))
    free_th = float(meta.get('free_thresh', 0.25))
    px = img.astype(np.float32)
    p = px / 255.0 if negate else (255.0 - px) / 255.0
    occ = np.full(img.shape, -1, dtype=np.int8)
    occ[p > occ_th] = 100
    occ[p < free_th] = 0
    occ = np.flipud(occ)
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    spec = GridSpec(float(meta['resolution']), float(origin[0]), float(origin[1]))
    return occ, spec


def load_pgm(path):
    with open(path, 'rb') as handle:
        data = handle.read()
    # P5 binario: magic, whitespace/comentarios, W H, maxval, raster
    tokens = []
    i = 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i] != ord('\n'):
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b'P5':
        raise ValueError(f'{path}: se esperaba PGM binario (P5)')
    w, h, maxv = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxv > 255:
        raise ValueError(f'{path}: maxval {maxv} no soportado')
    raster = np.frombuffer(data, dtype=np.uint8, count=w * h, offset=i + 1)
    return raster.reshape(h, w)


def geodesic_distances(free, sources):
    """BFS multi-fuente sobre celdas libres. Devuelve distancias en celdas (inf
    donde no llega). free: bool HxW; sources: iterable de (gx, gy)."""
    h, w = free.shape
    dist = np.full((h, w), np.inf, dtype=np.float64)
    queue = deque()
    for gx, gy in sources:
        if 0 <= gx < w and 0 <= gy < h and free[gy, gx]:
            dist[gy, gx] = 0.0
            queue.append((gx, gy))
    steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    while queue:
        gx, gy = queue.popleft()
        base = dist[gy, gx]
        for dx, dy, c in steps:
            nx, ny = gx + dx, gy + dy
            if 0 <= nx < w and 0 <= ny < h and free[ny, nx] and dist[ny, nx] > base + c:
                dist[ny, nx] = base + c
                queue.append((nx, ny))
    return dist


def farthest_point_waypoints(free, start_cell, coverage_cells, max_wp):
    """Farthest-point sampling geodesico hasta cubrir todo el alcanzable."""
    reach = np.isfinite(geodesic_distances(free, [start_cell]))
    reachable = free & reach
    selected = [start_cell]
    while len(selected) < max_wp:
        dist = geodesic_distances(free, selected)
        dist[~reachable] = -1.0
        idx = int(np.argmax(dist))
        gy, gx = np.unravel_index(idx, dist.shape)
        worst = dist[gy, gx]
        if worst <= coverage_cells:
            break
        selected.append((int(gx), int(gy)))
    dist = geodesic_distances(free, selected)
    worst = float(dist[reachable].max()) if reachable.any() else 0.0
    return selected, reachable, worst


def order_route(free, cells, start_cell):
    """Vecino mas cercano geodesico desde el arranque (el arranque no es waypoint)."""
    pending = [c for c in cells if c != start_cell]
    route = []
    cur = start_cell
    while pending:
        dist = geodesic_distances(free, [cur])
        best = min(pending, key=lambda c: dist[c[1], c[0]])
        route.append(best)
        pending.remove(best)
        cur = best
    return route


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--map', default='maps/maze_slam.yaml')
    ap.add_argument('--start', nargs=2, type=float, default=[2.45, -2.0],
                    metavar=('X', 'Y'), help='pose de arranque en frame map (m)')
    ap.add_argument('--coverage', type=float, default=1.5,
                    help='radio de cobertura geodesico por waypoint (m)')
    ap.add_argument('--inflation', type=float, default=0.26,
                    help='inflado del mapa (robot_radius + margen, m)')
    ap.add_argument('--max-wp', type=int, default=25)
    ap.add_argument('--out', default='config/parte_c/waypoints_laberinto.yaml')
    ap.add_argument('--preview', default='', help='PNG de control (opcional)')
    args = ap.parse_args()

    occ, spec = load_map(args.map)
    radius_cells = int(round(args.inflation / spec.resolution))
    inflated = inflate_occupancy(occ, radius_cells)
    free = (inflated == 0)
    if not free.any():
        raise SystemExit('el mapa inflado no tiene celdas libres')

    start_cell = world_to_grid(args.start[0], args.start[1], spec)
    snapped = nearest_free_cell(inflated, start_cell, max_radius=40)
    if snapped is None:
        raise SystemExit(f'la pose de arranque {args.start} no tiene celda libre cerca')
    start_cell = snapped

    coverage_cells = args.coverage / spec.resolution
    cells, reachable, worst = farthest_point_waypoints(
        free, start_cell, coverage_cells, args.max_wp)
    route = order_route(free, cells, start_cell)

    waypoints = []
    prev = grid_to_world(*start_cell, spec)
    for cell in route:
        x, y = grid_to_world(*cell, spec)
        yaw = math.atan2(y - prev[1], x - prev[0])
        assert free[cell[1], cell[0]], f'waypoint {cell} no esta en celda libre inflada'
        waypoints.append({'x': round(x, 2), 'y': round(y, 2),
                          'yaw': round(yaw, 2), 'scan': True})
        prev = (x, y)

    header = (
        '# Waypoints de busqueda del cono sobre el mapa del laberinto (frame map, m).\n'
        f'# GENERADO por scripts/gen_search_waypoints.py sobre {args.map}\n'
        f'#   start=({args.start[0]}, {args.start[1]})  coverage={args.coverage} m  '
        f'inflacion={args.inflation} m\n'
        f'# Cobertura: toda celda libre alcanzable queda a <= {max(worst * spec.resolution, args.coverage):.2f} m\n'
        '# (geodesicos) de un waypoint; todos caen en celda libre del mapa inflado.\n'
        '# La mision hace giro-scan de 360 en cada uno (scan_turn_steps).\n'
        '# Regenerar en el laboratorio si la pose de arranque real difiere:\n'
        '#   python3 scripts/gen_search_waypoints.py --start X Y\n'
    )
    with open(args.out, 'w') as handle:
        handle.write(header)
        yaml.safe_dump({'waypoints': waypoints}, handle,
                       default_flow_style=None, sort_keys=False)

    n_reach = int(reachable.sum())
    print(f'mapa {occ.shape[1]}x{occ.shape[0]} @ {spec.resolution} m; '
          f'{n_reach} celdas libres alcanzables ({n_reach * spec.resolution**2:.1f} m2)')
    print(f'{len(waypoints)} waypoints -> {args.out}; peor celda a '
          f'{worst * spec.resolution:.2f} m del waypoint mas cercano')

    if args.preview:
        render_preview(occ, inflated, spec, start_cell, route, args.preview)
        print(f'preview -> {args.preview}')


def render_preview(occ, inflated, spec, start_cell, route, path):
    import cv2
    h, w = occ.shape
    img = np.full((h, w, 3), 255, dtype=np.uint8)
    img[occ == -1] = (205, 205, 205)
    img[inflated == 100] = (230, 220, 210)
    img[occ == 100] = (60, 60, 60)
    pts = [start_cell] + route
    for a, b in zip(pts, pts[1:]):
        cv2.line(img, a, b, (200, 150, 60), 1)
    cv2.circle(img, start_cell, 4, (60, 160, 60), -1)
    for i, (gx, gy) in enumerate(route, start=1):
        cv2.circle(img, (gx, gy), 3, (50, 50, 220), -1)
        cv2.putText(img, str(i), (gx + 4, gy - 3), cv2.FONT_HERSHEY_PLAIN,
                    0.9, (50, 50, 220), 1)
    cv2.imwrite(path, np.flipud(img))  # y hacia arriba -> imagen y hacia abajo


if __name__ == '__main__':
    main()
