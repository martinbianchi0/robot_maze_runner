"""Primitivas de grilla de ocupacion para validar goals (autocontenido).

Se vendorizan aca (en vez de importar de maze_nav) porque la pila de navegacion
de toma-2 no expone estas funciones como modulo reutilizable, y porque desacoplar
`maze_mission` de los internals de maze_nav lo hace robusto a los cambios de la
Parte B (solo dependemos del contrato de topicos). Son utilidades genericas de
OccupancyGrid, no especificas de maze_nav.

El inflado replica el criterio del `navigator` de toma-2: obstaculo = pared
mapeada (occ>=lethal) O desconocido (occ<0); se bloquea toda celda a <=
(robot_radius+inflation) de un obstaculo. Asi, un goal que pasa la validacion de
la mision cae en el mismo espacio navegable que usara el planner.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt


@dataclass(frozen=True)
class GridSpec:
    resolution: float
    origin_x: float
    origin_y: float


def world_to_grid(x, y, spec):
    eps = 1e-9
    gx = int(math.floor((x - spec.origin_x) / spec.resolution + eps))
    gy = int(math.floor((y - spec.origin_y) / spec.resolution + eps))
    return gx, gy


def grid_to_world(gx, gy, spec):
    x = spec.origin_x + (gx + 0.5) * spec.resolution
    y = spec.origin_y + (gy + 0.5) * spec.resolution
    return x, y


def in_bounds(grid, cell):
    gx, gy = cell
    h, w = grid.shape
    return 0 <= gx < w and 0 <= gy < h


def is_cell_free(grid, cell, lethal_threshold=50, allow_unknown=False):
    if not in_bounds(grid, cell):
        return False
    gx, gy = cell
    value = int(grid[gy, gx])
    if value < 0:
        return bool(allow_unknown)
    return value < lethal_threshold


def nearest_free_cell(grid, start, max_radius=12, lethal_threshold=50, allow_unknown=False):
    if is_cell_free(grid, start, lethal_threshold, allow_unknown):
        return start
    visited = {start}
    queue = deque([(start, 0)])
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while queue:
        (gx, gy), dist = queue.popleft()
        if dist >= max_radius:
            continue
        for dx, dy in dirs:
            nxt = (gx + dx, gy + dy)
            if nxt in visited or not in_bounds(grid, nxt):
                continue
            if is_cell_free(grid, nxt, lethal_threshold, allow_unknown):
                return nxt
            visited.add(nxt)
            queue.append((nxt, dist + 1))
    return None


def inflate_occupancy(grid, radius_cells, *, unknown_as_obstacle=True, lethal_threshold=50):
    """Marca como lethal (100) toda celda libre a <= radius_cells de un obstaculo.

    Obstaculo = pared mapeada (>= lethal_threshold). Si unknown_as_obstacle (default),
    tambien el desconocido (< 0) cuenta como obstaculo, igual que el navigator de toma-2
    (para validar goals del cono). unknown_as_obstacle=False lo excluye, para usar el
    mapa como campo de costo de exploracion sin que lo desconocido bloquee. Los
    desconocidos quedan en -1 (no navegables via is_cell_free con allow_unknown=False).
    Devuelve una grilla nueva.
    """
    grid = np.asarray(grid)
    inflated = grid.copy()
    obstacles = grid >= lethal_threshold
    if unknown_as_obstacle:
        obstacles = obstacles | (grid < 0)
    if radius_cells <= 0 or not obstacles.any():
        return inflated
    dist_cells = distance_transform_edt(~obstacles)
    inflated[(dist_cells <= radius_cells) & (grid >= 0)] = 100
    return inflated
