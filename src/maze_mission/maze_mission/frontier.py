"""Exploracion por fronteras sobre una occupancy grid (autocontenido, sin ROS).

Una celda frontera es una celda libre adyacente a una desconocida: el borde entre lo
mapeado y lo por mapear. Explorar = ir a la mejor frontera (utilidad = tamanio del
cluster menos alpha por el costo de camino), hasta que no queden fronteras (mapa cerrado).
Reemplaza la busqueda por waypoints fijos. Fundamento: teoricas 20 (fronteras) y 21
(utilidad = informacion - costo).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import binary_dilation, label

from maze_mission.occupancy import (
    GridSpec,
    grid_to_world,
    inflate_occupancy,
    nearest_free_cell,
    world_to_grid,
)

_NEIGH8 = np.ones((3, 3), dtype=bool)
_DIRS8 = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))


def find_frontier_cells(grid, lethal=50):
    grid = np.asarray(grid)
    free = (grid >= 0) & (grid < lethal)
    unknown = grid < 0
    unknown_adjacent = binary_dilation(unknown, structure=_NEIGH8)
    return free & unknown_adjacent


@dataclass(frozen=True)
class FrontierCluster:
    cells: tuple[tuple[int, int], ...]   # tupla de (gx, gy)
    size: int
    centroid_gx: int
    centroid_gy: int


def cluster_frontiers(frontier_mask, min_cells):
    labels, n = label(np.asarray(frontier_mask), structure=_NEIGH8)
    clusters = []
    for idx in range(1, n + 1):
        ys, xs = np.where(labels == idx)
        if xs.size < min_cells:
            continue
        cells = tuple((int(x), int(y)) for x, y in zip(xs, ys))
        clusters.append(FrontierCluster(
            cells=cells,
            size=int(xs.size),
            centroid_gx=int(round(float(xs.mean()))),
            centroid_gy=int(round(float(ys.mean()))),
        ))
    return clusters


def path_cost_field(inflated_grid, start_cell, lethal=50):
    grid = np.asarray(inflated_grid)
    h, w = grid.shape
    dist = np.full((h, w), -1, dtype=np.int32)
    sx, sy = start_cell
    if not (0 <= sx < w and 0 <= sy < h):
        return dist
    if not (0 <= grid[sy, sx] < lethal):
        return dist
    dist[sy, sx] = 0
    queue = deque([(sx, sy)])
    while queue:
        x, y = queue.popleft()
        d = dist[y, x]
        for dx, dy in _DIRS8:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and dist[ny, nx] < 0:
                if 0 <= grid[ny, nx] < lethal:
                    dist[ny, nx] = d + 1
                    queue.append((nx, ny))
    return dist


@dataclass(frozen=True)
class FrontierGoal:
    x: float
    y: float
    yaw: float
    utility: float
    cost: float
    gain: float


def select_frontier_goal(grid, spec, robot_xy, *, lethal=50,
                         inflation_cells, min_frontier_cells, alpha):
    grid = np.asarray(grid)
    frontier_mask = find_frontier_cells(grid, lethal)
    clusters = cluster_frontiers(frontier_mask, min_frontier_cells)
    if not clusters:
        return None

    # Solo las paredes reales inflan: el BFS ya no atraviesa desconocido (es -1, no
    # libre), asi que inflar el desconocido solo volveria inalcanzables las celdas
    # frontera (que por definicion lindan con desconocido). unknown_as_obstacle=False.
    inflated = inflate_occupancy(grid, inflation_cells,
                                 unknown_as_obstacle=False, lethal_threshold=lethal)
    start = world_to_grid(robot_xy[0], robot_xy[1], spec)
    start_free = nearest_free_cell(inflated, start, lethal_threshold=lethal)
    if start_free is None:
        return None
    start = start_free
    dist = path_cost_field(inflated, start, lethal)

    best = None
    for cluster in clusters:
        reachable = [(int(dist[gy, gx]), gx, gy)
                     for (gx, gy) in cluster.cells if dist[gy, gx] >= 0]
        if not reachable:
            continue
        cost, gx, gy = min(reachable)
        gain = float(cluster.size)
        utility = gain - alpha * cost
        if best is None or utility > best[0]:
            best = (utility, gx, gy, float(cost), gain)

    if best is None:
        return None
    utility, gx, gy, cost, gain = best
    x, y = grid_to_world(gx, gy, spec)
    yaw = math.atan2(y - robot_xy[1], x - robot_xy[0])
    return FrontierGoal(x=x, y=y, yaw=yaw, utility=utility, cost=cost, gain=gain)
