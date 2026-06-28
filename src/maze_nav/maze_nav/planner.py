"""A* y utilidades de colision sobre grillas."""

import heapq
import math
from collections import deque
from dataclasses import dataclass

import numpy as np


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


def bresenham_cells(a, b):
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    cells = []
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells


def is_segment_free(grid, start, goal, lethal_threshold=50, allow_unknown=False):
    return all(
        is_cell_free(grid, cell, lethal_threshold, allow_unknown)
        for cell in bresenham_cells(start, goal)
    )


def nearest_free_cell(grid, start, max_radius=12, lethal_threshold=50, allow_unknown=False):
    if is_cell_free(grid, start, lethal_threshold, allow_unknown):
        return start
    visited = {start}
    q = deque([(start, 0)])
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while q:
        (gx, gy), dist = q.popleft()
        if dist >= max_radius:
            continue
        for dx, dy in dirs:
            nxt = (gx + dx, gy + dy)
            if nxt in visited or not in_bounds(grid, nxt):
                continue
            if is_cell_free(grid, nxt, lethal_threshold, allow_unknown):
                return nxt
            visited.add(nxt)
            q.append((nxt, dist + 1))
    return None


def _neighbors(grid, cell, allow_diagonal, lethal_threshold, allow_unknown):
    gx, gy = cell
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    if allow_diagonal:
        dirs += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
    for dx, dy in dirs:
        nxt = (gx + dx, gy + dy)
        if not is_cell_free(grid, nxt, lethal_threshold, allow_unknown):
            continue
        if dx and dy:
            # Evita cortar esquinas entre dos obstaculos inflados.
            if not is_cell_free(grid, (gx + dx, gy), lethal_threshold, allow_unknown):
                continue
            if not is_cell_free(grid, (gx, gy + dy), lethal_threshold, allow_unknown):
                continue
        yield nxt, math.hypot(dx, dy)


def _heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def astar(grid, start, goal, allow_diagonal=True, lethal_threshold=50, allow_unknown=False):
    """Planifica sobre grilla. Devuelve lista de celdas o [] si no hay camino."""
    if not is_cell_free(grid, start, lethal_threshold, allow_unknown):
        return []
    if not is_cell_free(grid, goal, lethal_threshold, allow_unknown):
        return []

    frontier = []
    heapq.heappush(frontier, (0.0, start))
    came_from = {start: None}
    cost_so_far = {start: 0.0}

    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            break
        for nxt, step_cost in _neighbors(
            grid, current, allow_diagonal, lethal_threshold, allow_unknown
        ):
            new_cost = cost_so_far[current] + step_cost
            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                priority = new_cost + _heuristic(nxt, goal)
                heapq.heappush(frontier, (priority, nxt))
                came_from[nxt] = current

    if goal not in came_from:
        return []

    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


def simplify_path(grid, path, lethal_threshold=50, allow_unknown=False):
    """Reduce waypoints manteniendo linea de vista libre."""
    if len(path) <= 2:
        return list(path)
    simplified = [path[0]]
    anchor_idx = 0
    probe_idx = 2
    while probe_idx < len(path):
        if not is_segment_free(
            grid, path[anchor_idx], path[probe_idx], lethal_threshold, allow_unknown
        ):
            simplified.append(path[probe_idx - 1])
            anchor_idx = probe_idx - 1
        probe_idx += 1
    simplified.append(path[-1])
    return simplified


def limit_path_stride(path, max_stride_cells=4):
    """Mantiene waypoints densos para el follower.

    A diferencia de `simplify_path`, no reemplaza un pasillo por un segmento
    largo. Solo reduce puntos consecutivos si hay demasiados para visualizar.
    """
    if len(path) <= 2 or max_stride_cells <= 1:
        return list(path)
    out = [path[0]]
    last = path[0]
    for cell in path[1:-1]:
        if max(abs(cell[0] - last[0]), abs(cell[1] - last[1])) >= max_stride_cells:
            out.append(cell)
            last = cell
    if out[-1] != path[-1]:
        out.append(path[-1])
    return out


def cells_to_world_path(cells, spec):
    return [grid_to_world(gx, gy, spec) for gx, gy in cells]


def path_length(cells):
    if len(cells) < 2:
        return 0.0
    pts = np.asarray(cells, dtype=float)
    return float(np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1])).sum())
