"""Exploracion por fronteras sobre una occupancy grid (autocontenido, sin ROS).

Una celda frontera es una celda libre adyacente a una desconocida: el borde entre lo
mapeado y lo por mapear. Explorar = ir a la mejor frontera (utilidad = tamanio del
cluster menos alpha por el costo de camino), hasta que no queden fronteras (mapa cerrado).
Reemplaza la busqueda por waypoints fijos. Fundamento: teoricas 20 (fronteras) y 21
(utilidad = informacion - costo).
"""
from __future__ import annotations

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


def find_frontier_cells(grid, lethal=50):
    grid = np.asarray(grid)
    free = (grid >= 0) & (grid < lethal)
    unknown = grid < 0
    unknown_adjacent = binary_dilation(unknown, structure=_NEIGH8)
    return free & unknown_adjacent
