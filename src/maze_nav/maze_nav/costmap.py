"""Costmap navegable e inflado de obstaculos."""

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt


FREE = 0
LETHAL = 100
UNKNOWN = -1


@dataclass(frozen=True)
class CostmapConfig:
    inflation_radius_m: float = 0.20
    occupied_threshold: int = 50
    unknown_as_obstacle: bool = True


def obstacle_mask(occupancy_grid, occupied_threshold=50, unknown_as_obstacle=True):
    grid = np.asarray(occupancy_grid)
    mask = grid >= occupied_threshold
    if unknown_as_obstacle:
        mask = mask | (grid < 0)
    return mask


def inflate_obstacles(occupancy_grid, resolution, config=None):
    """Devuelve costmap int8 con obstaculos inflados.

    Default conservador: lo desconocido cuenta como obstaculo. Si se desactiva,
    lo desconocido queda como -1 para que el planner pueda decidir aparte.
    """
    if config is None:
        config = CostmapConfig()
    grid = np.asarray(occupancy_grid)
    obstacles = obstacle_mask(
        grid,
        occupied_threshold=config.occupied_threshold,
        unknown_as_obstacle=config.unknown_as_obstacle,
    )
    if obstacles.size == 0:
        return grid.astype(np.int8, copy=True)

    distances_cells = distance_transform_edt(~obstacles)
    inflated = distances_cells * float(resolution) <= float(config.inflation_radius_m)

    costmap = np.full(grid.shape, FREE, dtype=np.int8)
    costmap[inflated] = LETHAL
    if not config.unknown_as_obstacle:
        costmap[grid < 0] = UNKNOWN
    return costmap


def overlay_obstacles(costmap, obstacle_cells):
    """Copia un costmap y marca celdas adicionales como letales."""
    out = np.array(costmap, dtype=np.int8, copy=True)
    h, w = out.shape
    for gx, gy in obstacle_cells:
        if 0 <= gx < w and 0 <= gy < h:
            out[gy, gx] = LETHAL
    return out
