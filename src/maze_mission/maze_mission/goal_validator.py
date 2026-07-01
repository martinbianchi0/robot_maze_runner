"""Validacion geometrica de goals de vision contra el mapa navegable.

Invariante de seguridad de la Parte C: ningun goal derivado de una deteccion
visual llega a /goal_pose sin pasar por aca. Se valida contra el /map estatico
(el mismo que consume el navigator de toma-2) inflado con el mismo criterio que
el navigator (robot_radius+inflation), de modo que un goal que pasa la validacion
cae en el mismo espacio navegable que usara el planner. La grilla ya inflada la
arma mission_node con maze_mission.occupancy.inflate_occupancy.

Regla dura de la consigna: si el cono se ve a traves de una reja, el punto
estimado cae sobre la pared -> celda no libre -> se rechaza. El snap a la celda
libre mas cercana esta acotado (radio y distancia metrica chicos) para tolerar
ruido metrico sin poder "saltar" al otro lado de una pared.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from maze_mission.occupancy import (
    GridSpec,
    grid_to_world,
    in_bounds,
    is_cell_free,
    nearest_free_cell,
    world_to_grid,
)


class GoalStatus(Enum):
    VALID = 'VALID'        # la celda del cono ya es navegable
    SNAPPED = 'SNAPPED'    # se corrigio a una celda libre muy cercana
    REJECTED = 'REJECTED'  # inalcanzable/no navegable (p.ej. cono tras pared)


@dataclass(frozen=True)
class ValidationResult:
    status: GoalStatus
    x: Optional[float] = None
    y: Optional[float] = None
    reason: str = ''


@dataclass(frozen=True)
class ValidatorConfig:
    lethal_threshold: int = 50
    allow_unknown: bool = False       # una celda desconocida no se acepta directo
    snap_radius_cells: int = 6        # R_SNAP chico: no puede cruzar una pared
    max_snap_dist_m: float = 0.25     # cota metrica del snap


def validate_goal(x, y, grid, spec: GridSpec,
                  cfg: ValidatorConfig = ValidatorConfig()) -> ValidationResult:
    """Valida un goal de mundo (x,y) contra el costmap `grid` (numpy HxW)."""
    cell = world_to_grid(x, y, spec)
    if not in_bounds(grid, cell):
        return ValidationResult(GoalStatus.REJECTED, reason='goal fuera del mapa')
    if is_cell_free(grid, cell, cfg.lethal_threshold, cfg.allow_unknown):
        return ValidationResult(GoalStatus.VALID, x, y, 'celda libre')
    snap = nearest_free_cell(grid, cell, cfg.snap_radius_cells,
                             cfg.lethal_threshold, cfg.allow_unknown)
    if snap is None:
        return ValidationResult(
            GoalStatus.REJECTED, reason='sin celda libre cercana (posible cono tras pared)')
    sx, sy = grid_to_world(snap[0], snap[1], spec)
    if math.hypot(sx - x, sy - y) > cfg.max_snap_dist_m:
        return ValidationResult(
            GoalStatus.REJECTED,
            reason='celda libre mas cercana demasiado lejos (posible cono tras pared)')
    return ValidationResult(GoalStatus.SNAPPED, sx, sy, 'snap a celda libre cercana')
