"""Waypoints de busqueda sobre el mapa.

Como el cono esta en un lugar desconocido, la busqueda recorre una lista
ordenada de waypoints que cubren el laberinto y hace un giro-scan en cada uno
para maximizar la cobertura visual. NO es exploracion de fronteras. Los
waypoints se cargan desde un YAML del perfil (config/parte_c/waypoints_*.yaml).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

import yaml


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float = 0.0     # orientacion sugerida (rad)
    scan: bool = True    # si hacer giro-scan al llegar


class WaypointRoute:
    """Iterador con estado sobre una lista de waypoints de busqueda."""

    def __init__(self, waypoints: List[Waypoint]):
        self._waypoints = list(waypoints)
        self._index = 0

    def __len__(self) -> int:
        return len(self._waypoints)

    def reset(self) -> None:
        self._index = 0

    def current(self) -> Optional[Waypoint]:
        if self._index < len(self._waypoints):
            return self._waypoints[self._index]
        return None

    def advance(self) -> Optional[Waypoint]:
        self._index += 1
        return self.current()

    def exhausted(self) -> bool:
        return self._index >= len(self._waypoints)


def scan_turn_yaws(base_yaw: float, steps: int) -> List[float]:
    """Yaws sucesivos del giro-scan de 360 deg en un waypoint.

    Devuelve `steps` orientaciones equiespaciadas (2*pi/steps) a partir de
    base_yaw, terminando de vuelta en base_yaw. Como el robot rota continuo
    entre goals (ALIGNING), la camara barre la vuelta completa sin importar
    su HFOV. steps <= 0 -> sin giro-scan.
    """
    if steps <= 0:
        return []
    delta = 2.0 * math.pi / steps
    return [wrap_angle(base_yaw + (k + 1) * delta) for k in range(steps)]


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def load_waypoints(path: str) -> List[Waypoint]:
    with open(path, 'r') as handle:
        raw = yaml.safe_load(handle) or {}
    items = raw.get('waypoints', [])
    return [
        Waypoint(
            x=float(item['x']),
            y=float(item['y']),
            yaw=float(item.get('yaw', 0.0)),
            scan=bool(item.get('scan', True)),
        )
        for item in items
    ]
