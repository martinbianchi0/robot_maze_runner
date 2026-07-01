"""Estimacion de la posicion del cono en el mundo a partir de una deteccion.

La camara monocular da bearing confiable pero rango debil. Se preven DOS
estrategias (la primaria se decide con datos en la etapa C1):

- LIDAR-fusion: bearing (vision) + rango del /scan en ese angulo -> punto metrico.
- Bearing-only servoing: micro-goals cortos sobre el rayo del bearing, validados
  y re-detectando, hasta acercarse.

Este modulo concentra la GEOMETRIA pura (testeable sin ROS).

Geometria del TurtleBot4 real (resuelta de la TF estatica del bag laberinto_conos):
- La camara OAK-D mira al frente: la cadena camara->base es identidad en yaw, asi
  que bearing_base = atan2(cx - u, fx) (+ a la izquierda del robot, base +y).
- El RPLIDAR (`rplidar_link`) esta montado con yaw +90deg respecto de base
  (`shell_link -> rplidar_link`). Por eso el angulo 0 del scan apunta a la
  IZQUIERDA del robot, y para un bearing de camara el indice del scan se busca en
  `angulo_scan = bearing_base + lidar_yaw_offset`, con lidar_yaw_offset = -pi/2 en
  el TB4 real (0 si el LIDAR estuviera alineado con la camara, p.ej. en sim).
- El origen del LIDAR esta ~4 cm detras del origen de base (lidar_offset_x=-0.04).

El bearing se asume expresado en el frame base del robot; el offset optico de la
camara ya lo resuelve el detector (que trabaja alineado a base en yaw).
"""
from __future__ import annotations

import math
from enum import Enum
from typing import Optional, Tuple

# Valores por defecto del TurtleBot4 real (de la TF estatica del bag).
TB4_LIDAR_YAW_OFFSET = -math.pi / 2.0
TB4_LIDAR_OFFSET_X = -0.04
TB4_LIDAR_OFFSET_Y = 0.0


class EstimatorStrategy(Enum):
    LIDAR_FUSION = 'lidar_fusion'
    BEARING_SERVOING = 'bearing_servoing'


def base_to_world(px: float, py: float, pyaw: float, bx: float, by: float) -> Tuple[float, float]:
    """Transforma un punto (bx,by) del frame base del robot al frame mundo."""
    c, s = math.cos(pyaw), math.sin(pyaw)
    return px + c * bx - s * by, py + s * bx + c * by


def project_bearing_range_to_world(robot_x: float, robot_y: float, robot_yaw: float,
                                   bearing_rad: float, range_m: float) -> Tuple[float, float]:
    """Punto de mundo dado (pose robot, bearing en base, rango) desde el origen de base."""
    return base_to_world(robot_x, robot_y, robot_yaw,
                         range_m * math.cos(bearing_rad), range_m * math.sin(bearing_rad))


def micro_goal_along_bearing(robot_x: float, robot_y: float, robot_yaw: float,
                             bearing_rad: float, step_m: float) -> Tuple[float, float]:
    """Micro-goal corto sobre el rayo del bearing (estrategia servoing)."""
    return project_bearing_range_to_world(robot_x, robot_y, robot_yaw, bearing_rad, step_m)


def scan_angle_from_bearing(bearing_base: float, lidar_yaw_offset: float = TB4_LIDAR_YAW_OFFSET) -> float:
    """Convierte un bearing en base al angulo correspondiente en el frame del scan."""
    return bearing_base + lidar_yaw_offset


def range_from_scan(scan_angle: float, angle_min: float, angle_increment: float,
                    ranges, sector_halfwidth: int = 2) -> Optional[float]:
    """Rango en `scan_angle` (frame del scan): mediana de un sector de +/- rayos.

    Devuelve None si no hay lecturas validas (finitas y > 0) en el sector, lo que
    obliga a caer al fallback servoing. `ranges` es la lista/array del LaserScan.
    """
    if angle_increment == 0.0 or len(ranges) == 0:
        return None
    center = int(round((scan_angle - angle_min) / angle_increment))
    lo = max(0, center - sector_halfwidth)
    hi = min(len(ranges) - 1, center + sector_halfwidth)
    if lo > hi:
        return None
    valid = [r for r in ranges[lo:hi + 1] if math.isfinite(r) and r > 0.0]
    if not valid:
        return None
    valid.sort()
    return valid[len(valid) // 2]


def range_from_scan_nearest(scan_angle: float, angle_min: float, angle_increment: float,
                            ranges, sector_halfwidth: int = 3,
                            cluster_tol: float = 0.15) -> Optional[float]:
    """Rango al objeto MAS CERCANO del sector (el cono, no la pared detras).

    Toma el minimo del sector y devuelve la mediana de las lecturas dentro de
    [rmin, rmin+cluster_tol]. Robusto a la pared de fondo (queda excluida por
    estar mas lejos) y a un rayo espurio (la mediana del cluster lo diluye).
    """
    if angle_increment == 0.0 or len(ranges) == 0:
        return None
    center = int(round((scan_angle - angle_min) / angle_increment))
    lo = max(0, center - sector_halfwidth)
    hi = min(len(ranges) - 1, center + sector_halfwidth)
    if lo > hi:
        return None
    valid = [r for r in ranges[lo:hi + 1] if math.isfinite(r) and r > 0.0]
    if not valid:
        return None
    rmin = min(valid)
    near = sorted(r for r in valid if r <= rmin + cluster_tol)
    return near[len(near) // 2]


def cone_world_from_lidar(bearing_base: float, pose: Tuple[float, float, float],
                          scan_ranges, angle_min: float, angle_increment: float,
                          lidar_yaw_offset: float = TB4_LIDAR_YAW_OFFSET,
                          lidar_offset_x: float = TB4_LIDAR_OFFSET_X,
                          lidar_offset_y: float = TB4_LIDAR_OFFSET_Y,
                          sector_halfwidth: int = 3,
                          range_mode: str = 'nearest'):
    """LIDAR-fusion: (bearing de camara + pose) -> (x, y de mundo, rango).

    Busca el rango del /scan en el angulo del scan que corresponde al bearing,
    arma el punto del cono en frame base (aplicando el offset del LIDAR) y lo pasa
    a mundo con la pose del robot. Devuelve None si no hay rango valido.
    """
    px, py, pyaw = pose
    scan_angle = scan_angle_from_bearing(bearing_base, lidar_yaw_offset)
    if range_mode == 'median':
        r = range_from_scan(scan_angle, angle_min, angle_increment, scan_ranges, sector_halfwidth)
    else:
        r = range_from_scan_nearest(scan_angle, angle_min, angle_increment, scan_ranges, sector_halfwidth)
    if r is None:
        return None
    bx = r * math.cos(bearing_base) + lidar_offset_x
    by = r * math.sin(bearing_base) + lidar_offset_y
    wx, wy = base_to_world(px, py, pyaw, bx, by)
    return (wx, wy, r)
