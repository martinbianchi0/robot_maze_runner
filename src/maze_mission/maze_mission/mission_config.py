"""Configuracion de la mision (topicos, timeouts, umbrales).

Es un dataclass de perfil, desacoplado de ROS para poder testearlo. El
mission_node declara parametros con estos mismos nombres y arma un MissionConfig
via from_dict. Los perfiles sim/bag/real setean estos valores en el YAML.

Interfaz de navegacion (pila de toma-2): la mision consume /amcl_pose
(PoseWithCovarianceStamped, de la MCL) como pose, /map (OccupancyGrid latched,
que publica map_publisher) como mapa a validar, y /nav_state (String) para saber
en que anda el navigator. No hay /nav_debug ni /global_costmap en toma-2: la
mision infla /map por su cuenta (occupancy.inflate_occupancy).
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MissionConfig:
    # Topicos que consume/produce la mision (todos parametrizables sim/bag/real).
    goal_pose_topic: str = '/goal_pose'
    nav_state_topic: str = '/nav_state'
    map_topic: str = '/map'
    pose_topic: str = '/amcl_pose'
    pose_topic_type: str = 'pose_with_covariance'   # pose_with_covariance | pose_stamped | odometry
    scan_topic: str = '/scan'
    detections_topic: str = 'cone_detections'
    mission_state_topic: str = '/mission_state'
    map_frame: str = 'map'
    # Inflado del mapa para validar goals (mismo criterio que el navigator:
    # robot_radius 0.14 + inflation 0.12 = 0.26 m).
    inflation_radius_m: float = 0.26
    # LIDAR-fusion (validado en C1 contra laberinto_conos):
    #   offset del montaje del LIDAR respecto de base. TB4 real = -pi/2 (LIDAR a
    #   +90 deg); sim con LIDAR alineado = 0. Modo 'nearest' gana a 'median'.
    lidar_yaw_offset: float = -math.pi / 2
    lidar_offset_x: float = -0.04
    lidar_sector_halfwidth: int = 3
    cone_range_mode: str = 'nearest'
    # Busqueda / percepcion.
    waypoints_file: str = ''
    detection_confidence_min: float = 0.4
    detection_stable_frames: int = 3
    verify_area_px_min: int = 1500          # area para confirmar cono cercano
    cone_standoff_m: float = 0.30           # frenar a esta distancia del cono
    max_replans: int = 2                    # reintentos antes de FAILURE por cono
    # Tiempos (s).
    control_hz: float = 5.0
    localize_timeout_s: float = 30.0
    goal_timeout_s: float = 30.0
    blocked_timeout_s: float = 6.0
    verify_timeout_s: float = 5.0

    @classmethod
    def from_dict(cls, values: dict) -> 'MissionConfig':
        valid = {f.name for f in dataclasses.fields(cls)}
        unknown = set(values) - valid
        if unknown:
            raise TypeError(f'claves de mision desconocidas: {sorted(unknown)}')
        return dataclasses.replace(cls(), **values)

    @classmethod
    def field_defaults(cls) -> dict:
        return {f.name: getattr(cls(), f.name) for f in dataclasses.fields(cls)}
