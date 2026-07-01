"""Tests de la geometria pura de estimacion cono->mundo."""
import math

from maze_mission.cone_goal_estimator import (
    TB4_LIDAR_OFFSET_X,
    cone_world_from_lidar,
    micro_goal_along_bearing,
    project_bearing_range_to_world,
    range_from_scan,
    range_from_scan_nearest,
    scan_angle_from_bearing,
)


def test_proyeccion_hacia_adelante():
    x, y = project_bearing_range_to_world(0.0, 0.0, 0.0, 0.0, 1.0)
    assert abs(x - 1.0) < 1e-9 and abs(y - 0.0) < 1e-9


def test_proyeccion_bearing_a_la_izquierda():
    x, y = project_bearing_range_to_world(0.0, 0.0, 0.0, math.pi / 2, 1.0)
    assert abs(x - 0.0) < 1e-9 and abs(y - 1.0) < 1e-9


def test_proyeccion_con_yaw_del_robot():
    x, y = project_bearing_range_to_world(0.0, 0.0, math.pi / 2, 0.0, 2.0)
    assert abs(x - 0.0) < 1e-9 and abs(y - 2.0) < 1e-9


def test_micro_goal_es_corto():
    x, y = micro_goal_along_bearing(0.0, 0.0, 0.0, 0.0, 0.5)
    assert abs(x - 0.5) < 1e-9 and abs(y - 0.0) < 1e-9


def test_range_from_scan_mediana():
    ranges = [1.0, 1.1, 1.2, 5.0, 5.0]
    # bearing 0 con angle_min 0 e incremento 1 -> centro indice 0; sector +/-2.
    value = range_from_scan(0.0, 0.0, 1.0, ranges, sector_halfwidth=2)
    assert value == 1.1   # mediana de [1.0, 1.1, 1.2]


def test_range_from_scan_sin_validos():
    ranges = [float('inf'), 0.0, float('nan')]
    assert range_from_scan(0.0, 0.0, 1.0, ranges, sector_halfwidth=1) is None


def test_scan_angle_offset_tb4():
    # cono al frente (bearing 0) -> scan a -90deg; a la izquierda (+90) -> scan a 0
    assert abs(scan_angle_from_bearing(0.0) - (-math.pi / 2)) < 1e-9
    assert abs(scan_angle_from_bearing(math.pi / 2)) < 1e-9


def test_cone_world_from_lidar_front():
    # 4 rayos: angle_min=-pi, incremento pi/2 -> [-pi,-pi/2,0,pi/2].
    # bearing 0 -> scan angle -pi/2 -> indice 1. sector_halfwidth=0 => solo ese rayo.
    ranges = [9.0, 1.0, 9.0, 9.0]
    out = cone_world_from_lidar(0.0, (0.0, 0.0, 0.0), ranges,
                                angle_min=-math.pi, angle_increment=math.pi / 2,
                                sector_halfwidth=0)
    assert out is not None
    wx, wy, r = out
    assert abs(r - 1.0) < 1e-9
    assert abs(wx - (1.0 + TB4_LIDAR_OFFSET_X)) < 1e-9   # 0.96
    assert abs(wy) < 1e-9


def test_cone_world_from_lidar_con_yaw():
    # robot en (2,3) mirando +90deg; cono al frente r=1 -> base (0.96,0) -> mundo (2,3.96)
    ranges = [9.0, 1.0, 9.0, 9.0]
    wx, wy, r = cone_world_from_lidar(0.0, (2.0, 3.0, math.pi / 2), ranges,
                                      angle_min=-math.pi, angle_increment=math.pi / 2,
                                      sector_halfwidth=0)
    assert abs(wx - 2.0) < 1e-9 and abs(wy - 3.96) < 1e-9


def test_cone_world_from_lidar_sin_rango():
    ranges = [float('inf')] * 4
    assert cone_world_from_lidar(0.0, (0.0, 0.0, 0.0), ranges,
                                 angle_min=-math.pi, angle_increment=math.pi / 2,
                                 sector_halfwidth=0) is None


def test_range_from_scan_nearest_excluye_pared_de_fondo():
    ranges = [1.0, 1.02, 5.0, 5.0, 5.0]
    r = range_from_scan_nearest(0.0, 0.0, 1.0, ranges, sector_halfwidth=2, cluster_tol=0.15)
    assert abs(r - 1.02) < 1e-9   # mediana del cluster cercano [1.0, 1.02]
