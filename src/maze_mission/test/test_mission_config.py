"""Tests de la config de mision (perfil sim/bag/real)."""
import math

import pytest

from maze_mission.mission_config import MissionConfig


def test_lidar_fusion_defaults_tb4():
    cfg = MissionConfig()
    assert abs(cfg.lidar_yaw_offset + math.pi / 2) < 1e-6   # -90 deg (TB4 real)
    assert cfg.cone_range_mode == 'nearest'


def test_defaults_y_override():
    cfg = MissionConfig.from_dict({'pose_topic': '/tb4_0/belief', 'control_hz': 4.0})
    assert cfg.pose_topic == '/tb4_0/belief'
    assert cfg.control_hz == 4.0
    assert cfg.goal_pose_topic == MissionConfig().goal_pose_topic


def test_clave_desconocida_falla():
    with pytest.raises(TypeError):
        MissionConfig.from_dict({'topic_inexistente': 'x'})


def test_field_defaults_cubre_todos_los_campos():
    defaults = MissionConfig.field_defaults()
    assert 'goal_pose_topic' in defaults
    assert defaults['pose_topic_type'] == 'pose_with_covariance'
    assert defaults['map_topic'] == '/map'


def test_config_tiene_parametros_de_frontera():
    cfg = MissionConfig()
    assert cfg.frontier_alpha == 0.1
    assert cfg.frontier_min_cells == 4


def test_from_dict_acepta_frontera():
    cfg = MissionConfig.from_dict({'frontier_alpha': 0.5, 'frontier_min_cells': 8})
    assert cfg.frontier_alpha == 0.5 and cfg.frontier_min_cells == 8
