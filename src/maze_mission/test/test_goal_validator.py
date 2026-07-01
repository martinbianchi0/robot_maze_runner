"""Tests de la validacion geometrica de goals (invariante 'no cruzar paredes')."""
import numpy as np

from maze_mission.goal_validator import GoalStatus, ValidatorConfig, validate_goal
from maze_mission.occupancy import GridSpec


def _spec():
    return GridSpec(0.1, 0.0, 0.0)


def test_celda_libre_es_valida():
    grid = np.zeros((5, 5), dtype=np.int16)
    result = validate_goal(0.25, 0.25, grid, _spec())
    assert result.status == GoalStatus.VALID
    assert abs(result.x - 0.25) < 1e-9 and abs(result.y - 0.25) < 1e-9


def test_celda_ocupada_snap_a_libre_cercana():
    grid = np.zeros((5, 5), dtype=np.int16)
    grid[2, 2] = 100
    cfg = ValidatorConfig(snap_radius_cells=4, max_snap_dist_m=0.25)
    result = validate_goal(0.25, 0.25, grid, _spec(), cfg)
    assert result.status == GoalStatus.SNAPPED
    assert result.x is not None and result.y is not None


def test_cono_tras_pared_rechazado_por_distancia():
    # Todo ocupado salvo una celda libre lejana: el snap excede la cota metrica.
    grid = np.full((7, 7), 100, dtype=np.int16)
    grid[0, 0] = 0
    cfg = ValidatorConfig(snap_radius_cells=8, max_snap_dist_m=0.25)
    result = validate_goal(0.35, 0.35, grid, _spec(), cfg)
    assert result.status == GoalStatus.REJECTED


def test_todo_ocupado_rechazado():
    grid = np.full((3, 3), 100, dtype=np.int16)
    result = validate_goal(0.15, 0.15, grid, _spec())
    assert result.status == GoalStatus.REJECTED


def test_fuera_de_mapa_rechazado():
    grid = np.zeros((5, 5), dtype=np.int16)
    result = validate_goal(100.0, 100.0, grid, _spec())
    assert result.status == GoalStatus.REJECTED
    assert 'fuera' in result.reason


def test_desconocido_no_se_acepta_directo():
    grid = np.full((5, 5), -1, dtype=np.int16)   # todo desconocido
    result = validate_goal(0.25, 0.25, grid, _spec())
    assert result.status == GoalStatus.REJECTED
