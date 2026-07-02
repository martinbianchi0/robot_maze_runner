import math

import pytest

from maze_mission.search_waypoints import scan_turn_yaws, wrap_angle


def test_tres_pasos_cubren_la_vuelta():
    yaws = scan_turn_yaws(0.0, 3)
    assert len(yaws) == 3
    esperados = [2.0 * math.pi / 3.0, -2.0 * math.pi / 3.0, 0.0]
    for got, exp in zip(yaws, esperados):
        assert abs(wrap_angle(got - exp)) < 1e-9


def test_termina_en_el_yaw_base():
    for base in (0.0, 1.0, -2.5):
        for steps in (2, 3, 4, 6):
            yaws = scan_turn_yaws(base, steps)
            assert len(yaws) == steps
            assert abs(wrap_angle(yaws[-1] - base)) < 1e-9


def test_pasos_equiespaciados():
    base = 0.7
    steps = 4
    yaws = [base] + scan_turn_yaws(base, steps)
    for prev, cur in zip(yaws, yaws[1:]):
        assert abs(wrap_angle(cur - prev) - 2.0 * math.pi / steps) < 1e-9


@pytest.mark.parametrize('steps', [0, -1])
def test_cero_pasos_sin_giro(steps):
    assert scan_turn_yaws(1.0, steps) == []


def test_yaws_envueltos():
    for yaw in scan_turn_yaws(3.0, 5):
        assert -math.pi <= yaw <= math.pi
