import math

import pytest

from maze_nav.follower import STATE_SAFE_DRIVE, STATE_STUCK_RECOVERY, STATE_WATCHDOG_STOP
from maze_nav.safe_drive import SafeDriveConfig, SafeDriveController, analyze_scan


def make_scan(front=2.0, left=2.0, right=2.0):
    ranges = []
    angle_min = -math.pi
    angle_increment = math.radians(1.0)
    for i in range(361):
        angle = angle_min + i * angle_increment
        deg = math.degrees(angle)
        if -25 <= deg <= 25:
            ranges.append(front)
        elif 25 < deg <= 105:
            ranges.append(left)
        elif -105 <= deg < -25:
            ranges.append(right)
        else:
            ranges.append(2.0)
    return ranges, angle_min, angle_increment


def controller():
    return SafeDriveController(SafeDriveConfig(max_linear_mps=0.05, max_angular_rps=0.25))


def test_safe_drive_moves_forward_when_front_is_clear():
    ranges, angle_min, angle_increment = make_scan(front=2.0, left=2.0, right=2.0)
    sectors = analyze_scan(ranges, angle_min, angle_increment, 0.05, 3.5)

    cmd = controller().compute(sectors, scan_age_s=0.0)

    assert cmd.state == STATE_SAFE_DRIVE
    assert cmd.linear > 0.0
    assert abs(cmd.angular) < 1e-6


def test_scan_analysis_ignores_isolated_front_speckle_for_nominal_clearance():
    ranges, angle_min, angle_increment = make_scan(front=1.2, left=2.0, right=2.0)
    # Punto aislado justo adelante: no debe convertir todo el frente en bloqueado.
    ranges[180] = 0.22

    sectors = analyze_scan(ranges, angle_min, angle_increment, 0.05, 3.5)

    assert sectors.front_min > 1.0
    assert sectors.front > 1.0


def test_scan_analysis_keeps_clustered_front_obstacle_as_emergency():
    ranges, angle_min, angle_increment = make_scan(front=1.2, left=2.0, right=2.0)
    for index in range(176, 185):
        ranges[index] = 0.13

    sectors = analyze_scan(ranges, angle_min, angle_increment, 0.05, 3.5)

    assert sectors.front_min == pytest.approx(0.13)


def test_scan_analysis_wraps_zero_to_360_degree_scans():
    ranges = [2.0] * 360
    # En Gazebo/TurtleBot el LaserScan puede venir como 0..359 grados.
    # 350 grados es frente-derecha y debe entrar al frente; 300 al sector derecho.
    for index in range(348, 353):
        ranges[index] = 0.42
    ranges[300] = 0.55
    sectors = analyze_scan(ranges, 0.0, math.radians(1.0), 0.05, 3.5)

    assert sectors.front_min == pytest.approx(0.42)
    assert sectors.right == pytest.approx(0.55)


def test_safe_drive_turns_toward_more_open_side_when_blocked():
    ranges, angle_min, angle_increment = make_scan(front=0.20, left=1.5, right=0.4)
    sectors = analyze_scan(ranges, angle_min, angle_increment, 0.05, 3.5)

    cmd = controller().compute(sectors, scan_age_s=0.0)

    assert cmd.state == STATE_STUCK_RECOVERY
    assert cmd.linear == 0.0
    assert cmd.angular > 0.0


def test_safe_drive_watchdog_stops_on_stale_scan():
    ranges, angle_min, angle_increment = make_scan()
    sectors = analyze_scan(ranges, angle_min, angle_increment, 0.05, 3.5)

    cmd = controller().compute(sectors, scan_age_s=2.0)

    assert cmd.state == STATE_WATCHDOG_STOP
    assert cmd.linear == 0.0
    assert cmd.angular == 0.0
