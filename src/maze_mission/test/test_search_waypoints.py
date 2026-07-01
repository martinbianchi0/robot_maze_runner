"""Tests de la ruta de waypoints de busqueda."""
from maze_mission.search_waypoints import Waypoint, WaypointRoute, load_waypoints


def test_route_avanza_y_se_agota():
    route = WaypointRoute([Waypoint(0.0, 0.0), Waypoint(1.0, 1.0)])
    assert len(route) == 2
    assert route.current().x == 0.0
    assert route.advance().x == 1.0
    assert not route.exhausted()
    assert route.advance() is None
    assert route.exhausted()


def test_route_reset():
    route = WaypointRoute([Waypoint(0.0, 0.0)])
    route.advance()
    assert route.exhausted()
    route.reset()
    assert route.current().x == 0.0


def test_load_waypoints(tmp_path):
    path = tmp_path / 'wp.yaml'
    path.write_text(
        'waypoints:\n'
        '  - {x: 1.0, y: 2.0}\n'
        '  - {x: 3.0, y: 4.0, yaw: 1.57, scan: false}\n')
    waypoints = load_waypoints(str(path))
    assert len(waypoints) == 2
    assert waypoints[0].x == 1.0 and waypoints[0].scan is True
    assert waypoints[1].yaw == 1.57 and waypoints[1].scan is False
