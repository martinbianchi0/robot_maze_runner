import math

from maze_mission.localization import cloud_spread, is_converged


def test_cluster_apretado_converge():
    xs = [1.0, 1.02, 0.98, 1.01]
    ys = [2.0, 2.01, 1.99, 2.0]
    yaws = [0.5, 0.52, 0.48, 0.5]
    sxy, syaw = cloud_spread(xs, ys, yaws)
    assert sxy < 0.05
    assert syaw < 0.05
    assert is_converged(sxy, syaw, xy_max=0.25, yaw_max=0.4)


def test_nube_dispersa_no_converge():
    xs = [0.0, 2.0, -2.0, 4.0]
    ys = [0.0, 2.0, 3.0, -1.0]
    yaws = [0.1, 0.1, 0.1, 0.1]
    sxy, syaw = cloud_spread(xs, ys, yaws)
    assert sxy > 1.0
    assert not is_converged(sxy, syaw, xy_max=0.25, yaw_max=0.4)


def test_yaw_bimodal_no_converge():
    # xy apretado pero yaw repartido entre 0 y pi (mapa simetrico): la media
    # circular no engancha y el spread de yaw debe ser grande.
    n = 10
    xs = [1.0] * n
    ys = [1.0] * n
    yaws = [0.0, math.pi] * (n // 2)
    sxy, syaw = cloud_spread(xs, ys, yaws)
    assert sxy < 0.01
    assert syaw > 1.0
    assert not is_converged(sxy, syaw, xy_max=0.25, yaw_max=0.4)


def test_yaw_envuelto_converge():
    # nube alrededor de +/-pi (borde de la envoltura): debe converger igual
    yaws = [math.pi - 0.02, -math.pi + 0.02, math.pi - 0.01, -math.pi + 0.03]
    sxy, syaw = cloud_spread([0.0] * 4, [0.0] * 4, yaws)
    assert syaw < 0.1


def test_nube_vacia_es_infinito():
    sxy, syaw = cloud_spread([], [], [])
    assert math.isinf(sxy) and math.isinf(syaw)
    assert not is_converged(sxy, syaw, xy_max=10.0, yaw_max=10.0)
