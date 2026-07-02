import numpy as np

from maze_mission.frontier import find_frontier_cells, cluster_frontiers, path_cost_field, select_frontier_goal
from maze_mission.occupancy import GridSpec


def test_borde_libre_desconocido_es_frontera():
    # columna 0 libre, columna 1 desconocida -> la col 0 es frontera
    grid = np.array([[0, -1, -1],
                     [0, -1, -1],
                     [0, -1, -1]], dtype=np.int16)
    mask = find_frontier_cells(grid, lethal=50)
    assert mask[:, 0].all()          # toda la columna libre lindante con desconocido
    assert not mask[:, 2].any()      # desconocido no es frontera


def test_grilla_toda_conocida_sin_fronteras():
    grid = np.zeros((4, 4), dtype=np.int16)   # todo libre, nada desconocido
    mask = find_frontier_cells(grid, lethal=50)
    assert not mask.any()


def test_pared_no_es_frontera():
    grid = np.array([[100, -1],
                     [0, -1]], dtype=np.int16)
    mask = find_frontier_cells(grid, lethal=50)
    assert not mask[0, 0]            # pared (lethal) no es frontera aunque linde desconocido
    assert mask[1, 0]               # libre lindante con desconocido si


def test_dos_frentes_separados_dos_clusters():
    mask = np.zeros((7, 7), dtype=bool)
    mask[0, 0:3] = True          # frente A (3 celdas)
    mask[6, 4:7] = True          # frente B (3 celdas), no contiguo
    clusters = cluster_frontiers(mask, min_cells=1)
    assert len(clusters) == 2
    assert {c.size for c in clusters} == {3}


def test_descarta_clusters_chicos():
    mask = np.zeros((7, 7), dtype=bool)
    mask[0, 0:2] = True          # 2 celdas
    mask[6, 0:5] = True          # 5 celdas
    clusters = cluster_frontiers(mask, min_cells=5)
    assert len(clusters) == 1
    assert clusters[0].size == 5


def test_centroide_dentro_del_frente():
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 1:4] = True          # fila y=2, x=1..3 -> centroide (2,2)
    clusters = cluster_frontiers(mask, min_cells=1)
    assert len(clusters) == 1
    assert clusters[0].centroid_gx == 2 and clusters[0].centroid_gy == 2


def test_costo_respeta_paredes():
    # laberinto en L: pared en el medio obliga a rodear
    grid = np.zeros((5, 5), dtype=np.int16)
    grid[0:4, 2] = 100           # pared vertical col 2, filas 0..3 (deja fila 4 abierta)
    dist = path_cost_field(grid, (0, 0), lethal=50)
    # celda al otro lado de la pared: alcanzable pero con costo > distancia recta
    assert dist[0, 4] > 4        # tuvo que bajar y rodear por la fila 4
    assert dist[0, 0] == 0


def test_frontera_amurallada_inalcanzable():
    grid = np.zeros((5, 5), dtype=np.int16)
    grid[1, 1:4] = 100
    grid[3, 1:4] = 100
    grid[1:4, 1] = 100
    grid[1:4, 3] = 100           # celda (2,2) encerrada por paredes
    dist = path_cost_field(grid, (0, 0), lethal=50)
    assert dist[2, 2] == -1      # inalcanzable


def _spec():
    return GridSpec(resolution=1.0, origin_x=0.0, origin_y=0.0)


def test_sin_fronteras_devuelve_none():
    grid = np.zeros((5, 5), dtype=np.int16)   # todo conocido-libre
    out = select_frontier_goal(grid, _spec(), (0.0, 0.0),
                               inflation_cells=0, min_frontier_cells=1, alpha=0.1)
    assert out is None


def test_elige_frontera_por_utilidad():
    # dos frentes: uno chico y cercano, uno grande y lejano.
    grid = np.zeros((9, 9), dtype=np.int16)
    grid[:, 8] = -1              # columna desconocida a la derecha (frente grande, lejano)
    grid[0, 0] = -1             # una celda desconocida arriba-izq (frente chico, cercano)
    # con alpha alto (costo domina) elige el cercano; con alpha bajo (ganancia domina) el grande
    cerca = select_frontier_goal(grid, _spec(), (1.0, 1.0),
                                 inflation_cells=0, min_frontier_cells=1, alpha=10.0)
    lejos = select_frontier_goal(grid, _spec(), (1.0, 1.0),
                                 inflation_cells=0, min_frontier_cells=1, alpha=0.01)
    assert cerca is not None and lejos is not None
    # el frente grande esta en x~7 (col 7 libre lindante con col 8 desconocida)
    assert lejos.x > cerca.x


def test_goal_en_mundo_dentro_de_bounds():
    grid = np.zeros((5, 5), dtype=np.int16)
    grid[:, 4] = -1
    out = select_frontier_goal(grid, _spec(), (0.0, 0.0),
                               inflation_cells=0, min_frontier_cells=1, alpha=0.1)
    assert out is not None
    assert 0.0 <= out.x <= 5.0 and 0.0 <= out.y <= 5.0


def test_inflation_no_bloquea_todas_las_fronteras():
    # con inflacion > 0, las celdas frontera (que lindan con desconocido) no deben
    # quedar todas infladas como obstaculo: debe seguir habiendo un goal alcanzable.
    grid = np.zeros((7, 7), dtype=np.int16)
    grid[:, 6] = -1
    out = select_frontier_goal(grid, _spec(), (0.0, 3.0),
                               inflation_cells=1, min_frontier_cells=1, alpha=0.1)
    assert out is not None
