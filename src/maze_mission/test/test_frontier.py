import numpy as np

from maze_mission.frontier import find_frontier_cells, cluster_frontiers


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
