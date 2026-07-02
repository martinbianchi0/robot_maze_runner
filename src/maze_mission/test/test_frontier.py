import numpy as np

from maze_mission.frontier import find_frontier_cells


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
