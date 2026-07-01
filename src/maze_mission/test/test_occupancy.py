"""Tests de las primitivas de grilla vendorizadas + inflado."""
import numpy as np

from maze_mission.occupancy import (
    GridSpec,
    grid_to_world,
    inflate_occupancy,
    is_cell_free,
    nearest_free_cell,
    world_to_grid,
)


def test_world_grid_roundtrip():
    spec = GridSpec(0.1, -1.0, -2.0)
    gx, gy = world_to_grid(0.0, 0.0, spec)
    x, y = grid_to_world(gx, gy, spec)
    assert abs(x - 0.05) < 0.1 and abs(y - 0.05) < 0.1


def test_is_cell_free_y_desconocido():
    grid = np.array([[0, 100], [-1, 0]], dtype=np.int16)
    assert is_cell_free(grid, (0, 0))
    assert not is_cell_free(grid, (1, 0))                 # lethal
    assert not is_cell_free(grid, (0, 1))                 # desconocido, allow_unknown False
    assert is_cell_free(grid, (0, 1), allow_unknown=True)
    assert not is_cell_free(grid, (5, 5))                 # fuera de bounds


def test_nearest_free():
    grid = np.zeros((3, 3), dtype=np.int16)
    grid[1, 1] = 100
    assert nearest_free_cell(grid, (1, 1), max_radius=3) is not None
    full = np.full((3, 3), 100, dtype=np.int16)
    assert nearest_free_cell(full, (1, 1), max_radius=3) is None


def test_inflate_bloquea_alrededor_de_obstaculo():
    grid = np.zeros((5, 5), dtype=np.int16)
    grid[2, 2] = 100
    inflated = inflate_occupancy(grid, radius_cells=1)
    assert inflated[2, 1] == 100 and inflated[1, 2] == 100 and inflated[2, 3] == 100
    assert inflated[0, 0] == 0     # esquina lejana sigue libre


def test_inflate_trata_desconocido_como_obstaculo():
    grid = np.zeros((5, 5), dtype=np.int16)
    grid[0, 0] = -1
    inflated = inflate_occupancy(grid, radius_cells=1)
    assert inflated[0, 1] == 100 or inflated[1, 0] == 100
    assert inflated[0, 0] == -1    # el desconocido queda -1 (sigue no navegable)
