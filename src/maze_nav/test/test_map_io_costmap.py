from pathlib import Path

import numpy as np

from maze_nav.costmap import CostmapConfig, inflate_obstacles, obstacle_mask
from maze_nav.map_io import image_to_occupancy, load_map_yaml


def test_image_to_occupancy_uses_ros_map_values_and_flips_y():
    image = np.array([
        [0, 205, 254],
        [254, 0, 205],
    ], dtype=np.uint8)

    grid = image_to_occupancy(image)

    assert grid.tolist() == [
        [0, 100, -1],
        [100, -1, 0],
    ]


def test_load_map_yaml_reads_relative_pgm(tmp_path):
    pgm = tmp_path / 'tiny.pgm'
    pgm.write_bytes(b'P5\n2 1\n255\n' + bytes([254, 0]))
    yaml = tmp_path / 'tiny.yaml'
    yaml.write_text(
        'image: tiny.pgm\n'
        'resolution: 0.05\n'
        'origin: [-1.0, -2.0, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n',
        encoding='utf-8',
    )

    grid, info = load_map_yaml(Path(yaml))

    assert grid.tolist() == [[0, 100]]
    assert info.width == 2
    assert info.height == 1
    assert info.resolution == 0.05
    assert info.origin_x == -1.0
    assert info.origin_y == -2.0


def test_obstacle_mask_can_treat_unknown_as_obstacle():
    grid = np.array([[0, -1, 100]], dtype=np.int8)

    assert obstacle_mask(grid, unknown_as_obstacle=True).tolist() == [[False, True, True]]
    assert obstacle_mask(grid, unknown_as_obstacle=False).tolist() == [[False, False, True]]


def test_inflate_obstacles_grows_by_metric_radius():
    grid = np.zeros((7, 7), dtype=np.int8)
    grid[3, 3] = 100

    costmap = inflate_obstacles(
        grid,
        resolution=0.10,
        config=CostmapConfig(inflation_radius_m=0.15, unknown_as_obstacle=True),
    )

    assert costmap[3, 3] == 100
    assert costmap[3, 4] == 100
    assert costmap[4, 4] == 100
    assert costmap[0, 0] == 0


def test_parte_a_map_artifacts_reference_existing_images():
    repo_root = Path(__file__).resolve().parents[3]
    for yaml_path in sorted((repo_root / 'results' / 'parte_a').glob('*.yaml')):
        grid, info = load_map_yaml(yaml_path)

        assert grid.shape == (info.height, info.width)
        assert info.resolution == 0.05
        assert info.origin_x == -8.0
        assert info.origin_y == -8.0
        assert np.count_nonzero(grid == 0) > 1000
        assert np.count_nonzero(grid == 100) > 100
