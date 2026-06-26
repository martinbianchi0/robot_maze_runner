import math

import numpy as np

from maze_slam.grid_mapping import (
    OccupancyGridMapper,
    bresenham,
    grid_to_world,
    logodds_to_occupancy_grid_data,
    probability_to_logodds,
    world_to_grid,
)


def test_world_grid_round_trip_uses_cell_center():
    origin_x = -6.0
    origin_y = -6.0
    resolution = 0.05
    cell = world_to_grid(0.0, 0.0, origin_x, origin_y, resolution)
    assert cell == (120, 120)

    x, y = grid_to_world(cell[0], cell[1], origin_x, origin_y, resolution)
    assert math.isclose(x, 0.025)
    assert math.isclose(y, 0.025)


def test_bresenham_includes_endpoints():
    assert bresenham(0, 0, 3, 0) == [(0, 0), (1, 0), (2, 0), (3, 0)]
    assert bresenham(0, 0, 0, 3) == [(0, 0), (0, 1), (0, 2), (0, 3)]
    assert bresenham(0, 0, 3, 3) == [(0, 0), (1, 1), (2, 2), (3, 3)]


def test_logodds_conversion_thresholds():
    log_odds = np.array(
        [
            [probability_to_logodds(0.20), 0.0, probability_to_logodds(0.80)],
        ],
        dtype=np.float32,
    )
    data = logodds_to_occupancy_grid_data(log_odds)
    assert data.tolist() == [[0, -1, 100]]


def test_mapper_updates_free_and_occupied_cells():
    mapper = OccupancyGridMapper(width_m=4.0, height_m=4.0, resolution=1.0, origin_x=-2.0, origin_y=-2.0)
    ranges = [1.0]
    local_cos = np.array([1.0], dtype=np.float32)
    local_sin = np.array([0.0], dtype=np.float32)

    stats = mapper.update_scan(
        pose=(0.0, 0.0, 0.0),
        ranges=ranges,
        local_cosines=local_cos,
        local_sines=local_sin,
        range_min=0.1,
        range_max=5.0,
        max_usable_range=5.0,
        ray_stride=1,
    )

    assert stats.beams_used == 1
    assert stats.hit_updates == 1
    assert mapper.log_odds[2, 2] < 0.0
    assert mapper.log_odds[2, 3] > 0.0


def test_mapper_filters_invalid_ranges():
    mapper = OccupancyGridMapper(width_m=4.0, height_m=4.0, resolution=1.0, origin_x=-2.0, origin_y=-2.0)
    ranges = [float("nan"), float("inf"), 0.01, 6.0]
    local_cos = np.ones(4, dtype=np.float32)
    local_sin = np.zeros(4, dtype=np.float32)

    stats = mapper.update_scan(
        pose=(0.0, 0.0, 0.0),
        ranges=ranges,
        local_cosines=local_cos,
        local_sines=local_sin,
        range_min=0.1,
        range_max=5.0,
        max_usable_range=5.0,
        ray_stride=1,
    )

    assert stats.beams_used == 0
    assert np.all(mapper.log_odds == 0.0)


def test_max_range_reading_marks_free_without_occupied_endpoint():
    mapper = OccupancyGridMapper(width_m=6.0, height_m=2.0, resolution=1.0, origin_x=-1.0, origin_y=-1.0)
    ranges = [5.0]
    local_cos = np.array([1.0], dtype=np.float32)
    local_sin = np.array([0.0], dtype=np.float32)

    stats = mapper.update_scan(
        pose=(0.0, 0.0, 0.0),
        ranges=ranges,
        local_cosines=local_cos,
        local_sines=local_sin,
        range_min=0.1,
        range_max=5.0,
        max_usable_range=5.0,
        ray_stride=1,
    )

    assert stats.beams_used == 1
    assert stats.hit_updates == 0
    assert np.count_nonzero(mapper.log_odds < 0.0) > 0
