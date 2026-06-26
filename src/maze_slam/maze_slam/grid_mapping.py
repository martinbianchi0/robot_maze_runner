"""Pure occupancy-grid mapping utilities for the known-pose mapper."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Sequence, Tuple

import numpy as np


GridCell = Tuple[int, int]


@dataclass(frozen=True)
class MappingStats:
    """Small update summary for debug logs and tests."""

    beams_used: int = 0
    hit_updates: int = 0
    free_updates: int = 0


def probability_to_logodds(probability: float) -> float:
    """Convert a probability in (0, 1) to log-odds."""
    if probability <= 0.0 or probability >= 1.0:
        raise ValueError("probability must be in the open interval (0, 1)")
    return math.log(probability / (1.0 - probability))


def logodds_to_probability(log_odds: np.ndarray) -> np.ndarray:
    """Convert log-odds values to probabilities."""
    return 1.0 / (1.0 + np.exp(-log_odds))


def world_to_grid(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> GridCell:
    """Convert world coordinates to integer grid indices."""
    return (
        int(math.floor((x - origin_x) / resolution)),
        int(math.floor((y - origin_y) / resolution)),
    )


def grid_to_world(
    ix: int,
    iy: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> Tuple[float, float]:
    """Return the world coordinates of a grid cell center."""
    return (
        origin_x + (float(ix) + 0.5) * resolution,
        origin_y + (float(iy) + 0.5) * resolution,
    )


def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[GridCell]:
    """Return all cells on a discrete line, including both endpoints."""
    cells: List[GridCell] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    x = x0
    y = y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            return cells
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


def logodds_to_occupancy_grid_data(
    log_odds: np.ndarray,
    occupied_probability_threshold: float = 0.65,
    free_probability_threshold: float = 0.35,
) -> np.ndarray:
    """Convert a log-odds grid to ROS OccupancyGrid data values."""
    probabilities = logodds_to_probability(log_odds)
    data = np.full(log_odds.shape, -1, dtype=np.int8)
    data[probabilities >= occupied_probability_threshold] = 100
    data[probabilities <= free_probability_threshold] = 0
    return data


class OccupancyGridMapper:
    """Incremental log-odds occupancy grid updated from known poses."""

    def __init__(
        self,
        width_m: float = 12.0,
        height_m: float = 12.0,
        resolution: float = 0.05,
        origin_x: float = -6.0,
        origin_y: float = -6.0,
        occupied_probability: float = 0.70,
        free_probability: float = 0.40,
        min_log_odds: float = -5.0,
        max_log_odds: float = 5.0,
    ) -> None:
        if resolution <= 0.0:
            raise ValueError("resolution must be positive")
        if width_m <= 0.0 or height_m <= 0.0:
            raise ValueError("map dimensions must be positive")
        if min_log_odds >= max_log_odds:
            raise ValueError("min_log_odds must be lower than max_log_odds")

        self.width_m = float(width_m)
        self.height_m = float(height_m)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.width = int(round(width_m / resolution))
        self.height = int(round(height_m / resolution))
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.occupied_update = probability_to_logodds(occupied_probability)
        self.free_update = probability_to_logodds(free_probability)
        self.min_log_odds = float(min_log_odds)
        self.max_log_odds = float(max_log_odds)

    def in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.width and 0 <= iy < self.height

    def world_to_grid(self, x: float, y: float) -> GridCell:
        return world_to_grid(x, y, self.origin_x, self.origin_y, self.resolution)

    def grid_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        return grid_to_world(ix, iy, self.origin_x, self.origin_y, self.resolution)

    def update_scan(
        self,
        pose: Tuple[float, float, float],
        ranges: Sequence[float],
        local_cosines: np.ndarray,
        local_sines: np.ndarray,
        range_min: float,
        range_max: float,
        max_usable_range: float,
        ray_stride: int = 2,
        hit_range_margin: float = 0.02,
    ) -> MappingStats:
        """Update the grid using a LaserScan and a known robot pose.

        Finite ranges past max_usable_range are treated as free-space rays.
        NaN, +/-inf, below-min, and above-sensor-max readings are ignored.
        """
        if len(ranges) != len(local_cosines) or len(ranges) != len(local_sines):
            raise ValueError("ranges and precomputed angle arrays must match")

        start_ix, start_iy = self.world_to_grid(pose[0], pose[1])
        if not self.in_bounds(start_ix, start_iy):
            return MappingStats()

        stride = max(1, int(ray_stride))
        usable_range = float(max_usable_range) if max_usable_range > 0.0 else float(range_max)
        usable_range = min(usable_range, float(range_max))
        if usable_range <= 0.0:
            return MappingStats()

        indices = np.arange(0, len(ranges), stride, dtype=np.int32)
        selected_ranges = np.asarray(ranges, dtype=np.float32)[indices]
        finite = np.isfinite(selected_ranges)
        within_sensor_limits = (selected_ranges >= range_min) & (selected_ranges <= range_max)
        valid = finite & within_sensor_limits
        if not np.any(valid):
            return MappingStats()

        valid_indices = indices[valid]
        valid_ranges = selected_ranges[valid].astype(np.float32, copy=False)
        distances = np.minimum(valid_ranges, usable_range)
        hit_mask = (
            (valid_ranges < (usable_range - hit_range_margin))
            & (valid_ranges < (float(range_max) - hit_range_margin))
        )

        yaw = pose[2]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        local_x = distances * local_cosines[valid_indices]
        local_y = distances * local_sines[valid_indices]
        world_x = pose[0] + (cos_yaw * local_x) - (sin_yaw * local_y)
        world_y = pose[1] + (sin_yaw * local_x) + (cos_yaw * local_y)

        end_ix = np.floor((world_x - self.origin_x) / self.resolution).astype(np.int32)
        end_iy = np.floor((world_y - self.origin_y) / self.resolution).astype(np.int32)

        free_updates = 0
        hit_updates = 0
        for ix, iy, is_hit in zip(end_ix.tolist(), end_iy.tolist(), hit_mask.tolist()):
            cells = bresenham(start_ix, start_iy, ix, iy)
            endpoint_is_occupied = bool(is_hit) and self.in_bounds(ix, iy)
            free_cells = cells[:-1] if endpoint_is_occupied else cells

            for cx, cy in free_cells:
                if self.in_bounds(cx, cy):
                    self.log_odds[cy, cx] += self.free_update
                    free_updates += 1

            if endpoint_is_occupied:
                self.log_odds[iy, ix] += self.occupied_update
                hit_updates += 1

        np.clip(self.log_odds, self.min_log_odds, self.max_log_odds, out=self.log_odds)
        return MappingStats(
            beams_used=int(valid_ranges.size),
            hit_updates=hit_updates,
            free_updates=free_updates,
        )

    def to_occupancy_grid_data(
        self,
        occupied_probability_threshold: float = 0.65,
        free_probability_threshold: float = 0.35,
    ) -> np.ndarray:
        return logodds_to_occupancy_grid_data(
            self.log_odds,
            occupied_probability_threshold=occupied_probability_threshold,
            free_probability_threshold=free_probability_threshold,
        )
