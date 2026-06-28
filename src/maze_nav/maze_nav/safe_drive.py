"""Control reactivo simple para recorrer sin teleop y sin depender del mapa."""

import math
from dataclasses import dataclass

import numpy as np

from maze_nav.follower import (
    STATE_BLOCKED_STOP,
    STATE_SAFE_DRIVE,
    STATE_STUCK_RECOVERY,
    STATE_WATCHDOG_STOP,
)


@dataclass(frozen=True)
class ScanSectors:
    # `front` es un clearance robusto: ignora outliers aislados del LaserScan.
    # `front_min` conserva el minimo crudo para freno de emergencia.
    front: float
    left: float
    right: float
    valid_count: int
    front_min: float = math.inf


@dataclass(frozen=True)
class SafeDriveConfig:
    max_linear_mps: float = 0.05
    max_angular_rps: float = 0.25
    front_clear_m: float = 0.65
    front_slow_m: float = 0.45
    front_stop_m: float = 0.32
    min_valid_ranges: int = 20
    watchdog_timeout_s: float = 0.80
    steer_k: float = 0.22


@dataclass(frozen=True)
class SafeDriveCommand:
    linear: float
    angular: float
    state: str
    sectors: ScanSectors


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def analyze_scan(ranges, angle_min, angle_increment, range_min, range_max):
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = angle_min + np.arange(ranges.size, dtype=np.float64) * angle_increment
    angles = np.arctan2(np.sin(angles), np.cos(angles))
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)

    def sector_values(deg_lo, deg_hi):
        mask = valid & (angles >= math.radians(deg_lo)) & (angles <= math.radians(deg_hi))
        if not np.any(mask):
            return np.asarray([], dtype=np.float64)
        return ranges[mask]

    def sector_min(deg_lo, deg_hi):
        values = sector_values(deg_lo, deg_hi)
        if values.size == 0:
            return math.inf
        return float(np.min(values))

    def sector_percentile(deg_lo, deg_hi, percentile):
        values = sector_values(deg_lo, deg_hi)
        if values.size == 0:
            return math.inf
        return float(np.percentile(values, percentile))

    # Un rayo aislado muy bajo aparece seguido en Gazebo/RViz como un "puntito"
    # blanco. Para freno de emergencia usamos un percentil bajo, no el minimo
    # crudo, asi seguimos frenando ante obstaculos reales sin quedar paralizados
    # por un outlier suelto.
    front_emergency = sector_percentile(-25.0, 25.0, 5.0)

    return ScanSectors(
        front=sector_percentile(-18.0, 18.0, 20.0),
        left=sector_min(30.0, 110.0),
        right=sector_min(-110.0, -30.0),
        valid_count=int(np.count_nonzero(valid)),
        front_min=front_emergency,
    )


class SafeDriveController:
    def __init__(self, config=None):
        self.config = config or SafeDriveConfig()

    def compute(self, sectors, scan_age_s=0.0):
        cfg = self.config
        if scan_age_s > cfg.watchdog_timeout_s or sectors.valid_count < cfg.min_valid_ranges:
            return SafeDriveCommand(0.0, 0.0, STATE_WATCHDOG_STOP, sectors)

        left = sectors.left if math.isfinite(sectors.left) else cfg.front_clear_m
        right = sectors.right if math.isfinite(sectors.right) else cfg.front_clear_m
        front = sectors.front

        if not math.isfinite(front):
            return SafeDriveCommand(0.0, 0.0, STATE_WATCHDOG_STOP, sectors)

        turn_dir = 1.0 if left >= right else -1.0
        side_balance = clamp(left - right, -1.0, 1.0)

        if front <= cfg.front_stop_m:
            if max(left, right) <= cfg.front_stop_m:
                return SafeDriveCommand(0.0, 0.0, STATE_BLOCKED_STOP, sectors)
            return SafeDriveCommand(
                0.0,
                turn_dir * cfg.max_angular_rps,
                STATE_STUCK_RECOVERY,
                sectors,
            )

        if front < cfg.front_slow_m:
            return SafeDriveCommand(
                cfg.max_linear_mps * 0.35,
                turn_dir * cfg.max_angular_rps * 0.75,
                STATE_STUCK_RECOVERY,
                sectors,
            )

        angular = clamp(cfg.steer_k * side_balance, -cfg.max_angular_rps, cfg.max_angular_rps)
        linear_scale = 1.0 if front >= cfg.front_clear_m else 0.60
        return SafeDriveCommand(
            cfg.max_linear_mps * linear_scale,
            angular,
            STATE_SAFE_DRIVE,
            sectors,
        )
