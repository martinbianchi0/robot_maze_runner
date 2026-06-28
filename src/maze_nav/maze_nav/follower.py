"""Seguimiento de path conservador con estados explicitos."""

import math
from dataclasses import dataclass


STATE_IDLE = 'IDLE'
STATE_ROTATE_TO_PATH = 'ROTATE_TO_PATH'
STATE_TRACK_PATH = 'TRACK_PATH'
STATE_BLOCKED_STOP = 'BLOCKED_STOP'
STATE_STUCK_RECOVERY = 'STUCK_RECOVERY'
STATE_GOAL_REACHED = 'GOAL_REACHED'
STATE_WATCHDOG_STOP = 'WATCHDOG_STOP'
STATE_SAFE_DRIVE = 'SAFE_DRIVE'


@dataclass(frozen=True)
class FollowerConfig:
    lookahead_m: float = 0.20
    max_linear_mps: float = 0.06
    max_angular_rps: float = 0.30
    min_tracking_linear_mps: float = 0.025
    min_approach_linear_mps: float = 0.024
    goal_tolerance_m: float = 0.10
    blocked_goal_tolerance_m: float = 0.12
    yaw_tolerance_rad: float = 0.35
    obstacle_stop_distance_m: float = 0.30
    obstacle_resume_distance_m: float = 0.36
    emergency_stop_distance_m: float = 0.16
    watchdog_timeout_s: float = 0.80
    heading_enter_rotate_rad: float = math.radians(70.0)
    heading_exit_rotate_rad: float = math.radians(35.0)
    avoidance_rotate_override_rad: float = math.radians(70.0)
    max_motion_heading_rad: float = math.radians(62.0)
    heading_k: float = 1.45
    final_yaw_k: float = 1.00
    slow_radius_m: float = 0.35
    waypoint_pass_radius_m: float = 0.09
    align_final_yaw: bool = False
    max_rotate_ticks: int = 300
    recovery_ticks: int = 18
    recovery_linear_mps: float = 0.0
    recovery_angular_scale: float = 0.55


@dataclass(frozen=True)
class FollowerCommand:
    linear: float
    angular: float
    state: str
    target_index: int = 0
    heading_error_rad: float = 0.0


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class PathFollower:
    """Controlador stateful de path para robot diferencial."""

    def __init__(self, config=None):
        self.config = config or FollowerConfig()
        self.state = STATE_IDLE
        self._last_rotate_sign = 0
        self._rotate_ticks = 0
        self._progress_index = 0
        self._recovery_ticks = 0
        self._recovery_sign = 1

    def reset(self):
        self.state = STATE_IDLE
        self._last_rotate_sign = 0
        self._rotate_ticks = 0
        self._progress_index = 0
        self._recovery_ticks = 0
        self._recovery_sign = 1

    def compute(
        self,
        pose,
        path_xy,
        goal_yaw=None,
        front_clearance_m=math.inf,
        front_emergency_m=None,
        avoidance_turn=0,
        pose_age_s=0.0,
        scan_age_s=0.0,
    ):
        cfg = self.config
        if pose_age_s > cfg.watchdog_timeout_s or scan_age_s > cfg.watchdog_timeout_s:
            self.state = STATE_WATCHDOG_STOP
            return FollowerCommand(0.0, 0.0, self.state)
        if not path_xy:
            self.reset()
            return FollowerCommand(0.0, 0.0, STATE_IDLE)

        x, y, yaw = pose
        robot_xy = (x, y)
        goal_xy = path_xy[-1]
        dist_goal = _dist(robot_xy, goal_xy)

        if dist_goal <= cfg.goal_tolerance_m:
            if goal_yaw is None or not cfg.align_final_yaw:
                self.state = STATE_GOAL_REACHED
                return FollowerCommand(0.0, 0.0, self.state, len(path_xy) - 1)
            yaw_err = wrap_angle(goal_yaw - yaw)
            if abs(yaw_err) <= cfg.yaw_tolerance_rad:
                self.state = STATE_GOAL_REACHED
                return FollowerCommand(0.0, 0.0, self.state, len(path_xy) - 1, yaw_err)
            angular = clamp(
                cfg.final_yaw_k * yaw_err,
                -cfg.max_angular_rps,
                cfg.max_angular_rps,
            )
            self.state = STATE_ROTATE_TO_PATH
            return FollowerCommand(0.0, angular, self.state, len(path_xy) - 1, yaw_err)

        self._advance_progress(robot_xy, path_xy)

        target_i = self._progress_index
        for i in range(self._progress_index, len(path_xy)):
            target_i = i
            if _dist(robot_xy, path_xy[i]) >= cfg.lookahead_m:
                break
        target = path_xy[target_i]

        target_angle = math.atan2(target[1] - y, target[0] - x)
        heading_error = wrap_angle(target_angle - yaw)
        abs_error = abs(heading_error)

        if front_emergency_m is None:
            front_emergency_m = front_clearance_m

        should_rotate = abs_error > cfg.heading_enter_rotate_rad
        if self.state == STATE_ROTATE_TO_PATH and abs_error > cfg.heading_exit_rotate_rad:
            should_rotate = True
        should_rotate_while_blocked = should_rotate or abs_error > cfg.yaw_tolerance_rad

        confirmed_emergency = (
            front_emergency_m < cfg.emergency_stop_distance_m
            and front_clearance_m < cfg.obstacle_resume_distance_m
        )
        front_limited = front_clearance_m < cfg.obstacle_stop_distance_m
        if (
            dist_goal <= max(cfg.goal_tolerance_m, cfg.blocked_goal_tolerance_m)
            and (confirmed_emergency or front_limited)
        ):
            self.state = STATE_GOAL_REACHED
            return FollowerCommand(0.0, 0.0, self.state, len(path_xy) - 1, heading_error)

        if confirmed_emergency:
            if should_rotate_while_blocked:
                return self._rotate_to_path_command(heading_error, target_i)
            self.state = STATE_BLOCKED_STOP
            return FollowerCommand(0.0, 0.0, self.state, target_i, heading_error)

        if front_limited:
            if should_rotate_while_blocked:
                return self._rotate_to_path_command(heading_error, target_i)
            return self._recovery_command(heading_error, target_i, avoidance_turn)

        if self.state == STATE_STUCK_RECOVERY and self._recovery_ticks > 0:
            if front_clearance_m < cfg.obstacle_resume_distance_m:
                return self._recovery_command(heading_error, target_i, avoidance_turn)
            self._recovery_ticks -= 1
            self.state = STATE_STUCK_RECOVERY
            angular = self._recovery_sign * cfg.max_angular_rps * cfg.recovery_angular_scale
            return FollowerCommand(
                min(cfg.recovery_linear_mps, cfg.max_linear_mps * 0.35),
                angular,
                self.state,
                target_i,
                heading_error,
            )

        if should_rotate:
            return self._rotate_to_path_command(heading_error, target_i)

        self._last_rotate_sign = 0
        self._rotate_ticks = 0
        self.state = STATE_TRACK_PATH

        if abs_error >= cfg.max_motion_heading_rad:
            linear = 0.0
        else:
            heading_scale = max(0.0, math.cos(abs_error)) ** 1.5
            goal_scale = clamp(
                (dist_goal - cfg.goal_tolerance_m) /
                max(1e-6, cfg.slow_radius_m - cfg.goal_tolerance_m),
                0.0,
                1.0,
            )
            linear = cfg.max_linear_mps * heading_scale * goal_scale
            if dist_goal > cfg.slow_radius_m and abs_error < math.radians(25.0):
                linear = max(linear, cfg.min_tracking_linear_mps)
            elif dist_goal > cfg.goal_tolerance_m and abs_error < math.radians(20.0):
                linear = max(linear, cfg.min_approach_linear_mps)
        angular = clamp(
            cfg.heading_k * heading_error,
            -cfg.max_angular_rps,
            cfg.max_angular_rps,
        )
        return FollowerCommand(linear, angular, self.state, target_i, heading_error)

    def _rotate_to_path_command(self, heading_error, target_i):
        cfg = self.config
        if self.state == STATE_ROTATE_TO_PATH and self._last_rotate_sign:
            sign = self._last_rotate_sign
        else:
            sign = 1 if heading_error > 0 else -1
        self._last_rotate_sign = sign
        self._rotate_ticks += 1
        if self._rotate_ticks > cfg.max_rotate_ticks:
            self.state = STATE_STUCK_RECOVERY
            return FollowerCommand(0.0, 0.0, self.state, target_i, heading_error)
        angular = sign * clamp(
            cfg.heading_k * abs(heading_error),
            0.0,
            cfg.max_angular_rps,
        )
        self.state = STATE_ROTATE_TO_PATH
        return FollowerCommand(0.0, angular, self.state, target_i, heading_error)

    def _recovery_command(self, heading_error, target_i, avoidance_turn):
        cfg = self.config
        if avoidance_turn:
            sign = 1 if avoidance_turn > 0 else -1
        elif self._recovery_sign:
            sign = self._recovery_sign
        else:
            sign = 1 if heading_error >= 0.0 else -1
        self._recovery_sign = sign
        self._recovery_ticks = cfg.recovery_ticks
        self._rotate_ticks = 0
        self.state = STATE_STUCK_RECOVERY
        return FollowerCommand(
            0.0,
            sign * cfg.max_angular_rps * cfg.recovery_angular_scale,
            self.state,
            target_i,
            heading_error,
        )

    def _advance_progress(self, robot_xy, path_xy):
        cfg = self.config
        if self._progress_index >= len(path_xy):
            self._progress_index = max(0, len(path_xy) - 1)

        pass_radius = max(cfg.waypoint_pass_radius_m, cfg.lookahead_m * 0.45)
        while self._progress_index < len(path_xy) - 1:
            current = path_xy[self._progress_index]
            nxt = path_xy[self._progress_index + 1]
            seg_x = nxt[0] - current[0]
            seg_y = nxt[1] - current[1]
            seg_len_sq = seg_x * seg_x + seg_y * seg_y
            robot_x = robot_xy[0] - current[0]
            robot_y = robot_xy[1] - current[1]
            projection = 0.0
            if seg_len_sq > 1e-9:
                projection = (robot_x * seg_x + robot_y * seg_y) / seg_len_sq

            if _dist(robot_xy, current) <= pass_radius or projection > 1.05:
                self._progress_index += 1
                continue
            break
