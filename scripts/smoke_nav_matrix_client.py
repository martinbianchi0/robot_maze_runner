#!/usr/bin/env python3
"""Cliente ROS para clasificar un caso de navegacion en custom_casa."""

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String


OK_FOR_PARTE_B = {
    'B_OK_REACHED',
    'B_OK_BLOCKED_SAFE',
    'A_MAP_OR_COSTMAP_ISSUE',
    'TF_POSE_ISSUE',
    'SCAN_SENSOR_ISSUE',
}


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def parse_goal(text):
    parts = [float(part.strip()) for part in text.split(',')]
    if len(parts) == 2:
        parts.append(0.0)
    if len(parts) != 3:
        raise argparse.ArgumentTypeError('goal debe ser x,y o x,y,yaw_deg')
    return (parts[0], parts[1], math.radians(parts[2]))


def pose_xy_from_odom(msg):
    p = msg.pose.pose.position
    return (float(p.x), float(p.y))


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def path_points(msg):
    return [
        (float(pose.pose.position.x), float(pose.pose.position.y))
        for pose in msg.poses
    ]


def path_length_m(points):
    if len(points) < 2:
        return 0.0
    return sum(distance(a, b) for a, b in zip(points, points[1:]))


def world_to_grid(x, y, grid_info):
    eps = 1e-9
    gx = int(math.floor((x - grid_info['origin_x']) / grid_info['resolution'] + eps))
    gy = int(math.floor((y - grid_info['origin_y']) / grid_info['resolution'] + eps))
    return gx, gy


def bresenham_cells(a, b):
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    cells = []
    x = x0
    y = y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells


def path_valid_in_costmap(points, grid_info, data, lethal_threshold=50):
    if not points:
        return False, 'empty_path'
    w = grid_info['width']
    h = grid_info['height']

    def cell_free(cell):
        gx, gy = cell
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return False, 'out_of_bounds'
        value = int(data[gy * w + gx])
        if value < 0:
            return False, 'unknown_cell'
        if value >= lethal_threshold:
            return False, f'occupied_cell:{value}'
        return True, ''

    cells = [world_to_grid(x, y, grid_info) for x, y in points]
    for cell in cells:
        ok, reason = cell_free(cell)
        if not ok:
            return False, reason
    for a, b in zip(cells, cells[1:]):
        for cell in bresenham_cells(a, b):
            ok, reason = cell_free(cell)
            if not ok:
                return False, reason
    return True, ''


def finite_or_none(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return value


class MatrixCaseClient:
    def __init__(self, node):
        self.node = node
        self.odom_xy = None
        self.state = None
        self.state_since = time.monotonic()
        self.latest_debug = None
        self.latest_cmd = None
        self.costmap_info = None
        self.costmap_data = None
        self.path = []
        self.path_updates = 0
        self.path_invalid_updates = 0
        self.path_valid = False
        self.path_invalid_reason = 'no_path'
        self.first_plan_latency_s = None
        self.goal_publish_time = None
        self.min_front_emergency_m = math.inf
        self.min_front_clearance_m = math.inf
        self.max_scan_overlay_cells = 0
        self.max_abs_linear = 0.0
        self.max_abs_angular = 0.0

        self.goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
        node.create_subscription(Odometry, '/odom', self._on_odom, 10)
        node.create_subscription(String, '/nav_state', self._on_state, 10)
        node.create_subscription(String, '/nav_debug', self._on_debug, 10)
        node.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        node.create_subscription(Path, '/planned_path', self._on_path, 10)
        node.create_subscription(OccupancyGrid, '/global_costmap', self._on_costmap, 10)

    def _on_odom(self, msg):
        self.odom_xy = pose_xy_from_odom(msg)

    def _on_state(self, msg):
        if msg.data != self.state:
            self.state = msg.data
            self.state_since = time.monotonic()

    def _on_debug(self, msg):
        try:
            self.latest_debug = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_debug = {'raw': msg.data}
        front_emergency = finite_or_none(self.latest_debug.get('front_emergency_m'))
        front_clearance = finite_or_none(self.latest_debug.get('front_clearance_m'))
        if front_emergency is not None:
            self.min_front_emergency_m = min(self.min_front_emergency_m, front_emergency)
        if front_clearance is not None:
            self.min_front_clearance_m = min(self.min_front_clearance_m, front_clearance)
        self.max_scan_overlay_cells = max(
            self.max_scan_overlay_cells,
            int(self.latest_debug.get('scan_overlay_cells') or 0),
        )

    def _on_cmd(self, msg):
        self.latest_cmd = (float(msg.linear.x), float(msg.angular.z))
        self.max_abs_linear = max(self.max_abs_linear, abs(self.latest_cmd[0]))
        self.max_abs_angular = max(self.max_abs_angular, abs(self.latest_cmd[1]))

    def _on_costmap(self, msg):
        self.costmap_info = {
            'width': int(msg.info.width),
            'height': int(msg.info.height),
            'resolution': float(msg.info.resolution),
            'origin_x': float(msg.info.origin.position.x),
            'origin_y': float(msg.info.origin.position.y),
        }
        self.costmap_data = list(msg.data)
        self._refresh_path_validity()

    def _on_path(self, msg):
        points = path_points(msg)
        if points != self.path:
            self.path_updates += 1
            if self.goal_publish_time is not None and self.first_plan_latency_s is None:
                self.first_plan_latency_s = time.monotonic() - self.goal_publish_time
        self.path = points
        self._refresh_path_validity()
        if not self.path_valid:
            self.path_invalid_updates += 1

    def _refresh_path_validity(self):
        if self.costmap_info is None or self.costmap_data is None:
            self.path_valid = False
            self.path_invalid_reason = 'no_costmap'
            return
        self.path_valid, self.path_invalid_reason = path_valid_in_costmap(
            self.path,
            self.costmap_info,
            self.costmap_data,
        )

    def wait_until_ready(self, timeout_s):
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.odom_xy is not None and self.costmap_info is not None:
                return True
        return False

    def publish_goal(self, goal):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        qx, qy, qz, qw = yaw_to_quaternion(goal[2])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.goal_publish_time = time.monotonic()
        for _ in range(10):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.goal_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def settle_terminal_messages(self, duration_s=0.5):
        end_time = time.monotonic() + duration_s
        while time.monotonic() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def run_case(self, case_name, expected, goal, timeout_s, block_observe_s):
        if not self.wait_until_ready(timeout_s=30.0):
            return self._result(
                case_name,
                expected,
                goal,
                'NOT_READY',
                0.0,
                0.0,
                math.inf,
                'WATCHDOG_STOP',
            )

        start_xy = self.odom_xy
        goal_xy = (goal[0], goal[1])
        self.publish_goal(goal)
        start_time = time.monotonic()
        last_print = 0.0

        while time.monotonic() - start_time < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            now = time.monotonic()
            elapsed = now - start_time
            moved = distance(start_xy, self.odom_xy) if self.odom_xy else 0.0
            goal_error = distance(self.odom_xy, goal_xy) if self.odom_xy else math.inf

            if elapsed - last_print >= 5.0:
                last_print = elapsed
                print(
                    'matrix_progress case={} t={:.1f}s state={} moved={:.3f} '
                    'goal_error={:.3f} path_updates={} path_valid={} reason={}'.format(
                        case_name,
                        elapsed,
                        self.state,
                        moved,
                        goal_error,
                        self.path_updates,
                        self.path_valid,
                        self.latest_debug.get('reason') if self.latest_debug else None,
                    ),
                    flush=True,
                )

            if self.state == 'GOAL_REACHED':
                self.settle_terminal_messages()
                moved = distance(start_xy, self.odom_xy) if self.odom_xy else moved
                goal_error = distance(self.odom_xy, goal_xy) if self.odom_xy else goal_error
                return self._result(
                    case_name,
                    expected,
                    goal,
                    'GOAL_REACHED',
                    elapsed,
                    moved,
                    goal_error,
                    self.state,
                )

            if self.state in {'BLOCKED_STOP', 'WATCHDOG_STOP'}:
                if now - self.state_since >= block_observe_s:
                    self.settle_terminal_messages()
                    moved = distance(start_xy, self.odom_xy) if self.odom_xy else moved
                    goal_error = distance(self.odom_xy, goal_xy) if self.odom_xy else goal_error
                    return self._result(
                        case_name,
                        expected,
                        goal,
                        self.state,
                        elapsed,
                        moved,
                        goal_error,
                        self.state,
                    )

        moved = distance(start_xy, self.odom_xy) if self.odom_xy else 0.0
        goal_error = distance(self.odom_xy, goal_xy) if self.odom_xy else math.inf
        return self._result(
            case_name,
            expected,
            goal,
            'TIMEOUT',
            timeout_s,
            moved,
            goal_error,
            self.state,
        )

    def _result(self, case_name, expected, goal, result, elapsed, moved, goal_error, final_state):
        path_len_m = path_length_m(self.path)
        debug_reason = self.latest_debug.get('reason') if self.latest_debug else None
        debug_perf = self.latest_debug.get('perf') if self.latest_debug else None
        classification, diagnosis = classify_result(
            expected=expected,
            result=result,
            final_state=final_state,
            cmd=self.latest_cmd,
            debug=self.latest_debug,
            path_valid=self.path_valid,
            path_invalid_reason=self.path_invalid_reason,
            path_invalid_updates=self.path_invalid_updates,
            min_front_emergency_m=self.min_front_emergency_m,
            min_front_clearance_m=self.min_front_clearance_m,
            max_scan_overlay_cells=self.max_scan_overlay_cells,
            goal_error_m=goal_error,
        )
        return {
            'case': case_name,
            'goal': {
                'x': round(goal[0], 4),
                'y': round(goal[1], 4),
                'yaw_deg': round(math.degrees(goal[2]), 2),
            },
            'expected': expected,
            'result': result,
            'classification': classification,
            'gate_ok_for_parte_b': classification in OK_FOR_PARTE_B,
            'diagnosis': diagnosis,
            'final_state': final_state,
            'elapsed_s': round(float(elapsed), 2),
            'goal_error_m': round(float(goal_error), 3) if math.isfinite(goal_error) else None,
            'moved_m': round(float(moved), 3),
            'min_front_scan_m': (
                round(self.min_front_emergency_m, 3)
                if math.isfinite(self.min_front_emergency_m)
                else None
            ),
            'min_front_clearance_m': (
                round(self.min_front_clearance_m, 3)
                if math.isfinite(self.min_front_clearance_m)
                else None
            ),
            'last_cmd_vel': self.latest_cmd,
            'max_abs_cmd_vel': {
                'linear_x': round(self.max_abs_linear, 4),
                'angular_z': round(self.max_abs_angular, 4),
            },
            'nav_debug_reason': debug_reason,
            'nav_debug': self.latest_debug,
            'path_waypoints': len(self.path),
            'planned_length_m': round(path_len_m, 3),
            'path_updates': self.path_updates,
            'replans_observed': max(0, self.path_updates - 1),
            'path_valid_costmap': self.path_valid,
            'path_invalid_reason': self.path_invalid_reason if not self.path_valid else '',
            'path_invalid_updates': self.path_invalid_updates,
            'first_plan_latency_s': (
                round(self.first_plan_latency_s, 3)
                if self.first_plan_latency_s is not None
                else None
            ),
            'perf': debug_perf,
            'max_scan_overlay_cells': self.max_scan_overlay_cells,
        }


def classify_result(
    expected,
    result,
    final_state,
    cmd,
    debug,
    path_valid,
    path_invalid_reason,
    path_invalid_updates,
    min_front_emergency_m,
    min_front_clearance_m,
    max_scan_overlay_cells,
    goal_error_m,
):
    reason = debug.get('reason') if debug else None
    front_emergency = finite_or_none(debug.get('front_emergency_m')) if debug else None
    front_clearance = finite_or_none(debug.get('front_clearance_m')) if debug else None
    zero_cmd = cmd is not None and abs(cmd[0]) < 1e-4 and abs(cmd[1]) < 1e-4

    scan_saw_close = (
        (front_emergency is not None and front_emergency < 0.35)
        or (front_clearance is not None and front_clearance < 0.35)
        or (math.isfinite(min_front_emergency_m) and min_front_emergency_m < 0.25)
    )
    scan_saw_suspicious_speckle = (
        front_emergency is not None
        and front_clearance is not None
        and front_emergency < 0.25
        and front_clearance > 0.60
    )
    map_scan_mismatch = (
        path_valid
        and max_scan_overlay_cells > 0
        and (
            scan_saw_close
            or (front_clearance is not None and front_clearance < 0.55)
            or (math.isfinite(min_front_clearance_m) and min_front_clearance_m < 0.45)
        )
    )

    if result == 'GOAL_REACHED':
        if expected == 'blocked':
            return (
                'B_BUG_STATE_MACHINE',
                'El caso esperaba bloqueo, pero el sistema declaro llegada.',
            )
        return 'B_OK_REACHED', 'Llego al goal dentro de tolerancia.'

    if result == 'BLOCKED_STOP' and zero_cmd:
        if expected == 'blocked':
            return (
                'B_OK_BLOCKED_SAFE',
                'El caso era bloqueado y el robot freno con cmd_vel cero.',
            )
        if scan_saw_suspicious_speckle:
            return (
                'SCAN_SENSOR_ISSUE',
                'El scan reporto un frente minimo muy bajo pero la clearance agregada era alta.',
            )
        if map_scan_mismatch:
            return (
                'A_MAP_OR_COSTMAP_ISSUE',
                'El path/costmap parecian transitables, pero el LIDAR/overlay marco obstaculos cercanos.',
            )
        if path_invalid_updates > 0 and path_invalid_reason:
            return (
                'A_MAP_OR_COSTMAP_ISSUE',
                f'El path quedo invalidado por el costmap dinamico: {path_invalid_reason}.',
            )
        if reason in {'plan_failed'}:
            return (
                'A_MAP_OR_COSTMAP_ISSUE',
                'El planner no encontro camino en el costmap disponible.',
            )
        if reason in {'no_progress', 'goal_regression'} and path_valid:
            return (
                'B_BUG_FOLLOWER',
                'El path seguia valido y no habia evidencia fuerte de obstaculo frontal, pero se bloqueo por progreso.',
            )
        return (
            'B_TOO_CONSERVATIVE',
            'Freno seguro sin evidencia suficiente para atribuirlo a mapa/scan.',
        )

    if result == 'WATCHDOG_STOP':
        if reason in {'no_scan'}:
            return 'SCAN_SENSOR_ISSUE', 'No hubo scan fresco para navegar.'
        if reason in {'no_pose'}:
            return 'TF_POSE_ISSUE', 'No hubo pose fresca para navegar.'
        return 'B_BUG_STATE_MACHINE', 'Watchdog sin causa externa clara.'

    if not path_valid and path_invalid_updates > 0 and (
        scan_saw_close or max_scan_overlay_cells > 0
    ):
        return (
            'A_MAP_OR_COSTMAP_ISSUE',
            f'El path fue invalidado por el costmap dinamico/scan: {path_invalid_reason}.',
        )

    if not path_valid and path_invalid_updates > 0:
        return (
            'B_BUG_PLANNER',
            f'El path publicado cruza una celda invalida del costmap: {path_invalid_reason}.',
        )

    if final_state == 'TRACK_PATH' and cmd is not None and abs(cmd[0]) < 1e-4:
        return (
            'B_BUG_FOLLOWER',
            'El follower quedo en TRACK_PATH sin avance lineal suficiente.',
        )

    if result == 'TIMEOUT':
        if map_scan_mismatch:
            return (
                'A_MAP_OR_COSTMAP_ISSUE',
                'Timeout con evidencia de mismatch entre path/costmap y scan.',
            )
        if (
            goal_error_m <= 0.12
            and (scan_saw_close or max_scan_overlay_cells > 0)
        ):
            return (
                'A_MAP_OR_COSTMAP_ISSUE',
                'Quedo dentro de tolerancia blanda cerca del goal, con scan/overlay bloqueando el ultimo tramo.',
            )
        if path_valid and goal_error_m > 0.15:
            return (
                'B_BUG_FOLLOWER',
                'Timeout con path valido y sin clasificacion externa clara.',
            )
        return (
            'NEEDS_MANUAL_RVIZ_REVIEW',
            'La evidencia headless no alcanza para clasificar el timeout.',
        )

    return (
        'NEEDS_MANUAL_RVIZ_REVIEW',
        'Resultado no terminal o no clasificado por la matriz.',
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--case-name', required=True)
    parser.add_argument(
        '--expected',
        choices=['reached', 'blocked', 'diagnostic'],
        required=True,
    )
    parser.add_argument('--goal', type=parse_goal, required=True)
    parser.add_argument('--timeout-s', type=float, default=70.0)
    parser.add_argument('--block-observe-s', type=float, default=5.0)
    parser.add_argument('--output-json')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('maze_nav_matrix_client')
    client = MatrixCaseClient(node)
    try:
        result = client.run_case(
            args.case_name,
            args.expected,
            args.goal,
            args.timeout_s,
            args.block_observe_s,
        )
        text = json.dumps(result, sort_keys=True)
        print('matrix_result ' + text, flush=True)
        if args.output_json:
            with open(args.output_json, 'w', encoding='utf-8') as fh:
                fh.write(text + '\n')
        return 0 if result['gate_ok_for_parte_b'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
