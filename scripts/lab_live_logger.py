#!/usr/bin/env python3
"""Logger liviano para corridas reales de laboratorio.

Graba eventos estructurados y CSVs faciles de postprocesar mientras el rosbag
captura los topicos completos. No depende de mensajes custom: las detecciones de
cono y /nav_debug viajan como std_msgs/String con JSON.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def atomic_json_dump(path: Path, data: Dict[str, Any]) -> None:
    tmp = path.with_suffix(path.suffix + '.tmp')
    with tmp.open('w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write('\n')
    tmp.replace(path)


class CsvSink:
    def __init__(self, path: Path, fields: Iterable[str]):
        self.path = path
        self.file = path.open('w', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.file, fieldnames=list(fields))
        self.writer.writeheader()

    def write(self, row: Dict[str, Any]) -> None:
        self.writer.writerow(row)
        self.file.flush()

    def close(self) -> None:
        self.file.close()


class LabLiveLogger(Node):
    def __init__(self, out_dir: Path, ns: str, map_yaml: str):
        super().__init__('lab_live_logger')
        self.out_dir = out_dir
        self.ns = ns.strip('/')
        self.map_yaml = map_yaml
        self.events_path = out_dir / 'events.jsonl'
        self.events_file = self.events_path.open('w', encoding='utf-8')

        self.poses = CsvSink(out_dir / 'poses.csv', [
            't_wall', 't_msg', 'x', 'y', 'yaw', 'cov_xx', 'cov_yy', 'cov_yaw_yaw',
        ])
        self.goals = CsvSink(out_dir / 'goals.csv', [
            't_wall', 't_msg', 'x', 'y', 'yaw', 'frame_id',
        ])
        self.states = CsvSink(out_dir / 'states.csv', [
            't_wall', 'source', 'state', 'pose_x', 'pose_y', 'pose_yaw',
        ])
        self.cmd_vel = CsvSink(out_dir / 'cmd_vel.csv', [
            't_wall', 'topic', 'linear_x', 'linear_y', 'angular_z',
        ])
        self.cones = CsvSink(out_dir / 'cone_detections.csv', [
            't_wall', 't_msg', 'count', 'best_bearing_rad', 'best_area_px',
            'best_confidence', 'best_u', 'best_v', 'pose_x', 'pose_y', 'pose_yaw',
        ])
        self.cloud = CsvSink(out_dir / 'particlecloud.csv', [
            't_wall', 'count', 'mean_x', 'mean_y', 'spread_xy', 'spread_yaw',
        ])
        self.paths = CsvSink(out_dir / 'path_updates.csv', [
            't_wall', 't_msg', 'poses', 'length_m', 'start_x', 'start_y', 'end_x', 'end_y',
        ])

        self.summary_path = out_dir / 'summary.json'
        self.latest_path_path = out_dir / 'latest_path.csv'
        self.last_pose: Optional[tuple[float, float, float]] = None
        self.last_nav_state: Optional[str] = None
        self.last_mission_state: Optional[str] = None
        self.last_cloud = None
        self.start_wall = self.now_wall()
        self.counts: Dict[str, int] = {
            'poses': 0,
            'goals': 0,
            'path_updates': 0,
            'nav_state_changes': 0,
            'mission_state_changes': 0,
            'cone_messages': 0,
            'cone_messages_with_detection': 0,
            'cmd_vel': 0,
            'nav_debug': 0,
            'map_messages': 0,
            'particlecloud': 0,
        }
        self.distance_m = 0.0
        self.recovery_events = 0
        self.idle_events = 0
        self.map_info: Dict[str, Any] = {}
        self.last_nav_debug: Optional[Dict[str, Any]] = None

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 20)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(NavPath, '/plan', self.on_plan, 10)
        self.create_subscription(String, '/nav_state', self.on_nav_state, 20)
        self.create_subscription(String, '/mission_state', self.on_mission_state, 20)
        self.create_subscription(String, '/cone_detections', self.on_cone_detections, 20)
        self.create_subscription(String, '/nav_debug', self.on_nav_debug, 20)
        self.create_subscription(PoseArray, '/particlecloud', self.on_particlecloud, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(Twist, '/cmd_vel', self.make_cmd_cb('/cmd_vel'), 20)
        if self.ns:
            topic = f'/{self.ns}/cmd_vel'
            self.create_subscription(Twist, topic, self.make_cmd_cb(topic), 20)

        self.create_timer(5.0, self.write_summary)
        self.write_event('logger_started', {'out_dir': str(out_dir), 'ns': self.ns,
                                            'map_yaml': self.map_yaml})

    def now_wall(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def pose_snapshot(self) -> Dict[str, Optional[float]]:
        if self.last_pose is None:
            return {'pose_x': None, 'pose_y': None, 'pose_yaw': None}
        return {'pose_x': self.last_pose[0], 'pose_y': self.last_pose[1],
                'pose_yaw': self.last_pose[2]}

    def write_event(self, event: str, data: Dict[str, Any]) -> None:
        row = {'t_wall': self.now_wall(), 'event': event, **data}
        self.events_file.write(json.dumps(row, sort_keys=True) + '\n')
        self.events_file.flush()

    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation)
        t_msg = stamp_to_float(msg.header.stamp)
        x, y = float(p.position.x), float(p.position.y)
        if self.last_pose is not None:
            self.distance_m += math.hypot(x - self.last_pose[0], y - self.last_pose[1])
        self.last_pose = (x, y, yaw)
        cov = list(msg.pose.covariance)
        self.counts['poses'] += 1
        self.poses.write({
            't_wall': self.now_wall(), 't_msg': t_msg, 'x': x, 'y': y, 'yaw': yaw,
            'cov_xx': cov[0] if len(cov) > 0 else '',
            'cov_yy': cov[7] if len(cov) > 7 else '',
            'cov_yaw_yaw': cov[35] if len(cov) > 35 else '',
        })

    def on_goal(self, msg: PoseStamped) -> None:
        p = msg.pose
        yaw = yaw_from_quat(p.orientation)
        row = {
            't_wall': self.now_wall(),
            't_msg': stamp_to_float(msg.header.stamp),
            'x': float(p.position.x),
            'y': float(p.position.y),
            'yaw': yaw,
            'frame_id': msg.header.frame_id,
        }
        self.counts['goals'] += 1
        self.goals.write(row)
        self.write_event('goal_pose', row)

    def on_plan(self, msg: NavPath) -> None:
        pts = [(float(ps.pose.position.x), float(ps.pose.position.y)) for ps in msg.poses]
        length = 0.0
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            length += math.hypot(x1 - x0, y1 - y0)
        start = pts[0] if pts else ('', '')
        end = pts[-1] if pts else ('', '')
        row = {
            't_wall': self.now_wall(),
            't_msg': stamp_to_float(msg.header.stamp),
            'poses': len(pts),
            'length_m': length,
            'start_x': start[0],
            'start_y': start[1],
            'end_x': end[0],
            'end_y': end[1],
        }
        self.counts['path_updates'] += 1
        self.paths.write(row)
        with self.latest_path_path.open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['i', 'x', 'y'])
            for i, (x, y) in enumerate(pts):
                writer.writerow([i, x, y])
        self.write_event('plan_update', row)

    def write_state(self, source: str, state: str) -> None:
        pose = self.pose_snapshot()
        self.states.write({'t_wall': self.now_wall(), 'source': source, 'state': state, **pose})
        self.write_event(f'{source}_state', {'state': state, **pose})

    def on_nav_state(self, msg: String) -> None:
        state = msg.data
        if state != self.last_nav_state:
            self.last_nav_state = state
            self.counts['nav_state_changes'] += 1
            if state == 'RECOVERY':
                self.recovery_events += 1
            if state == 'IDLE':
                self.idle_events += 1
            self.write_state('nav', state)

    def on_mission_state(self, msg: String) -> None:
        state = msg.data
        if state != self.last_mission_state:
            self.last_mission_state = state
            self.counts['mission_state_changes'] += 1
            self.write_state('mission', state)

    def make_cmd_cb(self, topic: str):
        def _cb(msg: Twist) -> None:
            self.counts['cmd_vel'] += 1
            self.cmd_vel.write({
                't_wall': self.now_wall(),
                'topic': topic,
                'linear_x': float(msg.linear.x),
                'linear_y': float(msg.linear.y),
                'angular_z': float(msg.angular.z),
            })
        return _cb

    def on_cone_detections(self, msg: String) -> None:
        self.counts['cone_messages'] += 1
        count = 0
        best = {}
        t_msg = ''
        try:
            raw = json.loads(msg.data)
            dets = raw.get('detections', [])
            count = len(dets)
            t_msg = raw.get('stamp_s', '')
            if dets:
                best = max(dets, key=lambda d: (float(d.get('confidence', 0.0)),
                                                float(d.get('area_px', 0.0))))
                self.counts['cone_messages_with_detection'] += 1
        except Exception as exc:  # noqa: BLE001
            self.write_event('cone_parse_error', {'error': str(exc), 'raw': msg.data[:200]})
        pose = self.pose_snapshot()
        row = {
            't_wall': self.now_wall(),
            't_msg': t_msg,
            'count': count,
            'best_bearing_rad': best.get('bearing_rad', ''),
            'best_area_px': best.get('area_px', ''),
            'best_confidence': best.get('confidence', ''),
            'best_u': best.get('u', ''),
            'best_v': best.get('v', ''),
            **pose,
        }
        self.cones.write(row)
        if count:
            self.write_event('cone_detection', row)

    def on_nav_debug(self, msg: String) -> None:
        self.counts['nav_debug'] += 1
        try:
            self.last_nav_debug = json.loads(msg.data)
        except Exception:
            self.last_nav_debug = {'raw': msg.data}
        self.write_event('nav_debug', self.last_nav_debug)

    def on_particlecloud(self, msg: PoseArray) -> None:
        poses = msg.poses
        self.counts['particlecloud'] += 1
        if not poses:
            return
        xs = [float(p.position.x) for p in poses]
        ys = [float(p.position.y) for p in poses]
        yaws = [yaw_from_quat(p.orientation) for p in poses]
        mx = sum(xs) / len(xs)
        my = sum(ys) / len(ys)
        spread_xy = math.sqrt(sum((x - mx) ** 2 + (y - my) ** 2
                                  for x, y in zip(xs, ys)) / len(xs))
        s = sum(math.sin(a) for a in yaws) / len(yaws)
        c = sum(math.cos(a) for a in yaws) / len(yaws)
        spread_yaw = math.sqrt(max(0.0, -2.0 * math.log(max(1e-9, math.hypot(s, c)))))
        self.last_cloud = {'count': len(poses), 'mean_x': mx, 'mean_y': my,
                           'spread_xy': spread_xy, 'spread_yaw': spread_yaw}
        self.cloud.write({'t_wall': self.now_wall(), **self.last_cloud})

    def on_map(self, msg: OccupancyGrid) -> None:
        self.counts['map_messages'] += 1
        self.map_info = {
            'frame_id': msg.header.frame_id,
            'width': int(msg.info.width),
            'height': int(msg.info.height),
            'resolution': float(msg.info.resolution),
            'origin_x': float(msg.info.origin.position.x),
            'origin_y': float(msg.info.origin.position.y),
            'map_yaml': self.map_yaml,
        }
        self.write_event('map_received', self.map_info)

    def build_summary(self) -> Dict[str, Any]:
        return {
            'out_dir': str(self.out_dir),
            'ns': self.ns,
            'map_yaml': self.map_yaml,
            'start_wall': self.start_wall,
            'end_wall': self.now_wall(),
            'duration_s': max(0.0, self.now_wall() - self.start_wall),
            'counts': self.counts,
            'distance_m': self.distance_m,
            'last_pose': self.last_pose,
            'last_nav_state': self.last_nav_state,
            'last_mission_state': self.last_mission_state,
            'last_particlecloud': self.last_cloud,
            'recovery_events': self.recovery_events,
            'idle_events': self.idle_events,
            'map_info': self.map_info,
            'last_nav_debug': self.last_nav_debug,
        }

    def write_summary(self) -> None:
        atomic_json_dump(self.summary_path, self.build_summary())

    def close(self) -> None:
        self.write_event('logger_stopped', {})
        self.write_summary()
        for sink in (self.poses, self.goals, self.states, self.cmd_vel, self.cones,
                     self.cloud, self.paths):
            sink.close()
        self.events_file.close()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', required=True, type=Path)
    parser.add_argument('--ns', default='tb4_0',
                        help='namespace del robot real, por ejemplo tb4_0')
    parser.add_argument('--map-yaml', default='maps/laberinto_lab_20260702.yaml')
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = LabLiveLogger(args.out_dir, args.ns, args.map_yaml)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
