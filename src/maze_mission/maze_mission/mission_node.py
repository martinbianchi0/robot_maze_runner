"""Nodo de la maquina de estados de mision (Parte C).

Coordina percepcion (maze_perception) y navegacion (maze_nav, pila de toma-2)
para encontrar y alcanzar el unico cono rojo del laberinto. Es una capa de
SUPERVISION: no hace control de bajo nivel. Envia goals validados por /goal_pose
y observa el estado de nav por /nav_state; el navigator resuelve A*, pure-pursuit,
evasion y recovery.

Invariante de seguridad: el UNICO metodo que publica /goal_pose es _emit_goal(),
que valida contra el mapa navegable (/map inflado) antes de publicar. Ademas, para
un goal de cono se rechaza de entrada si el punto estimado cae sobre un obstaculo
del mapa CRUDO (el LIDAR golpeo una pared -> el cono esta DETRAS): no se snapea a
la cara de la pared, se descarta y se sigue buscando. Ninguna deteccion visual
produce un goal que atraviese una pared.

Interfaz de la pila de toma-2 (ver docs/decisiones/INTERFAZ_MAZE_NAV.md):
  - pose:   /amcl_pose (PoseWithCovarianceStamped) de la MCL.
  - mapa:   /map (OccupancyGrid latched) de map_publisher; se guarda crudo + inflado.
  - scan:   LaserScan (para estimar el rango del cono por LIDAR-fusion).
  - estado: /nav_state (String): IDLE, PLANNING, FOLLOWING, ALIGNING, REACHED,
            RECOVERY. Exito de navegacion = REACHED.
"""
from __future__ import annotations

import math
from enum import Enum

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from maze_perception.detections import ConeDetections
from maze_mission.cone_goal_estimator import cone_world_from_lidar
from maze_mission.goal_validator import GoalStatus, ValidatorConfig, validate_goal
from maze_mission.mission_config import MissionConfig
from maze_mission.occupancy import GridSpec, in_bounds, inflate_occupancy, world_to_grid
from maze_mission.search_waypoints import WaypointRoute, load_waypoints


class MissionState(Enum):
    INIT = 'INIT'
    LOAD_MAP = 'LOAD_MAP'
    LOCALIZE = 'LOCALIZE'
    SEARCH_CONE = 'SEARCH_CONE'
    CONE_DETECTED = 'CONE_DETECTED'
    ESTIMATE_CONE_GOAL = 'ESTIMATE_CONE_GOAL'
    PLAN_TO_CONE = 'PLAN_TO_CONE'
    NAVIGATE_TO_CONE = 'NAVIGATE_TO_CONE'
    AVOID_OBSTACLE = 'AVOID_OBSTACLE'
    REPLAN = 'REPLAN'
    VERIFY_CONE = 'VERIFY_CONE'
    DONE = 'DONE'
    FAILURE = 'FAILURE'


# Estados del navigator de toma-2 en /nav_state.
NAV_REACHED = 'REACHED'
NAV_RECOVERY = 'RECOVERY'
NAV_IDLE = 'IDLE'
NAV_MOVING = ('FOLLOWING', 'PLANNING', 'ALIGNING')
LETHAL = 50


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def grid_from_occupancy(msg: OccupancyGrid):
    grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    spec = GridSpec(msg.info.resolution,
                    msg.info.origin.position.x, msg.info.origin.position.y)
    return grid, spec


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')
        self.cfg = self._declare_config()
        self.validator_cfg = self._declare_validator_config()

        self.raw_grid = None        # mapa crudo (para detectar cono-tras-pared)
        self.grid = None            # mapa inflado (para validar navegabilidad)
        self.spec = None
        self.pose = None            # (x, y, yaw) en frame map
        self.scan = None            # LaserScan
        self.last_detections = None
        self.nav_state = None

        self.state = MissionState.INIT
        self.state_since = self.get_clock().now()
        self.route = self._load_route()
        self.chosen = None          # ConeDetection en curso
        self.stable_count = 0
        self.cone_goal = None       # (x, y) estimado del cono
        self.replan_count = 0
        self.wp_sent = False
        self._last_note = ''

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        sensor = QoSProfile(depth=10)
        sensor.reliability = ReliabilityPolicy.BEST_EFFORT

        self.goal_pub = self.create_publisher(PoseStamped, self.cfg.goal_pose_topic, 10)
        self.state_pub = self.create_publisher(String, self.cfg.mission_state_topic, 10)

        self.create_subscription(OccupancyGrid, self.cfg.map_topic, self._on_map, latched)
        self.create_subscription(String, self.cfg.detections_topic, self._on_detections, 10)
        self.create_subscription(String, self.cfg.nav_state_topic, self._on_nav_state, 10)
        self.create_subscription(LaserScan, self.cfg.scan_topic, self._on_scan, sensor)
        self._subscribe_pose(sensor)

        self.timer = self.create_timer(1.0 / max(1.0, self.cfg.control_hz), self._on_timer)
        self.get_logger().info(
            f'mission_node iniciado. pose={self.cfg.pose_topic} map={self.cfg.map_topic} '
            f'scan={self.cfg.scan_topic} waypoints={len(self.route)} '
            f'lidar_offset={math.degrees(self.cfg.lidar_yaw_offset):.0f}deg')

    # -- parametros ----------------------------------------------------------
    def _declare_config(self) -> MissionConfig:
        values = {n: self.declare_parameter(n, d).value
                  for n, d in MissionConfig.field_defaults().items()}
        return MissionConfig.from_dict(values)

    def _declare_validator_config(self) -> ValidatorConfig:
        return ValidatorConfig(
            lethal_threshold=self.declare_parameter('validator.lethal_threshold', 50).value,
            allow_unknown=self.declare_parameter('validator.allow_unknown', False).value,
            snap_radius_cells=self.declare_parameter('validator.snap_radius_cells', 6).value,
            max_snap_dist_m=self.declare_parameter('validator.max_snap_dist_m', 0.25).value,
        )

    def _load_route(self) -> WaypointRoute:
        if self.cfg.waypoints_file:
            try:
                return WaypointRoute(load_waypoints(self.cfg.waypoints_file))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'no pude cargar waypoints: {exc}')
        return WaypointRoute([])

    # -- callbacks -----------------------------------------------------------
    def _subscribe_pose(self, qos):
        kind = self.cfg.pose_topic_type
        if kind == 'odometry':
            self.create_subscription(Odometry, self.cfg.pose_topic, self._on_odom, qos)
        elif kind == 'pose_stamped':
            self.create_subscription(PoseStamped, self.cfg.pose_topic, self._on_pose, qos)
        else:
            self.create_subscription(
                PoseWithCovarianceStamped, self.cfg.pose_topic, self._on_pose_cov, qos)

    def _on_map(self, msg: OccupancyGrid):
        try:
            raw, spec = grid_from_occupancy(msg)
            radius = int(round(self.cfg.inflation_radius_m / max(spec.resolution, 1e-6)))
            self.raw_grid = raw
            self.grid = inflate_occupancy(raw, radius)
            self.spec = spec
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'mapa invalido: {exc}')

    def _on_detections(self, msg: String):
        try:
            self.last_detections = ConeDetections.from_json(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'detecciones invalidas: {exc}')

    def _on_nav_state(self, msg: String):
        self.nav_state = msg.data

    def _on_scan(self, msg: LaserScan):
        self.scan = msg

    def _on_pose(self, msg: PoseStamped):
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw_from_quat(msg.pose.orientation))

    def _on_pose_cov(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    # -- invariante de emision de goal --------------------------------------
    def _emit_goal(self, x, y, yaw=0.0) -> bool:
        """UNICO publicador de /goal_pose. Valida contra el mapa inflado antes de emitir."""
        if self.grid is None or self.spec is None:
            return False
        result = validate_goal(x, y, self.grid, self.spec, self.validator_cfg)
        if result.status == GoalStatus.REJECTED:
            self._note(f'goal ({x:.2f},{y:.2f}) RECHAZADO: {result.reason}')
            return False
        msg = PoseStamped()
        msg.header.frame_id = self.cfg.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(result.x)
        msg.pose.position.y = float(result.y)
        _, _, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.goal_pub.publish(msg)
        self._note(f'goal emitido ({result.x:.2f},{result.y:.2f}) [{result.status.value}]')
        return True

    # -- helpers -------------------------------------------------------------
    def _note(self, text):
        if text != self._last_note:
            self.get_logger().info(text)
            self._last_note = text

    def _elapsed(self):
        return (self.get_clock().now() - self.state_since).nanoseconds * 1e-9

    def _to(self, new_state):
        if new_state != self.state:
            self.state = new_state
            self.state_since = self.get_clock().now()

    def _current_cone(self):
        if self.last_detections is None:
            return None
        best = self.last_detections.best()
        if best is None or best.confidence < self.cfg.detection_confidence_min:
            return None
        return best

    def _cone_on_obstacle(self, x, y) -> bool:
        """True si el punto cae sobre un obstaculo del mapa CRUDO (cono tras pared)."""
        if self.raw_grid is None or self.spec is None:
            return False
        cell = world_to_grid(x, y, self.spec)
        if not in_bounds(self.raw_grid, cell):
            return True
        val = int(self.raw_grid[cell[1], cell[0]])
        return val >= LETHAL or val < 0

    def _estimate_cone(self, det):
        if self.scan is None or self.pose is None:
            return None
        return cone_world_from_lidar(
            det.bearing_rad, self.pose, list(self.scan.ranges),
            self.scan.angle_min, self.scan.angle_increment,
            lidar_yaw_offset=self.cfg.lidar_yaw_offset,
            lidar_offset_x=self.cfg.lidar_offset_x,
            sector_halfwidth=self.cfg.lidar_sector_halfwidth,
            range_mode=self.cfg.cone_range_mode)

    def _standoff(self, cone_xy):
        cx, cy = cone_xy
        px, py, _ = self.pose
        d = math.hypot(cx - px, cy - py)
        if d <= self.cfg.cone_standoff_m or d < 1e-3:
            return px, py
        t = (d - self.cfg.cone_standoff_m) / d
        return px + (cx - px) * t, py + (cy - py) * t

    # -- maquina de estados --------------------------------------------------
    def _on_timer(self):
        self.state_pub.publish(String(data=self.state.value))
        handler = getattr(self, f'_state_{self.state.name.lower()}', None)
        if handler is not None:
            handler()

    def _state_init(self):
        if self.grid is not None and self.pose is not None:
            self._to(MissionState.LOAD_MAP)

    def _state_load_map(self):
        if self.grid is not None:
            self._to(MissionState.LOCALIZE)

    def _state_localize(self):
        # TODO(real): criterio de convergencia de la MCL (varianza de /particlecloud).
        if self.pose is not None:
            self._note('localizacion disponible')
            self._to(MissionState.SEARCH_CONE)

    def _state_search_cone(self):
        det = self._current_cone()
        if det is not None:
            self.stable_count += 1
            if self.stable_count >= self.cfg.detection_stable_frames:
                self.chosen = det
                self._to(MissionState.CONE_DETECTED)
            return
        self.stable_count = 0
        # Gracia inicial: dar tiempo a que lleguen detecciones antes de salir a
        # recorrer waypoints (evita un detour espurio si el primer tick corre
        # antes de la primera deteccion; las detecciones no son latched).
        if not self.wp_sent and self._elapsed() < 1.5:
            return
        # recorrido de waypoints (TODO real: giro-scan en cada uno)
        wp = self.route.current()
        if wp is None:
            self._note('waypoints agotados sin encontrar el cono')
            self._to(MissionState.FAILURE)
            return
        if not self.wp_sent:
            if self._emit_goal(wp.x, wp.y, wp.yaw):
                self.wp_sent = True
        elif self.nav_state == NAV_REACHED:
            self.route.advance()
            self.wp_sent = False

    def _state_cone_detected(self):
        det = self._current_cone()
        if det is None:
            self.stable_count = 0
            self._to(MissionState.SEARCH_CONE)
            return
        self.chosen = det
        self._to(MissionState.ESTIMATE_CONE_GOAL)

    def _state_estimate_cone_goal(self):
        if self.scan is None or self.pose is None:
            return
        est = self._estimate_cone(self.chosen)
        if est is None:
            self._note('sin rango LIDAR al cono (fallback servoing pendiente); sigo buscando')
            self._to(MissionState.SEARCH_CONE)
            return
        self.cone_goal = (est[0], est[1])
        self._to(MissionState.PLAN_TO_CONE)

    def _state_plan_to_cone(self):
        cx, cy = self.cone_goal
        if self._cone_on_obstacle(cx, cy):
            self._note(f'cono en ({cx:.2f},{cy:.2f}) DETRAS DE PARED (obstaculo mapeado): rechazado')
            self.stable_count = 0
            self._to(MissionState.SEARCH_CONE)
            return
        gx, gy = self._standoff((cx, cy))
        if self._emit_goal(gx, gy):
            self.replan_count = 0
            self._to(MissionState.NAVIGATE_TO_CONE)
        else:
            self.stable_count = 0
            self._to(MissionState.SEARCH_CONE)

    def _state_navigate_to_cone(self):
        if self.nav_state == NAV_REACHED:
            self._to(MissionState.VERIFY_CONE)
        elif self.nav_state == NAV_RECOVERY and self._elapsed() > self.cfg.blocked_timeout_s:
            self._to(MissionState.AVOID_OBSTACLE)
        elif self.nav_state == NAV_IDLE and self._elapsed() > 2.0:
            self._to(MissionState.REPLAN)
        elif self._elapsed() > self.cfg.goal_timeout_s:
            self._to(MissionState.REPLAN)

    def _state_avoid_obstacle(self):
        if self.nav_state in NAV_MOVING:
            self._to(MissionState.NAVIGATE_TO_CONE)
        elif self._elapsed() > self.cfg.blocked_timeout_s:
            self._to(MissionState.REPLAN)

    def _state_replan(self):
        self.replan_count += 1
        if self.replan_count > self.cfg.max_replans:
            self._note('cono inalcanzable tras reintentos: FAILURE')
            self._to(MissionState.FAILURE)
            return
        det = self._current_cone()
        est = self._estimate_cone(det) if det is not None else None
        if est is not None and not self._cone_on_obstacle(est[0], est[1]):
            gx, gy = self._standoff((est[0], est[1]))
            if self._emit_goal(gx, gy):
                self._to(MissionState.NAVIGATE_TO_CONE)
                return
        self._to(MissionState.SEARCH_CONE)

    def _state_verify_cone(self):
        det = self._current_cone()
        if det is not None and det.area_px >= self.cfg.verify_area_px_min:
            self._to(MissionState.DONE)
        elif self._elapsed() > self.cfg.verify_timeout_s:
            self._note('no se confirmo el cono de cerca; sigo buscando')
            self._to(MissionState.SEARCH_CONE)

    def _state_done(self):
        self._note('MISION COMPLETA: cono rojo alcanzado y verificado')

    def _state_failure(self):
        self._note('MISION FALLIDA: aborto seguro (no se emiten mas goals)')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
