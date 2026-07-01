"""Nodo de la maquina de estados de mision (Parte C).

Coordina percepcion (maze_perception) y navegacion (maze_nav, pila de toma-2)
para encontrar y alcanzar el unico cono rojo del laberinto. Es una capa de
SUPERVISION: no hace control de bajo nivel. Envia goals validados por /goal_pose
y observa el estado de nav por /nav_state; el navigator resuelve A*, pure-pursuit,
evasion y recovery.

Invariante de seguridad: el UNICO metodo que publica /goal_pose es _emit_goal(),
que valida contra el mapa navegable (/map inflado) antes de publicar. Ninguna
deteccion visual puede producir un goal que atraviese una pared.

Interfaz de la pila de toma-2 (ver docs/decisiones/INTERFAZ_MAZE_NAV.md):
  - pose:   /amcl_pose (PoseWithCovarianceStamped) de la MCL (localizer).
  - mapa:   /map (OccupancyGrid latched) de map_publisher; se infla aca mismo.
  - estado: /nav_state (String): IDLE, PLANNING, FOLLOWING, ALIGNING, REACHED,
            RECOVERY. Exito de navegacion = REACHED. Goal inalcanzable => el
            navigator vuelve a IDLE sin pasar por REACHED.

Estado de implementacion: el andamiaje (M0) declara parametros, arma la config,
cablea pub/sub, ingiere e infla el mapa y aplica el invariante _emit_goal. La
logica completa de transiciones se completa en la etapa C5 (M3); los handlers
avanzados estan marcados como TODO(M3).
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
from std_msgs.msg import String

from maze_perception.detections import ConeDetections
from maze_mission.goal_validator import GoalStatus, ValidatorConfig, validate_goal
from maze_mission.mission_config import MissionConfig
from maze_mission.occupancy import GridSpec, inflate_occupancy
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


# Estados que publica el navigator de toma-2 en /nav_state (para M3).
NAV_REACHED = 'REACHED'
NAV_RECOVERY = 'RECOVERY'
NAV_IDLE = 'IDLE'


def quaternion_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def grid_from_occupancy(msg: OccupancyGrid):
    """OccupancyGrid crudo -> (grid numpy HxW int16, GridSpec)."""
    grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    spec = GridSpec(msg.info.resolution,
                    msg.info.origin.position.x, msg.info.origin.position.y)
    return grid, spec


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')
        self.cfg = self._declare_config()
        self.validator_cfg = self._declare_validator_config()

        self.grid = None            # mapa YA inflado (para validar goals)
        self.spec = None
        self.pose = None            # (x, y, yaw) en frame map
        self.last_detections = None
        self.nav_state = None
        self.state = MissionState.INIT
        self.route = self._load_route()
        self._last_note = ''

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        sensor = QoSProfile(depth=10)
        sensor.reliability = ReliabilityPolicy.BEST_EFFORT

        self.goal_pub = self.create_publisher(PoseStamped, self.cfg.goal_pose_topic, 10)
        self.state_pub = self.create_publisher(String, self.cfg.mission_state_topic, 10)

        self.create_subscription(OccupancyGrid, self.cfg.map_topic, self._on_map, latched)
        self.create_subscription(
            String, self.cfg.detections_topic, self._on_detections, 10)
        self.create_subscription(String, self.cfg.nav_state_topic, self._on_nav_state, 10)
        self._subscribe_pose(sensor)

        self.timer = self.create_timer(1.0 / max(1.0, self.cfg.control_hz), self._on_timer)
        self.get_logger().info(
            f'mission_node iniciado. pose={self.cfg.pose_topic} '
            f'map={self.cfg.map_topic} waypoints={len(self.route)} '
            f'lidar_offset={math.degrees(self.cfg.lidar_yaw_offset):.0f}deg '
            f'range_mode={self.cfg.cone_range_mode}')

    # -- declaracion de parametros ------------------------------------------
    def _declare_config(self) -> MissionConfig:
        values = {}
        for name, default in MissionConfig.field_defaults().items():
            values[name] = self.declare_parameter(name, default).value
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
        else:  # pose_with_covariance (default, MCL /amcl_pose)
            self.create_subscription(
                PoseWithCovarianceStamped, self.cfg.pose_topic, self._on_pose_cov, qos)

    def _on_map(self, msg: OccupancyGrid):
        try:
            raw, spec = grid_from_occupancy(msg)
            radius_cells = int(round(self.cfg.inflation_radius_m / max(spec.resolution, 1e-6)))
            self.grid = inflate_occupancy(raw, radius_cells)
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

    def _on_pose(self, msg: PoseStamped):
        self.pose = (msg.pose.position.x, msg.pose.position.y,
                     yaw_from_quat(msg.pose.orientation))

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
            self._note('sin mapa: no se emite goal')
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
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.goal_pub.publish(msg)
        self._note(f'goal emitido ({result.x:.2f},{result.y:.2f}) [{result.status.value}]')
        return True

    def _note(self, text: str):
        if text != self._last_note:
            self.get_logger().info(text)
            self._last_note = text

    # -- maquina de estados --------------------------------------------------
    def _on_timer(self):
        self.state_pub.publish(String(data=self.state.value))
        handler = getattr(self, f'_state_{self.state.name.lower()}', None)
        if handler is not None:
            handler()

    def _state_init(self):
        if self.grid is not None:
            self.state = MissionState.LOAD_MAP

    def _state_load_map(self):
        if self.grid is not None:
            self.state = MissionState.LOCALIZE

    def _state_localize(self):
        # TODO(M3): criterio real de convergencia de la MCL (varianza de
        # /particlecloud estable + coherencia scan<->map).
        if self.pose is not None:
            self._note('localizacion disponible (criterio real pendiente M3)')
            self.state = MissionState.SEARCH_CONE

    def _state_search_cone(self):
        # TODO(M3): emitir waypoints (via _emit_goal), giro-scan, y pasar a
        # CONE_DETECTED al ver rojo estable. NAVIGATE_TO_CONE: exito = nav_state
        # REACHED; bloqueo = RECOVERY sostenido; goal inalcanzable = vuelta a IDLE.
        self._note('SEARCH_CONE: recorrido de waypoints pendiente (M3)')

    # Estados CONE_DETECTED..VERIFY_CONE, DONE, FAILURE: TODO(M3).


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
