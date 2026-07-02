"""Navegador autonomo (Parte B): planning A* + control pure-pursuit + FSM.

Consume la estimacion de pose de la localizacion MCL (/amcl_pose), el mapa
estatico (/map) y el objetivo (/goal_pose, herramienta "2D Goal Pose" de RViz).
Publica /cmd_vel para llevar el robot al objetivo, evitando paredes mapeadas y
obstaculos NO mapeados detectados con el LIDAR.

Maquina de estados:
    IDLE      -> sin objetivo; robot quieto
    PLANNING  -> corre A* desde la pose actual al objetivo
    FOLLOWING -> sigue el camino con pure-pursuit
    ALIGNING  -> llego a la posicion; gira al angulo final
    REACHED   -> objetivo cumplido; vuelve a IDLE
    RECOVERY  -> obstaculo bloqueando; re-planifica esquivandolo
"""
import heapq
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile, DurabilityPolicy,
                       ReliabilityPolicy)
from scipy.ndimage import distance_transform_edt

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from maze_nav.nav_utils import wrap_angle, world_to_grid, grid_to_world


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.declare_parameter('robot_radius', 0.14)
        self.declare_parameter('inflation', 0.12)      # margen extra sobre el radio
        self.declare_parameter('lookahead', 0.35)
        self.declare_parameter('v_max', 0.18)
        self.declare_parameter('w_max', 1.2)
        self.declare_parameter('goal_tol', 0.12)       # m
        self.declare_parameter('yaw_tol', 0.15)        # rad ~ 8.6 grados
        self.declare_parameter('safety_stop', 0.22)    # m, freno de emergencia
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('max_recovery_attempts', 3)
        self.declare_parameter('recovery_hold_s', 0.8)
        self.declare_parameter('recovery_backoff_s', 0.45)
        self.declare_parameter('recovery_backoff_speed', -0.05)
        self.declare_parameter('front_blocked_confirmations', 2)
        self.declare_parameter('front_obstacle_mark_radius', 0.12)
        # Montaje del LIDAR respecto a base: offset lineal (TB3 burger: base_scan
        # 3.2 cm atras; TB4 real: -0.04) y angular (TB3 sim: 0; TB4 real: el
        # RPLIDAR esta a +90 deg -> +pi/2). Ver INTERFAZ_MAZE_NAV.md.
        self.declare_parameter('scan_x_offset', -0.032)
        self.declare_parameter('scan_yaw_offset', 0.0)

        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.inflation = float(self.get_parameter('inflation').value)
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.goal_tol = float(self.get_parameter('goal_tol').value)
        self.yaw_tol = float(self.get_parameter('yaw_tol').value)
        self.safety_stop = float(self.get_parameter('safety_stop').value)
        self.scan_dx = float(self.get_parameter('scan_x_offset').value)
        self.scan_dyaw = float(self.get_parameter('scan_yaw_offset').value)
        self.max_recovery_attempts = int(self.get_parameter('max_recovery_attempts').value)
        self.recovery_hold_s = float(self.get_parameter('recovery_hold_s').value)
        self.recovery_backoff_s = float(self.get_parameter('recovery_backoff_s').value)
        self.recovery_backoff_speed = float(self.get_parameter('recovery_backoff_speed').value)
        self.front_blocked_confirmations = int(
            self.get_parameter('front_blocked_confirmations').value)
        self.front_obstacle_mark_radius = float(
            self.get_parameter('front_obstacle_mark_radius').value)

        self.map = None                 # dict con occ,res,origin,H,W
        self.cost = None                # EDT (m) a obstaculo, para penalizar cercania
        self.blocked = None             # bool grid: no navegable (obstaculo inflado)
        self.dyn_obstacles = set()      # celdas de obstaculos no mapeados (gx,gy)
        self.pose = None                # (x,y,theta) estimada
        self.goal = None                # (x,y,theta)
        self.path = []                  # lista de (x,y) en mundo
        self.path_idx = 0               # avanza monotono a lo largo de self.path
        self.state = 'IDLE'
        self.scan = None
        self.last_forward_clearance = None
        self.last_dyn_added = 0
        self.last_cmd = (0.0, 0.0)
        self.last_debug_note = 'init'
        self.last_plan_cells = 0
        self.last_plan_length_m = 0.0
        self.recovery_attempts = 0
        self.recovery_started_s = None
        self.front_blocked_hits = 0
        self.blocked_reason = ''

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self.on_scan, qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.state_pub = self.create_publisher(String, '/nav_state', 10)
        self.debug_pub = self.create_publisher(String, '/nav_debug', 10)

        rate = float(self.get_parameter('control_rate').value)
        self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().info('Navigator listo: esperando /map, /amcl_pose y /goal_pose')

    # --------------------------------------------------------------------- #
    def on_map(self, msg):
        W, H = msg.info.width, msg.info.height
        res = msg.info.resolution
        origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        occ = np.array(msg.data, dtype=np.int8).reshape(H, W)
        self.map = {'occ': occ, 'res': res, 'origin': origin, 'H': H, 'W': W}
        self._build_costmap()
        self.get_logger().info(f'Costmap construido ({W}x{H} @ {res}m)')

    def _build_costmap(self):
        occ = self.map['occ']
        res = self.map['res']
        # paredes mapeadas + desconocido = obstaculo (no navegar por lo no visto)
        obst = (occ == 100) | (occ == -1)
        # obstaculos dinamicos (no mapeados)
        if self.dyn_obstacles:
            for (gx, gy) in self.dyn_obstacles:
                if 0 <= gy < self.map['H'] and 0 <= gx < self.map['W']:
                    obst[gy, gx] = True
        dist = distance_transform_edt(~obst).astype(np.float32) * res
        self.cost = dist
        # inflado: no se puede pasar a menos de (radio+margen) de un obstaculo
        self.blocked = dist < (self.robot_radius + self.inflation)

    def on_pose(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, quat_to_yaw(p.orientation))

    def on_goal(self, msg):
        yaw = quat_to_yaw(msg.pose.orientation)
        self.goal = (msg.pose.position.x, msg.pose.position.y, yaw)
        self.dyn_obstacles.clear()
        self.recovery_attempts = 0
        self.recovery_started_s = None
        self.front_blocked_hits = 0
        self.blocked_reason = ''
        self.get_logger().info(
            f'Nuevo objetivo: ({self.goal[0]:.2f}, {self.goal[1]:.2f}, {math.degrees(yaw):.0f}deg)')
        self.state = 'PLANNING'
        self.last_debug_note = 'goal_received'

    def on_scan(self, msg):
        self.scan = msg

    # --------------------------------------------------------------------- #
    #   Planning A*                                                          #
    # --------------------------------------------------------------------- #
    def _nearest_free(self, gx, gy, max_r=25):
        """Si la celda esta bloqueada, busca la celda navegable mas cercana."""
        H, W = self.map['H'], self.map['W']
        if 0 <= gx < W and 0 <= gy < H and not self.blocked[gy, gx]:
            return gx, gy
        for r in range(1, max_r):
            for dx in range(-r, r + 1):
                for dy in (-r, r):
                    x, y = gx + dx, gy + dy
                    if 0 <= x < W and 0 <= y < H and not self.blocked[y, x]:
                        return x, y
                for dyy in range(-r, r + 1):
                    for dxx in (-r, r):
                        x, y = gx + dxx, gy + dyy
                        if 0 <= x < W and 0 <= y < H and not self.blocked[y, x]:
                            return x, y
        return None

    def plan(self):
        if self.map is None or self.pose is None or self.goal is None:
            return False
        res = self.map['res']
        origin = self.map['origin']
        H, W = self.map['H'], self.map['W']
        s = self._nearest_free(*world_to_grid(self.pose[0], self.pose[1], origin, res))
        g = self._nearest_free(*world_to_grid(self.goal[0], self.goal[1], origin, res))
        if s is None or g is None:
            self.get_logger().warn('Start u objetivo sin celda navegable cercana')
            self.last_debug_note = 'plan_failed_no_nearest_free'
            return False

        blocked = self.blocked
        cost = self.cost
        # penalizacion por cercania a obstaculos (empuja el camino al centro)
        clear = self.robot_radius + self.inflation
        neighbors = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                     (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]

        def h(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        open_heap = [(h(s, g), 0.0, s)]
        came = {}
        gscore = {s: 0.0}
        found = False
        while open_heap:
            _, gc, cur = heapq.heappop(open_heap)
            if cur == g:
                found = True
                break
            if gc > gscore.get(cur, 1e18):
                continue
            cx, cy = cur
            for dx, dy, step in neighbors:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < W and 0 <= ny < H) or blocked[ny, nx]:
                    continue
                # empujar hacia zonas despejadas: penalizar proximidad a obstaculos
                prox = max(0.0, clear + 0.25 - cost[ny, nx])
                ng = gc + step + prox * 3.0
                if ng < gscore.get((nx, ny), 1e18):
                    gscore[(nx, ny)] = ng
                    came[(nx, ny)] = cur
                    heapq.heappush(open_heap, (ng + h((nx, ny), g), ng, (nx, ny)))
        if not found:
            self.get_logger().warn('A*: no se encontro camino al objetivo')
            self.last_debug_note = 'plan_failed_astar_no_path'
            return False

        # reconstruir y pasar a mundo
        cells = [g]
        while cells[-1] != s:
            cells.append(came[cells[-1]])
        cells.reverse()
        self.path = [grid_to_world(gx, gy, origin, res) for (gx, gy) in cells]
        self.path_idx = 0
        self.last_plan_cells = len(self.path)
        self.last_plan_length_m = sum(
            math.hypot(x1 - x0, y1 - y0)
            for (x0, y0), (x1, y1) in zip(self.path, self.path[1:])
        )
        self.last_debug_note = 'plan_ok'
        self._publish_path()
        return True

    def _publish_path(self):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in self.path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    # --------------------------------------------------------------------- #
    #   Deteccion de obstaculos no mapeados                                  #
    # --------------------------------------------------------------------- #
    def _forward_clearance(self):
        """Distancia libre minima en el arco frontal (m). None si no hay scan."""
        if self.scan is None:
            return None
        r = np.asarray(self.scan.ranges, dtype=np.float32)
        ang = self.scan.angle_min + np.arange(len(r)) * self.scan.angle_increment
        # "frente" en frame base = angulo del scan + montaje del LIDAR
        front = np.abs(wrap_angle_vec(ang + self.scan_dyaw)) < 0.5   # +/- ~30 grados
        rr = r[front]
        rr = rr[np.isfinite(rr) & (rr > self.scan.range_min)]
        return float(rr.min()) if len(rr) else None

    def _register_obstacle_from_scan(self):
        """Agrega al costmap los impactos del LIDAR que caen en celdas 'libres'
        del mapa (=> obstaculo no mapeado) y re-infla. Devuelve # celdas nuevas.

        Filtra impactos cercanos a paredes ya mapeadas (radio 2 celdas) para no
        marcar como 'nuevo' lo que en realidad es error de localizacion MCL.
        """
        if self.scan is None or self.pose is None:
            return 0
        r = np.asarray(self.scan.ranges, dtype=np.float32)
        ang = self.scan.angle_min + np.arange(len(r)) * self.scan.angle_increment
        ok = np.isfinite(r) & (r > self.scan.range_min) & (r < 2.0)
        r, ang = r[ok], ang[ok]
        px, py, pth = self.pose
        # Aplicar el montaje del LIDAR: offset lineal + angular (params).
        c, s = math.cos(pth), math.sin(pth)
        sx = px + c * self.scan_dx
        sy = py + s * self.scan_dx
        ex = sx + r * np.cos(pth + self.scan_dyaw + ang)
        ey = sy + r * np.sin(pth + self.scan_dyaw + ang)
        res, origin = self.map['res'], self.map['origin']
        H, W = self.map['H'], self.map['W']
        occ = self.map['occ']
        near_wall_m = 2 * res   # tolerancia MCL: si esta a <=2 celdas de una pared mapeada, no es nuevo
        added = 0
        for x, y in zip(ex, ey):
            gx = int((x - origin[0]) / res)
            gy = int((y - origin[1]) / res)
            if not (0 <= gx < W and 0 <= gy < H):
                continue
            if occ[gy, gx] != 0:
                continue   # celda ya ocupada/desconocida en el mapa
            if self.cost[gy, gx] < near_wall_m:
                continue   # cerca de una pared mapeada -> probable error MCL
            if (gx, gy) in self.dyn_obstacles:
                continue
            self.dyn_obstacles.add((gx, gy))
            added += 1
        if added:
            self._build_costmap()
            self.get_logger().info(f'Obstaculo no mapeado: +{added} celdas, re-planificando')
        self.last_dyn_added = added
        return added

    def _mark_front_blockage(self):
        """Marca una barrera chica al frente cuando el LIDAR ve algo demasiado
        cerca pero los hits finos no alcanzan para registrar un obstaculo nuevo.

        Es deliberadamente conservador: solo se usa tras bloqueos frontales
        repetidos y se limpia al recibir un nuevo goal.
        """
        if self.map is None or self.pose is None:
            return 0
        px, py, pth = self.pose
        res, origin = self.map['res'], self.map['origin']
        H, W = self.map['H'], self.map['W']
        clearance = self.last_forward_clearance
        if clearance is None or not math.isfinite(clearance):
            clearance = self.safety_stop
        mark_dist = max(self.safety_stop, min(0.60, clearance + 0.05))
        mark_radius_cells = max(1, int(math.ceil(self.front_obstacle_mark_radius / res)))
        added = 0
        for da in (-0.35, -0.18, 0.0, 0.18, 0.35):
            x = px + mark_dist * math.cos(pth + da)
            y = py + mark_dist * math.sin(pth + da)
            gx = int((x - origin[0]) / res)
            gy = int((y - origin[1]) / res)
            for yy in range(gy - mark_radius_cells, gy + mark_radius_cells + 1):
                for xx in range(gx - mark_radius_cells, gx + mark_radius_cells + 1):
                    if not (0 <= xx < W and 0 <= yy < H):
                        continue
                    if math.hypot(xx - gx, yy - gy) > mark_radius_cells:
                        continue
                    if (xx, yy) in self.dyn_obstacles:
                        continue
                    self.dyn_obstacles.add((xx, yy))
                    added += 1
        if added:
            self._build_costmap()
            self.get_logger().warn(
                f'Bloqueo frontal repetido: marco +{added} celdas dinamicas')
        self.last_dyn_added += added
        return added

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _recovery_elapsed(self):
        if self.recovery_started_s is None:
            return 0.0
        return max(0.0, self._now_s() - self.recovery_started_s)

    def _declare_blocked(self, reason):
        self.blocked_reason = reason
        self.state = 'IDLE'
        self.recovery_started_s = None
        self._stop()
        self.get_logger().warn(f'Bloqueado: {reason}; robot detenido')
        self._publish_debug(reason)

    def _enter_recovery(self, reason):
        if self.recovery_attempts >= self.max_recovery_attempts:
            self._declare_blocked('blocked_max_recovery_attempts')
            return
        self.recovery_attempts += 1
        self.recovery_started_s = self._now_s()
        self.state = 'RECOVERY'
        self._stop()
        self._publish_debug(reason)

    def _handle_front_blocked(self, reason):
        self.front_blocked_hits += 1
        added = self._register_obstacle_from_scan()
        if added == 0 and self.front_blocked_hits >= self.front_blocked_confirmations:
            added = self._mark_front_blockage()
        if added == 0:
            reason = f'{reason}_unregistered'
        self._enter_recovery(reason)

    # --------------------------------------------------------------------- #
    #   Control                                                             #
    # --------------------------------------------------------------------- #
    def control_loop(self):
        self.last_dyn_added = 0
        self.state_pub.publish(String(data=self.state))
        if self.state in ('IDLE', 'REACHED'):
            self._stop()
            self._publish_debug('stop_idle_or_reached')
            return
        if self.pose is None:
            self._stop()
            self._publish_debug('waiting_pose')
            return

        if self.state == 'PLANNING':
            if self.plan():
                self.state = 'FOLLOWING'
                note = 'plan_ok_following'
            else:
                self.state = 'IDLE'
                note = self.last_debug_note
            self._publish_debug(note)
            return

        # freno de emergencia SOLO por obstaculo NO mapeado (evita loop
        # FOLLOWING<->RECOVERY cuando el plan pasa cerca de una pared conocida
        # y el error de MCL hace que el LIDAR la vea a <safety_stop).
        clr = self._forward_clearance()
        self.last_forward_clearance = clr
        if self.state == 'FOLLOWING' and clr is not None and clr < self.safety_stop:
            self._handle_front_blocked('front_blocked_dynamic_obstacle')
            return
        if self.state == 'FOLLOWING' and (clr is None or clr >= self.safety_stop):
            self.front_blocked_hits = 0

        if self.state == 'RECOVERY':
            elapsed = self._recovery_elapsed()
            if elapsed < self.recovery_hold_s:
                self._stop()
                self._publish_debug('recovery_hold')
                return
            if elapsed < self.recovery_hold_s + self.recovery_backoff_s:
                tw = Twist()
                tw.linear.x = min(0.0, self.recovery_backoff_speed)
                self._publish_cmd(tw)
                self._publish_debug('recovery_short_backoff')
                return

            clr = self._forward_clearance()
            self.last_forward_clearance = clr
            if clr is not None and clr < self.safety_stop:
                self._handle_front_blocked('recovery_front_still_blocked')
                return
            if self.plan():
                self.state = 'FOLLOWING'
                self.recovery_started_s = None
                self._publish_debug('recovery_replan_ok')
            else:
                self._declare_blocked('blocked_no_recovery_plan')
            return

        if self.state == 'FOLLOWING':
            self._follow()
            self._publish_debug('following')
            return

        if self.state == 'ALIGNING':
            self._align()
            self._publish_debug('aligning')
            return

    def _follow(self):
        px, py, pth = self.pose
        gx, gy, _ = self.goal
        dist_goal = math.hypot(gx - px, gy - py)
        if dist_goal < self.goal_tol:
            self.state = 'ALIGNING'
            self.last_debug_note = 'goal_position_reached_aligning'
            return
        # pure pursuit: buscar el punto del camino a 'lookahead' del robot
        target = self._lookahead_point(px, py)
        if target is None:
            self.state = 'ALIGNING'
            self.last_debug_note = 'path_exhausted_aligning'
            return
        ang = math.atan2(target[1] - py, target[0] - px)
        err = wrap_angle(ang - pth)
        tw = Twist()
        # si el error angular es grande, girar en el lugar antes de avanzar
        if abs(err) > 0.8:
            tw.linear.x = 0.0
        else:
            slow = max(0.3, 1.0 - abs(err))
            tw.linear.x = self.v_max * slow * min(1.0, dist_goal / 0.3)
        tw.angular.z = max(-self.w_max, min(self.w_max, 1.8 * err))
        self._publish_cmd(tw)

    def _lookahead_point(self, px, py):
        if not self.path:
            return None
        # 1) avanzar el indice al punto del path mas cercano al robot, buscando
        # SOLO hacia adelante (no retroceder). Asi si el robot se desvia lateral,
        # sigue tirando el lookahead hacia adelante del path, en vez de mandarlo
        # al inicio (que era el bug del orbit).
        best_i, best_d = self.path_idx, float('inf')
        for i in range(self.path_idx, min(len(self.path), self.path_idx + 40)):
            x, y = self.path[i]
            d = math.hypot(x - px, y - py)
            if d < best_d:
                best_d = d
                best_i = i
        self.path_idx = best_i

        # 2) desde el indice actual, avanzar hasta acumular >= lookahead
        for i in range(self.path_idx, len(self.path)):
            x, y = self.path[i]
            if math.hypot(x - px, y - py) >= self.lookahead:
                return (x, y)
        return self.path[-1]

    def _align(self):
        _, _, pth = self.pose
        err = wrap_angle(self.goal[2] - pth)
        if abs(err) < self.yaw_tol:
            self._stop()
            self.state = 'REACHED'
            self.recovery_started_s = None
            self.last_debug_note = 'goal_reached'
            self.get_logger().info('Objetivo alcanzado (posicion y angulo).')
            return
        tw = Twist()
        # velocidad proporcional con piso: sin piso, cuando err ~ 0.15 la
        # velocidad cae bajo el ruido de la MCL y el robot nunca converge.
        min_w = 0.3
        w = 1.5 * err
        if abs(w) < min_w:
            w = math.copysign(min_w, err)
        tw.angular.z = max(-self.w_max, min(self.w_max, w))
        self._publish_cmd(tw)

    def _stop(self):
        self._publish_cmd(Twist())

    def _publish_cmd(self, msg):
        self.last_cmd = (float(msg.linear.x), float(msg.angular.z))
        self.cmd_pub.publish(msg)

    def _publish_debug(self, note):
        if note:
            self.last_debug_note = note
        payload = {
            'version': 1,
            'stamp_s': self.get_clock().now().nanoseconds * 1e-9,
            'state': self.state,
            'reason': self.last_debug_note,
            'note': self.last_debug_note,
            'pose': None,
            'goal': None,
            'path_len': int(len(self.path)),
            'path_idx': int(self.path_idx),
            'last_plan_cells': int(self.last_plan_cells),
            'last_plan_length_m': float(self.last_plan_length_m),
            'forward_clearance_m': self.last_forward_clearance,
            'safety_stop_m': float(self.safety_stop),
            'dyn_obstacle_cells': int(len(self.dyn_obstacles)),
            'last_dyn_added': int(self.last_dyn_added),
            'recovery_attempts': int(self.recovery_attempts),
            'max_recovery_attempts': int(self.max_recovery_attempts),
            'recovery_elapsed_s': float(self._recovery_elapsed()),
            'front_blocked_hits': int(self.front_blocked_hits),
            'blocked_reason': self.blocked_reason,
            'cmd': {'linear_x': self.last_cmd[0], 'angular_z': self.last_cmd[1]},
            'robot_radius_m': float(self.robot_radius),
            'inflation_m': float(self.inflation),
        }
        if self.pose is not None:
            payload['pose'] = {
                'x': float(self.pose[0]),
                'y': float(self.pose[1]),
                'yaw': float(self.pose[2]),
            }
        if self.goal is not None:
            payload['goal'] = {
                'x': float(self.goal[0]),
                'y': float(self.goal[1]),
                'yaw': float(self.goal[2]),
            }
        self.debug_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def wrap_angle_vec(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def main():
    rclpy.init()
    node = Navigator()
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
