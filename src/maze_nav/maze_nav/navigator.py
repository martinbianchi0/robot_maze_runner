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
        self.declare_parameter('yaw_tol', 0.10)        # rad
        self.declare_parameter('safety_stop', 0.22)    # m, freno de emergencia
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('scan_topic', '/scan')

        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.inflation = float(self.get_parameter('inflation').value)
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.goal_tol = float(self.get_parameter('goal_tol').value)
        self.yaw_tol = float(self.get_parameter('yaw_tol').value)
        self.safety_stop = float(self.get_parameter('safety_stop').value)

        self.map = None                 # dict con occ,res,origin,H,W
        self.cost = None                # EDT (m) a obstaculo, para penalizar cercania
        self.blocked = None             # bool grid: no navegable (obstaculo inflado)
        self.dyn_obstacles = set()      # celdas de obstaculos no mapeados (gx,gy)
        self.pose = None                # (x,y,theta) estimada
        self.goal = None                # (x,y,theta)
        self.path = []                  # lista de (x,y) en mundo
        self.state = 'IDLE'
        self.scan = None

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
        self.get_logger().info(
            f'Nuevo objetivo: ({self.goal[0]:.2f}, {self.goal[1]:.2f}, {math.degrees(yaw):.0f}deg)')
        self.state = 'PLANNING'

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
            return False

        # reconstruir y pasar a mundo
        cells = [g]
        while cells[-1] != s:
            cells.append(came[cells[-1]])
        cells.reverse()
        self.path = [grid_to_world(gx, gy, origin, res) for (gx, gy) in cells]
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
        front = np.abs(wrap_angle_vec(ang)) < 0.5   # +/- ~30 grados
        rr = r[front]
        rr = rr[np.isfinite(rr) & (rr > self.scan.range_min)]
        return float(rr.min()) if len(rr) else None

    def _register_obstacle_from_scan(self):
        """Agrega al costmap los impactos del LIDAR que caen en celdas 'libres'
        del mapa (=> obstaculo no mapeado) y re-infla."""
        if self.scan is None or self.pose is None:
            return
        r = np.asarray(self.scan.ranges, dtype=np.float32)
        ang = self.scan.angle_min + np.arange(len(r)) * self.scan.angle_increment
        ok = np.isfinite(r) & (r > self.scan.range_min) & (r < 2.0)
        r, ang = r[ok], ang[ok]
        px, py, pth = self.pose
        ex = px + r * np.cos(pth + ang)
        ey = py + r * np.sin(pth + ang)
        res, origin = self.map['res'], self.map['origin']
        H, W = self.map['H'], self.map['W']
        added = 0
        for x, y in zip(ex, ey):
            gx = int((x - origin[0]) / res)
            gy = int((y - origin[1]) / res)
            if 0 <= gx < W and 0 <= gy < H and self.map['occ'][gy, gx] == 0:
                if (gx, gy) not in self.dyn_obstacles:
                    self.dyn_obstacles.add((gx, gy))
                    added += 1
        if added:
            self._build_costmap()
            self.get_logger().info(f'Obstaculo no mapeado: +{added} celdas, re-planificando')

    # --------------------------------------------------------------------- #
    #   Control                                                             #
    # --------------------------------------------------------------------- #
    def control_loop(self):
        self.state_pub.publish(String(data=self.state))
        if self.state in ('IDLE', 'REACHED'):
            self._stop()
            return
        if self.pose is None:
            self._stop()
            return

        if self.state == 'PLANNING':
            if self.plan():
                self.state = 'FOLLOWING'
            else:
                self.state = 'IDLE'
            return

        # freno de emergencia por obstaculo muy cercano
        clr = self._forward_clearance()
        if self.state == 'FOLLOWING' and clr is not None and clr < self.safety_stop:
            self._stop()
            self._register_obstacle_from_scan()
            self.state = 'RECOVERY'
            return

        if self.state == 'RECOVERY':
            if self.plan():
                self.state = 'FOLLOWING'
            else:
                # sin camino: retroceder un poco y reintentar
                tw = Twist()
                tw.linear.x = -0.06
                self.cmd_pub.publish(tw)
            return

        if self.state == 'FOLLOWING':
            self._follow()
            return

        if self.state == 'ALIGNING':
            self._align()
            return

    def _follow(self):
        px, py, pth = self.pose
        gx, gy, _ = self.goal
        dist_goal = math.hypot(gx - px, gy - py)
        if dist_goal < self.goal_tol:
            self.state = 'ALIGNING'
            return
        # pure pursuit: buscar el punto del camino a 'lookahead' del robot
        target = self._lookahead_point(px, py)
        if target is None:
            self.state = 'ALIGNING'
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
        self.cmd_pub.publish(tw)

    def _lookahead_point(self, px, py):
        if not self.path:
            return None
        # descartar los puntos ya pasados (mas cercanos que lookahead), quedarse
        # con el primero mas alla del radio de lookahead
        best = None
        for (x, y) in self.path:
            if math.hypot(x - px, y - py) >= self.lookahead:
                best = (x, y)
                break
        if best is None:
            best = self.path[-1]  # cerca del final: apuntar al ultimo punto
        return best

    def _align(self):
        _, _, pth = self.pose
        err = wrap_angle(self.goal[2] - pth)
        if abs(err) < self.yaw_tol:
            self._stop()
            self.state = 'REACHED'
            self.get_logger().info('Objetivo alcanzado (posicion y angulo).')
            return
        tw = Twist()
        tw.angular.z = max(-self.w_max, min(self.w_max, 1.5 * err))
        self.cmd_pub.publish(tw)

    def _stop(self):
        self.cmd_pub.publish(Twist())


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
