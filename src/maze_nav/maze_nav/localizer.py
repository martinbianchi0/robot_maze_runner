"""Localizacion Monte Carlo (MCL) sobre el mapa estatico de la Parte A.

Filtro de particulas con:
- Prediccion: modelo de movimiento por odometria sobre /calc_odom (ruidosa).
- Correccion: likelihood field (endpoint model) contra el mapa conocido.
- Inicializacion: /initialpose (herramienta "2D Pose Estimate" de RViz).

Publica /amcl_pose (estimacion), /particlecloud (nube para visualizar) y el TF
map->calc_odom (la correccion de la deriva de la odometria). NUNCA usa /odom
(ground truth): solo odometria ruidosa + LIDAR + mapa. Consigna Parte B, 1.1 y 1.4.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from scipy.ndimage import distance_transform_edt

from geometry_msgs.msg import (PoseWithCovarianceStamped, PoseArray, Pose,
                               TransformStamped, Quaternion)
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from maze_nav.nav_utils import wrap_angle, angle_mean, sample_motion, odom_deltas


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Localizer(Node):
    def __init__(self):
        super().__init__('localizer')
        self.declare_parameter('n_particles', 600)
        self.declare_parameter('sigma_hit', 0.35)      # m, ancho del likelihood (gentil)
        self.declare_parameter('max_beams', 30)
        self.declare_parameter('alpha1', 0.05)         # ruido rot por rot
        self.declare_parameter('alpha2', 0.05)         # ruido rot por trans
        self.declare_parameter('alpha3', 0.05)         # ruido trans por trans
        self.declare_parameter('alpha4', 0.05)         # ruido trans por rot
        self.declare_parameter('update_min_d', 0.05)   # m para gatillar update
        self.declare_parameter('update_min_a', 0.05)   # rad para gatillar update
        self.declare_parameter('init_xy_std', 0.30)
        self.declare_parameter('init_yaw_std', 0.30)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/calc_odom')

        self.N = int(self.get_parameter('n_particles').value)
        self.sigma = float(self.get_parameter('sigma_hit').value)
        self.max_beams = int(self.get_parameter('max_beams').value)
        self.alphas = tuple(float(self.get_parameter(f'alpha{i}').value) for i in (1, 2, 3, 4))
        self.min_d = float(self.get_parameter('update_min_d').value)
        self.min_a = float(self.get_parameter('update_min_a').value)
        self.init_xy_std = float(self.get_parameter('init_xy_std').value)
        self.init_yaw_std = float(self.get_parameter('init_yaw_std').value)

        self.rng = np.random.default_rng(0)
        self.particles = None            # (N,3)
        self.weights = None              # (N,)
        self.dist_field = None           # likelihood field (m)
        self.map_meta = None
        self.last_odom = None            # (x,y,theta) usada en el ultimo update
        self.cur_odom = None             # (x,y,theta) mas reciente
        self.prev_est = None             # estimado anterior (histeresis del modo)
        self.have_map = False

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.on_initpose, 10)

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.cloud_pub = self.create_publisher(PoseArray, '/particlecloud', 10)
        self.tf_bc = TransformBroadcaster(self)

        self.get_logger().info(f'Localizer MCL listo: N={self.N}, esperando /map e /initialpose')

    # --------------------------------------------------------------------- #
    def on_map(self, msg):
        if self.have_map:
            return
        W, H = msg.info.width, msg.info.height
        res = msg.info.resolution
        origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        occ = np.array(msg.data, dtype=np.int8).reshape(H, W)
        self.map_meta = {'res': res, 'origin': origin, 'H': H, 'W': W}
        # likelihood field: distancia (m) a la celda ocupada mas cercana
        occupied = (occ == 100)
        if not occupied.any():
            self.get_logger().warn('El mapa no tiene celdas ocupadas!')
        self.dist_field = distance_transform_edt(~occupied).astype(np.float32) * res
        self.free = (occ == 0)
        self.have_map = True
        self.get_logger().info(f'Likelihood field listo ({W}x{H}). Fijá la pose inicial en RViz.')
        # arranque por defecto: disperso alrededor del spawn (0,0,0)
        self._init_particles(0.0, 0.0, 0.0, spread=1.0)

    def on_odom(self, msg):
        p = msg.pose.pose
        self.cur_odom = (p.position.x, p.position.y, quat_to_yaw(p.orientation))
        if self.last_odom is None:
            self.last_odom = self.cur_odom

    def on_initpose(self, msg):
        p = msg.pose.pose
        yaw = quat_to_yaw(p.orientation)
        self.get_logger().info(f'Pose inicial fijada: ({p.position.x:.2f}, {p.position.y:.2f}, {math.degrees(yaw):.0f}deg)')
        self._init_particles(p.position.x, p.position.y, yaw)

    def _init_particles(self, x, y, yaw, spread=1.0):
        n = self.N
        self.particles = np.empty((n, 3), dtype=np.float64)
        self.particles[:, 0] = x + self.rng.normal(0, self.init_xy_std * spread, n)
        self.particles[:, 1] = y + self.rng.normal(0, self.init_xy_std * spread, n)
        self.particles[:, 2] = yaw + self.rng.normal(0, self.init_yaw_std * spread, n)
        self.weights = np.full(n, 1.0 / n)
        self.last_odom = self.cur_odom
        self.prev_est = (x, y)

    # --------------------------------------------------------------------- #
    def on_scan(self, msg):
        if not self.have_map or self.particles is None or self.cur_odom is None:
            return
        # 1) prediccion (motion) desde la ultima odom usada
        if self.last_odom is None:
            self.last_odom = self.cur_odom
        deltas = odom_deltas(self.last_odom, self.cur_odom)
        moved = deltas[1] > self.min_d or abs(wrap_angle(self.cur_odom[2] - self.last_odom[2])) > self.min_a
        if moved:
            sample_motion(self.particles, deltas, self.alphas, self.rng)
            self.last_odom = self.cur_odom
            # 2) correccion (measurement) solo cuando el robot se movio
            self._measurement_update(msg)
            self._resample()
        self._publish_estimate(msg.header.stamp)

    def _measurement_update(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        n_beams = len(ranges)
        step = max(1, n_beams // self.max_beams)
        idx = np.arange(0, n_beams, step)
        r = ranges[idx]
        ang = scan.angle_min + idx * scan.angle_increment
        rmax = scan.range_max if scan.range_max > 0 else 3.5
        valid = np.isfinite(r) & (r > scan.range_min) & (r < rmax)
        r = r[valid]
        ang = ang[valid]
        if len(r) == 0:
            return
        res = self.map_meta['res']
        ox, oy = self.map_meta['origin']
        H, W = self.map_meta['H'], self.map_meta['W']

        px = self.particles[:, 0][:, None]
        py = self.particles[:, 1][:, None]
        pth = self.particles[:, 2][:, None]
        ex = px + r[None, :] * np.cos(pth + ang[None, :])
        ey = py + r[None, :] * np.sin(pth + ang[None, :])
        gx = ((ex - ox) / res).astype(np.int32)
        gy = ((ey - oy) / res).astype(np.int32)
        inb = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
        gx = np.clip(gx, 0, W - 1)
        gy = np.clip(gy, 0, H - 1)
        d = self.dist_field[gy, gx]
        d[~inb] = 2.0   # fuera del mapa: penalizacion
        # Modelo de mezcla (Probabilistic Robotics, likelihood field): cada rayo
        # aporta z_hit*gaussiana + z_rand. El termino uniforme acota la penalizacion
        # por rayo -> evita la sobre-confianza que colapsaba el filtro a una
        # hipotesis simetrica equivocada.
        q = np.exp(-(d * d) / (2.0 * self.sigma * self.sigma))
        p = 0.8 * q + 0.2
        logw = np.sum(np.log(p), axis=1)
        logw -= logw.max()
        w = np.exp(logw)
        w += 1e-12
        w /= w.sum()
        self.weights = w

    def _resample(self):
        neff = 1.0 / np.sum(self.weights ** 2)
        if neff > self.N / 3.0:
            return
        # resampleo sistematico
        positions = (np.arange(self.N) + self.rng.random()) / self.N
        cumsum = np.cumsum(self.weights)
        cumsum[-1] = 1.0
        new_idx = np.searchsorted(cumsum, positions)
        self.particles = self.particles[new_idx].copy()
        # roughening: pequeno jitter para evitar empobrecimiento
        self.particles[:, 0] += self.rng.normal(0, 0.01, self.N)
        self.particles[:, 1] += self.rng.normal(0, 0.01, self.N)
        self.particles[:, 2] += self.rng.normal(0, 0.01, self.N)
        self.weights = np.full(self.N, 1.0 / self.N)

    # --------------------------------------------------------------------- #
    def _publish_estimate(self, stamp):
        # Estimacion robusta con histeresis temporal: nos quedamos con el modo
        # (cluster) cercano al estimado anterior. Asi el reporte no salta entre
        # hipotesis simetricas del mapa; solo "salta" si ese modo se muere de peso
        # (relocalizacion / secuestro). El promedio global seria inutil si el filtro
        # es bimodal (daria un punto intermedio sin sentido).
        if self.prev_est is not None:
            cx, cy = self.prev_est
        else:
            best = int(np.argmax(self.weights))
            cx, cy = self.particles[best, 0], self.particles[best, 1]
        near = (np.abs(self.particles[:, 0] - cx) < 0.7) & \
               (np.abs(self.particles[:, 1] - cy) < 0.7)
        w = self.weights * near
        if w.sum() < 1e-6:
            # el modo anterior se quedo sin peso: reenganchar al modo mas probable
            best = int(np.argmax(self.weights))
            bx, by = self.particles[best, 0], self.particles[best, 1]
            near = (np.abs(self.particles[:, 0] - bx) < 0.7) & \
                   (np.abs(self.particles[:, 1] - by) < 0.7)
            w = self.weights * near
        w = w / w.sum()
        ex = float(np.sum(self.particles[:, 0] * w))
        ey = float(np.sum(self.particles[:, 1] * w))
        eth = angle_mean(self.particles[:, 2], w)
        self.prev_est = (ex, ey)

        # /amcl_pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = ex
        msg.pose.pose.position.y = ey
        msg.pose.pose.orientation = yaw_to_quat(eth)
        dx = self.particles[:, 0] - ex
        dy = self.particles[:, 1] - ey
        cov = msg.pose.covariance
        cov[0] = float(np.sum(w * dx * dx))
        cov[7] = float(np.sum(w * dy * dy))
        dth = np.array([wrap_angle(a - eth) for a in self.particles[:, 2]])
        cov[35] = float(np.sum(w * dth * dth))
        self.pose_pub.publish(msg)

        # /particlecloud (submuestreo para no saturar RViz)
        cloud = PoseArray()
        cloud.header = msg.header
        stepc = max(1, self.N // 200)
        for i in range(0, self.N, stepc):
            pose = Pose()
            pose.position.x = float(self.particles[i, 0])
            pose.position.y = float(self.particles[i, 1])
            pose.orientation = yaw_to_quat(float(self.particles[i, 2]))
            cloud.poses.append(pose)
        self.cloud_pub.publish(cloud)

        # TF map->calc_odom (correccion de la deriva de la odometria)
        if self.cur_odom is not None:
            ox, oy, oth = self.cur_odom
            th_mo = wrap_angle(eth - oth)
            c, s = math.cos(th_mo), math.sin(th_mo)
            tx = ex - (c * ox - s * oy)
            ty = ey - (s * ox + c * oy)
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'map'
            t.child_frame_id = 'calc_odom'
            t.transform.translation.x = tx
            t.transform.translation.y = ty
            t.transform.rotation = yaw_to_quat(th_mo)
            self.tf_bc.sendTransform(t)


def main():
    rclpy.init()
    node = Localizer()
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
