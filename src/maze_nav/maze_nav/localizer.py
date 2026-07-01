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
        self.declare_parameter('max_beams', 60)
        self.declare_parameter('alpha1', 0.05)         # ruido rot por rot
        self.declare_parameter('alpha2', 0.05)         # ruido rot por trans
        self.declare_parameter('alpha3', 0.05)         # ruido trans por trans
        self.declare_parameter('alpha4', 0.05)         # ruido trans por rot
        self.declare_parameter('update_min_d', 0.05)   # m para gatillar update
        self.declare_parameter('update_min_a', 0.05)   # rad para gatillar update
        self.declare_parameter('init_xy_std', 0.30)
        self.declare_parameter('init_yaw_std', 0.30)
        # Augmented MCL (Probabilistic Robotics 8.3): inyeccion de particulas
        # aleatorias cuando la likelihood cae respecto de una media larga.
        self.declare_parameter('alpha_slow', 0.001)
        self.declare_parameter('alpha_fast', 0.1)
        self.declare_parameter('resample_neff_ratio', 0.2)  # antes 1/3, ahora 1/5
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/calc_odom')
        # Offset del LIDAR (base_scan) respecto a base_footprint. TB3 burger:
        # base_scan esta 3.2 cm ATRAS del centro. Ignorar esto shifteaba el scan
        # ~0.6 celdas y hacia que MCL nunca alineara con el mapa.
        self.declare_parameter('scan_x_offset', -0.032)
        self.declare_parameter('scan_y_offset', 0.0)

        self.N = int(self.get_parameter('n_particles').value)
        self.sigma = float(self.get_parameter('sigma_hit').value)
        self.max_beams = int(self.get_parameter('max_beams').value)
        self.alphas = tuple(float(self.get_parameter(f'alpha{i}').value) for i in (1, 2, 3, 4))
        self.min_d = float(self.get_parameter('update_min_d').value)
        self.min_a = float(self.get_parameter('update_min_a').value)
        self.init_xy_std = float(self.get_parameter('init_xy_std').value)
        self.init_yaw_std = float(self.get_parameter('init_yaw_std').value)
        self.alpha_slow = float(self.get_parameter('alpha_slow').value)
        self.alpha_fast = float(self.get_parameter('alpha_fast').value)
        self.neff_ratio = float(self.get_parameter('resample_neff_ratio').value)
        self.scan_dx = float(self.get_parameter('scan_x_offset').value)
        self.scan_dy = float(self.get_parameter('scan_y_offset').value)
        self.w_slow = 0.0
        self.w_fast = 0.0
        self.last_mean_w = 1.0   # para debug
        self.prev_eth = None
        self.yaw_smooth = 0.35   # limite de salto de yaw por publicacion

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
        # Origen del scan en el mundo: base_footprint + rotacion del offset del LIDAR.
        c, s = np.cos(pth), np.sin(pth)
        sx = px + c * self.scan_dx - s * self.scan_dy
        sy = py + s * self.scan_dx + c * self.scan_dy
        ex = sx + r[None, :] * np.cos(pth + ang[None, :])
        ey = sy + r[None, :] * np.sin(pth + ang[None, :])
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
        # log-mean por particula, normalizado por # rayos -> promedio robusto para
        # augmented MCL sin que dependa del tamano del scan.
        n_beams_used = p.shape[1]
        mean_logw_per_beam = float(np.mean(logw)) / max(1, n_beams_used)
        w_avg = math.exp(mean_logw_per_beam)
        self.last_mean_w = w_avg
        self.w_slow += self.alpha_slow * (w_avg - self.w_slow)
        self.w_fast += self.alpha_fast * (w_avg - self.w_fast)

        logw -= logw.max()
        w = np.exp(logw)
        w += 1e-12
        w /= w.sum()
        self.weights = w

    def _resample(self):
        neff = 1.0 / np.sum(self.weights ** 2)
        if neff > self.N * self.neff_ratio:
            return
        # Fraccion de inyeccion (Augmented MCL). Umbral 5% para no disparar por
        # ruido chico, tope 5% para no romper la unimodalidad del filtro.
        w_diff = max(0.0, 1.0 - self.w_fast / max(self.w_slow, 1e-12))
        if w_diff < 0.05:
            w_diff = 0.0
        n_inject = int(self.N * min(w_diff, 0.05))

        n_keep = self.N - n_inject
        # resampleo sistematico sobre las particulas actuales
        positions = (np.arange(n_keep) + self.rng.random()) / n_keep
        cumsum = np.cumsum(self.weights)
        cumsum[-1] = 1.0
        new_idx = np.searchsorted(cumsum, positions)
        kept = self.particles[new_idx].copy()
        # roughening: pequeno jitter para evitar empobrecimiento
        kept[:, 0] += self.rng.normal(0, 0.01, n_keep)
        kept[:, 1] += self.rng.normal(0, 0.01, n_keep)
        kept[:, 2] += self.rng.normal(0, 0.01, n_keep)

        if n_inject > 0:
            injected = self._sample_free_cells(n_inject)
            self.particles = np.concatenate([kept, injected], axis=0)
            self.get_logger().info(
                f'MCL: inyecto {n_inject} particulas (w_fast={self.w_fast:.3g}, '
                f'w_slow={self.w_slow:.3g})')
        else:
            self.particles = kept
        self.weights = np.full(self.N, 1.0 / self.N)

    def _sample_free_cells(self, n):
        """Devuelve n poses uniformes sobre celdas libres del mapa (yaw random)."""
        ys, xs = np.where(self.free)
        if len(xs) == 0:
            return np.zeros((n, 3))
        idx = self.rng.integers(0, len(xs), size=n)
        res = self.map_meta['res']
        ox, oy = self.map_meta['origin']
        px = ox + (xs[idx] + 0.5) * res
        py = oy + (ys[idx] + 0.5) * res
        pth = self.rng.uniform(-math.pi, math.pi, size=n)
        return np.stack([px, py, pth], axis=1)

    # --------------------------------------------------------------------- #
    def _publish_estimate(self, stamp):
        # Reporte por MODA: usar solo el top-20% de particulas por peso. La media
        # ponderada global es inestable cuando la distribucion es multimodal
        # (yaw en mapas simetricos, o justo despues de inyeccion aleatoria).
        n_top = max(10, self.N // 5)
        top_idx = np.argpartition(self.weights, -n_top)[-n_top:]
        tw = self.weights[top_idx]
        tw = tw / tw.sum()
        ex = float(np.sum(self.particles[top_idx, 0] * tw))
        ey = float(np.sum(self.particles[top_idx, 1] * tw))
        eth = angle_mean(self.particles[top_idx, 2], tw)

        # Smoothing de yaw: cap el salto entre publicaciones consecutivas para no
        # confundir al navigator con thrashing de la estimacion angular.
        if self.prev_eth is not None:
            d = wrap_angle(eth - self.prev_eth)
            if abs(d) > self.yaw_smooth:
                eth = wrap_angle(self.prev_eth + math.copysign(self.yaw_smooth, d))
        self.prev_eth = eth
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
        cov[0] = float(np.sum(self.weights * dx * dx))
        cov[7] = float(np.sum(self.weights * dy * dy))
        dth = np.array([wrap_angle(a - eth) for a in self.particles[:, 2]])
        cov[35] = float(np.sum(self.weights * dth * dth))
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
