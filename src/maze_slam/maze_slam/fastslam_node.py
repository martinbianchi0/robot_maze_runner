"""Nodo ROS 2 que ejecuta Grid-Based FastSLAM para la Parte A del TP Final.

Topics:
- Entrada: scan (LaserScan), odom (Odometry).  Por defecto /tb4_0/scan, /tb4_0/odom.
- Salida:  /map (OccupancyGrid), /belief (PoseStamped),
           /maze_slam/particles (PoseArray).
- TF:      map -> odom (correccion del SLAM; el resto del arbol lo da el bag).
- Trigger: publicar Empty en /maze_slam/save_request guarda el mapa en disco.

Notas de diseño:
- El update de SLAM se dispara solo tras movimiento significativo (update_min_*),
  no a la tasa de odom (20 Hz), para no acumular ruido del modelo de movimiento.
- El scan se integra en la pose del LIDAR, no de la base: el rplidar del TB4 esta
  montado rotado +90deg y -4cm en x (params sensor_*).
- map->odom se re-emite con stamp al futuro (transform_tolerance) para no provocar
  errores de extrapolacion en RViz.
"""

import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped, Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, QoSReliabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from tf2_ros import TransformBroadcaster

from maze_slam.fastslam import FastSLAM, wrap_angle


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class FastSLAMNode(Node):
    def __init__(self):
        super().__init__('maze_slam')

        self.declare_parameter('n_particles', 15)
        self.declare_parameter('map_size', 240)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('odom_topic', '/calc_odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_rate', 4.0)
        self.declare_parameter('maps_dir', 'maps')
        self.declare_parameter('seed', 42)
        # Disparar update de SLAM solo tras movimiento significativo (evita meter
        # ruido del modelo de movimiento a 20 Hz -> deriva rotacional).
        self.declare_parameter('update_min_trans', 0.04)   # m
        self.declare_parameter('update_min_rot', 0.04)      # rad (~2.3 deg)
        # Margen de validez de la TF map->odom hacia el futuro. Moderado: si es muy
        # chico RViz tira "extrapolation into the future"; si es muy grande, los scans
        # quedan "earlier than all data in the transform cache". 0.3 + timer rapido va.
        self.declare_parameter('transform_tolerance', 0.3)  # s
        # Transformacion fija base_link -> LIDAR (TB4: rplidar rotado +90deg, -4cm en x).
        self.declare_parameter('sensor_x', -0.040)
        self.declare_parameter('sensor_y', 0.0)
        self.declare_parameter('sensor_yaw', math.pi / 2.0)
        # Modelo de medicion / movimiento (valores elegidos en el barrido de tuning).
        self.declare_parameter('sigma_hit', 0.08)
        self.declare_parameter('alpha1', 0.04)
        self.declare_parameter('alpha2', 0.02)
        self.declare_parameter('alpha3', 0.05)
        self.declare_parameter('alpha4', 0.02)
        self.declare_parameter('use_scan_match', True)
        # publish_tf: True en el bag TB4 (somos duenios de map->odom). False en la
        # casa de la catedra, que ya publica un map->odom estatico (sino chocan).
        self.declare_parameter('publish_tf', True)

        n = int(self.get_parameter('n_particles').value)
        ms = int(self.get_parameter('map_size').value)
        res = float(self.get_parameter('resolution').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.maps_dir = self.get_parameter('maps_dir').value
        self.update_min_trans = float(self.get_parameter('update_min_trans').value)
        self.update_min_rot = float(self.get_parameter('update_min_rot').value)
        self.transform_tolerance = float(self.get_parameter('transform_tolerance').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        alpha = (
            float(self.get_parameter('alpha1').value),
            float(self.get_parameter('alpha2').value),
            float(self.get_parameter('alpha3').value),
            float(self.get_parameter('alpha4').value),
        )
        self.fs = FastSLAM(
            n_particles=n, map_size=ms, resolution=res,
            alpha=alpha,
            sigma_hit=float(self.get_parameter('sigma_hit').value),
            use_scan_match=bool(self.get_parameter('use_scan_match').value),
            sensor_x=float(self.get_parameter('sensor_x').value),
            sensor_y=float(self.get_parameter('sensor_y').value),
            sensor_yaw=float(self.get_parameter('sensor_yaw').value),
        )
        self.rng = np.random.default_rng(int(self.get_parameter('seed').value))

        # Callback groups separados para que /clock (default group) NUNCA se bloquee:
        # - fast_cbg: odom + TF timer (livianos, urgentes). Mantienen latest_stamp y
        #   el TF al dia a 20 Hz.
        # - heavy_cbg: on_scan (paso de SLAM ~45ms) + publish_state (publica el grid
        #   500x500, pesado). Aislados aca, no frenan al reloj ni al TF.
        # El /clock (lo crea rclpy con use_sim_time) queda en el default group, libre,
        # asi now() siempre refleja el tiempo del bag sin atrasos -> sin bajones de TF.
        self.fast_cbg = MutuallyExclusiveCallbackGroup()
        self.heavy_cbg = MutuallyExclusiveCallbackGroup()

        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        # /tb4_0/odom del bag se publica con reliability=BEST_EFFORT. Un sub RELIABLE
        # rechaza esos mensajes. Suscribimos con sensor_data (BEST_EFFORT, depth=5).
        odom_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data,
                                 callback_group=self.heavy_cbg)
        self.create_subscription(Odometry, odom_topic, self.on_odom, odom_qos,
                                 callback_group=self.fast_cbg)
        self.create_subscription(Empty, '/maze_slam/save_request', self.on_save, 10)
        # Alternativa con nombre: publicar String con basename (ej. "maze_slam")
        # para guardar en maps/<basename>.{pgm,yaml} y no pisar otros mapas.
        self.create_subscription(String, '/maze_slam/save_request_named',
                                 self.on_save_named, 10)
        self.save_basename = 'casa_slam'

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.pub_amcl = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.pub_belief = self.create_publisher(PoseStamped, '/belief', 10)
        self.pub_parts = self.create_publisher(PoseArray, '/maze_slam/particles', 10)
        self.tf_br = TransformBroadcaster(self)

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / max(rate, 0.5), self.publish_state, callback_group=self.heavy_cbg)
        # Timer rapido SOLO para el TF map->odom (barato), en el grupo concurrente.
        self.create_timer(0.05, self.publish_map_odom_tf, callback_group=self.fast_cbg)

        self.last_scan = None
        self.last_odom_pose = None  # (x, y, theta, stamp)
        self.scan_count = 0
        # Arrancamos con map->odom = identidad para que el arbol TF exista desde el
        # principio (antes del primer update de SLAM) y RViz no se queje al inicio.
        self.map_odom = (0.0, 0.0, 0.0)
        self.latest_stamp = None  # ultimo stamp de dato recibido (rclpy Time)
        self.base_frame = 'base_link'  # child del odom (lo confirma el 1er msg)
        self.display = None   # mejor particula cacheada (con histeresis) para publicar
        self.display_pid = None

        from maze_slam._kernels import HAS_NUMBA
        self.get_logger().info(
            f'maze_slam listo: N={n}, mapa={ms}x{ms} @ {res:.3f}m, '
            f'scan={scan_topic}, odom={odom_topic}, numba={HAS_NUMBA}'
        )
        if not HAS_NUMBA:
            self.get_logger().warn(
                'numba NO disponible -> integrate en Python puro (lento). '
                'Con config C (80p) el nodo se atrasa en vivo: instalá numba '
                '(pip3 install numba) o bajá el rate del bag (./shs/bag.sh --rate 0.5).')

    # -------- callbacks --------
    def on_odom(self, msg: Odometry):
        # Solo guardamos la ultima pose de odom. El delta del modelo de movimiento
        # se consume en on_scan, cuando el robot ya se movio lo suficiente.
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        th = quat_to_yaw(msg.pose.pose.orientation)
        self.last_odom_pose = (x, y, th, msg.header.stamp)
        self.latest_stamp = Time.from_msg(msg.header.stamp)
        # Auto-detectar frames del odom (frame padre y child = base del robot).
        if msg.header.frame_id and msg.header.frame_id != self.odom_frame:
            self.odom_frame = msg.header.frame_id
        if msg.child_frame_id:
            self.base_frame = msg.child_frame_id

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg
        self.latest_stamp = Time.from_msg(msg.header.stamp)
        if self.last_odom_pose is None:
            return
        x, y, th, _ = self.last_odom_pose
        first = self.scan_count == 0
        delta = self.fs.odom_delta(
            x, y, th, min_trans=self.update_min_trans, min_rot=self.update_min_rot
        )
        if delta is None and not first:
            # No se movio lo suficiente: dejamos el mapa estable, no re-integramos.
            return

        # Un paso de SLAM: motion -> scan-match -> weigh -> resample -> integrate.
        self.fs.step(delta, msg.ranges, msg.angle_min, msg.angle_increment,
                     msg.range_max, self.rng)
        self.scan_count += 1
        # Elegir la particula a mostrar CON HISTERESIS y cachearla: asi el mapa
        # publicado no salta entre hipotesis tick a tick (lo que se veia como
        # "el mapa cambia cada 2 segundos").
        self.display = self.fs.best_sticky(self.display_pid)
        self.display_pid = self.display.pid
        self._update_correction()
        # El TF map->odom lo publica el timer de 20 Hz (en su propio thread), no aca.

    def _update_correction(self):
        """Recalcula la TF map->odom a partir de la mejor particula y la ultima
        odometria. Se guarda y el timer la re-emite con stamp al futuro."""
        if self.last_odom_pose is None:
            return
        ox, oy, oth, _ = self.last_odom_pose
        best = self.display or self.fs.best()
        # T_map_odom = T_map_base * inv(T_odom_base). En 2D:
        dtheta = wrap_angle(best.theta - oth)
        c = math.cos(dtheta)
        s = math.sin(dtheta)
        tx = best.x - (c * ox - s * oy)
        ty = best.y - (s * ox + c * oy)
        self.map_odom = (tx, ty, dtheta)

    def on_save(self, _msg):
        self.save_map()

    def on_save_named(self, msg):
        name = msg.data.strip() or 'casa_slam'
        # sanitizar: solo basename, sin path traversal ni extension
        name = os.path.basename(name).removesuffix('.pgm').removesuffix('.yaml')
        self.save_basename = name
        self.save_map()

    # -------- publicacion periodica --------
    def publish_state(self):
        if self.last_scan is None or self.map_odom is None or self.display is None:
            return
        # Con use_sim_time=True el clock del nodo sigue el /clock del bag, asi que
        # now() esta a la altura de los timestamps de scan/odom.
        stamp = self.get_clock().now().to_msg()
        # Publicamos la particula cacheada con histeresis (no re-elegimos en cada
        # tick): mapa estable, sin parpadeo.
        best = self.display

        # OccupancyGrid del mejor mapa
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = self.map_frame
        grid.info.resolution = self.fs.res
        grid.info.width = self.fs.W
        grid.info.height = self.fs.H
        grid.info.origin.position.x = float(self.fs.origin)
        grid.info.origin.position.y = float(self.fs.origin)
        grid.info.origin.orientation.w = 1.0
        grid.data = self.fs.occupancy_data(best).flatten().tolist()
        self.pub_map.publish(grid)

        # belief
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.map_frame
        ps.pose.position.x = float(best.x)
        ps.pose.position.y = float(best.y)
        ps.pose.orientation = yaw_to_quat(best.theta)
        self.pub_belief.publish(ps)

        pc = PoseWithCovarianceStamped()
        pc.header.frame_id = 'map'
        pc.header.stamp = ps.header.stamp
        pc.pose.pose.position.x = float(best.x)
        pc.pose.pose.position.y = float(best.y)
        pc.pose.pose.orientation = yaw_to_quat(best.theta)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05      # var x, y
        cov[35] = 0.05              # var yaw
        pc.pose.covariance = cov
        self.pub_amcl.publish(pc)

        # particulas
        pa = PoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = self.map_frame
        for p in self.fs.particles:
            pose = Pose()
            pose.position.x = float(p.x)
            pose.position.y = float(p.y)
            pose.orientation = yaw_to_quat(p.theta)
            pa.poses.append(pose)
        self.pub_parts.publish(pa)
        # (el TF map->odom lo publica el timer rapido de 20 Hz, no aca)

    def publish_map_odom_tf(self, base_stamp=None):
        if not self.publish_tf or self.map_odom is None:
            return
        # Estampar con el tiempo del ULTIMO dato recibido (scan/odom), no con now():
        # al arranque el reloj del nodo va atrasado respecto a los mensajes (el /clock
        # tarda), y now()+tol caeria ANTES del scan -> "extrapolation into the future".
        now = self.get_clock().now()
        base = now
        if self.latest_stamp is not None and self.latest_stamp.nanoseconds > now.nanoseconds:
            base = self.latest_stamp
        stamp = (base + Duration(seconds=self.transform_tolerance)).to_msg()

        tfs = []
        # 1) map -> odom (correccion del SLAM)
        tx, ty, dtheta = self.map_odom
        tfs.append(self._make_tf(stamp, self.map_frame, self.odom_frame,
                                 tx, ty, dtheta))
        # 2) odom -> base: lo publicamos NOSOTROS desde la odometria, estampado
        #    adelante igual que map->odom. Asi toda la cadena tiene timing consistente
        #    y RViz no extrapola. El odom->base del bag (que va atrasado) lo ignoramos.
        if self.last_odom_pose is not None:
            ox, oy, oth, _ = self.last_odom_pose
            tfs.append(self._make_tf(stamp, self.odom_frame, self.base_frame,
                                     ox, oy, oth))
        self.tf_br.sendTransform(tfs)

    @staticmethod
    def _make_tf(stamp, parent, child, x, y, yaw):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x = float(x)
        tf.transform.translation.y = float(y)
        tf.transform.translation.z = 0.0
        tf.transform.rotation = yaw_to_quat(yaw)
        return tf

    # -------- guardado de mapa --------
    def save_map(self):
        if self.scan_count == 0:
            # Nunca llego un scan: no tiene sentido escribir un mapa vacio.
            return
        os.makedirs(self.maps_dir, exist_ok=True)
        best = self.display or self.fs.best()
        # PGM grises convencion ROS: 0=ocupado(negro), 254=libre(blanco), 205=desconocido.
        img = np.full((self.fs.H, self.fs.W), 205, dtype=np.uint8)
        img[best.log_odds > self.fs.occ_threshold] = 0
        img[best.log_odds < (self.fs.l_free * 0.5)] = 254
        # PGM se escribe top-down; ROS map_server espera el mismo formato.
        img_to_save = np.flipud(img)

        pgm_path = os.path.join(self.maps_dir, f'{self.save_basename}.pgm')
        yaml_path = os.path.join(self.maps_dir, f'{self.save_basename}.yaml')
        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{self.fs.W} {self.fs.H}\n255\n'.encode())
            f.write(img_to_save.tobytes())
        with open(yaml_path, 'w') as f:
            f.write(f'image: {self.save_basename}.pgm\n')
            f.write(f'resolution: {self.fs.res}\n')
            f.write(f'origin: [{self.fs.origin}, {self.fs.origin}, 0.0]\n')
            f.write('negate: 0\n')
            f.write('occupied_thresh: 0.65\n')
            f.write('free_thresh: 0.196\n')
        self.get_logger().info(f'Mapa guardado: {pgm_path} + {yaml_path}')


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = FastSLAMNode()
    # Multi-thread: el timer de TF (en su callback group) corre en paralelo al paso
    # pesado de SLAM, asi map->odom se publica parejo y RViz no se queja.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.save_map()
        except Exception as e:  # noqa: BLE001
            print(f'No pude guardar el mapa al cerrar: {e}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
