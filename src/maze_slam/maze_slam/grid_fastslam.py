#!/usr/bin/env python3
"""Etapa 2 - Grid-Based FastSLAM (Opcion 1).

Cada particula = hipotesis de trayectoria + mapa de ocupacion propio
en log-odds. Pesado por likelihood field (distance transform de la grilla
ocupada de cada particula) evaluado sobre los endpoints del scan.

Inputs:
  /calc_odom (motion source)
  /scan (sensor)
  /odom (solo para Path de ground truth en RViz - debug)

Outputs:
  /map         OccupancyGrid del mapa de la mejor particula
  /belief      PoseStamped pose corregida (mejor particula)
  /particles   PoseArray (todas las hipotesis, debug)
  /belief_path nav_msgs/Path acumulado del belief
  /real_path   nav_msgs/Path acumulado del /odom (debug)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseStamped, PoseArray

from maze_slam.utils import (
    yaw_from_quaternion, quaternion_from_yaw,
    logodds_to_occupancy, update_map_from_scan,
    odom_deltas, sample_motion, systematic_resample, n_eff,
    world_to_grid_vec, L_CLIP,
    scan_match_local, scan_endpoints_robot_frame,
    get_backend, to_numpy, compute_distance_transform,
)


class GridFastSLAM(Node):

    def __init__(self):
        super().__init__('grid_fastslam')

        # Parametros
        self.declare_parameter('map_size_m', 16.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('max_range', 3.5)
        self.declare_parameter('n_particles', 20)
        self.declare_parameter('odom_topic', '/calc_odom')
        self.declare_parameter('truth_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        # Trigger de actualizacion (no procesar todos los scans)
        self.declare_parameter('min_d_trans', 0.05)
        self.declare_parameter('min_d_rot', 0.05)
        # Beams a usar (subsample para velocidad)
        self.declare_parameter('beam_step', 4)
        # Sigma del likelihood field (m)
        self.declare_parameter('sigma_hit', 0.10)
        # Ruido motion model conservador: el grueso de la correccion lo hace
        # scan-matching, no la dispersion de particulas.
        self.declare_parameter('alpha1', 0.05)
        self.declare_parameter('alpha2', 0.02)
        self.declare_parameter('alpha3', 0.05)
        self.declare_parameter('alpha4', 0.02)
        self.declare_parameter('publish_period_s', 1.0)
        self.declare_parameter('seed', 0)
        # Scan-matching local (correlative grid search).
        # EXPERIMENTAL: implementacion vectorizada funciona en unit-tests
        # pero la integracion con ref_dt compartida colapsa diversidad de
        # particulas y diverge en sim. Para activar hacen falta:
        #   - per-particle distance transform (cada particula matchea contra
        #     su propio mapa) - improved proposal estilo Grisetti
        #   - sampling con covarianza estimada del cost landscape
        # Dejado off por default; cuando tengamos rosbag real lo retomamos.
        self.declare_parameter('scan_match', False)
        self.declare_parameter('match_win_xy', 0.10)         # ventana m
        self.declare_parameter('match_step_xy', 0.02)        # paso m
        self.declare_parameter('match_win_th_deg', 3.0)      # ventana grados (limitada)
        self.declare_parameter('match_step_th_deg', 1.0)     # paso grados
        self.declare_parameter('match_reg_xy', 50.0)         # prior xy (mas peso)
        self.declare_parameter('match_reg_th', 50000.0)      # prior theta (muy estricto)
        # No hacer scan-match hasta que el mapa tenga al menos N celdas ocupadas:
        # con mapa sparse las paredes "tiran" mal y el match diverge.
        self.declare_parameter('match_min_occ', 800)
        self.declare_parameter('min_range', 0.12)            # filtro lecturas invalidas LIDAR
        # Backend: 'auto' (intenta GPU), 'cpu', 'gpu'
        self.declare_parameter('backend', 'auto')
        self.declare_parameter('gpu_device', -1)         # -1 = default GPU
        self.declare_parameter('gpu_mem_limit_gb', 0.0)  # 0 = sin limite
        # Per-particle DT: cada particula matchea/pesa contra SU mapa (FastSLAM
        # canonical / improved proposal). Requiere mas computo; con backend=gpu
        # es factible para N=20-30.
        self.declare_parameter('per_particle_dt', False)

        size_m = float(self.get_parameter('map_size_m').value)
        self.res = float(self.get_parameter('resolution').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.N = int(self.get_parameter('n_particles').value)
        odom_topic = self.get_parameter('odom_topic').value
        truth_topic = self.get_parameter('truth_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        map_topic = self.get_parameter('map_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.min_d_trans = float(self.get_parameter('min_d_trans').value)
        self.min_d_rot = float(self.get_parameter('min_d_rot').value)
        self.beam_step = int(self.get_parameter('beam_step').value)
        self.sigma_hit = float(self.get_parameter('sigma_hit').value)
        self.alpha1 = float(self.get_parameter('alpha1').value)
        self.alpha2 = float(self.get_parameter('alpha2').value)
        self.alpha3 = float(self.get_parameter('alpha3').value)
        self.alpha4 = float(self.get_parameter('alpha4').value)
        pub_period = float(self.get_parameter('publish_period_s').value)
        seed = int(self.get_parameter('seed').value)
        self.use_match = bool(self.get_parameter('scan_match').value)
        self.match_win_xy = float(self.get_parameter('match_win_xy').value)
        self.match_step_xy = float(self.get_parameter('match_step_xy').value)
        self.match_win_th = math.radians(float(self.get_parameter('match_win_th_deg').value))
        self.match_step_th = math.radians(float(self.get_parameter('match_step_th_deg').value))
        self.match_reg_xy = float(self.get_parameter('match_reg_xy').value)
        self.match_reg_th = float(self.get_parameter('match_reg_th').value)
        self.match_min_occ = int(self.get_parameter('match_min_occ').value)
        self.min_range = float(self.get_parameter('min_range').value)
        backend_pref = self.get_parameter('backend').value
        gpu_device = int(self.get_parameter('gpu_device').value)
        gpu_mem_limit = float(self.get_parameter('gpu_mem_limit_gb').value)
        self.per_particle_dt = bool(self.get_parameter('per_particle_dt').value)
        # Resuelve backend (CuPy + fallback a NumPy si no esta o falla).
        try:
            self.xp, self.dt_edt, self.backend_name = get_backend(
                backend_pref,
                device=gpu_device if gpu_device >= 0 else None,
                mem_limit_gb=gpu_mem_limit if gpu_mem_limit > 0 else None,
            )
        except Exception as e:
            self.get_logger().warn(f'backend "{backend_pref}" no disponible ({e}); usando cpu')
            self.xp, self.dt_edt, self.backend_name = get_backend('cpu')

        self.H = int(round(size_m / self.res))
        self.W = self.H
        self.origin_x = -size_m / 2.0
        self.origin_y = -size_m / 2.0

        self.rng = np.random.default_rng(seed if seed != 0 else None)

        # Estado de particulas
        self.poses = np.zeros((self.N, 3), dtype=np.float64)        # (x,y,yaw)
        self.maps = np.zeros((self.N, self.H, self.W), dtype=np.float32)
        self.weights = np.ones(self.N, dtype=np.float64) / self.N

        # Odometria: bufferea el ultimo /calc_odom; el motion model se aplica
        # UNA vez por scan (sampleando con el delta acumulado), no por callback
        # de odom. Sino se acumula varianza de mas (sum-of-squares > square-of-sum).
        self.last_odom_xyt = None       # pose mas reciente recibida en /calc_odom
        self.last_scan_xyt = None       # pose de /calc_odom cuando ultima vez procesamos scan
        self.scan_angles = None
        self.scan_count = 0
        self.updates = 0
        self.resamples = 0
        # Stats PRE-resample (cuando son significativos)
        self.last_n_eff_pre = float(self.N)
        self.last_spread_pre = 0.0

        # Subs / pubs
        self.create_subscription(Odometry, odom_topic, self.cb_odom, 50)
        self.create_subscription(LaserScan, scan_topic, self.cb_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, truth_topic, self.cb_truth, 10)

        # /map necesita TRANSIENT_LOCAL (latching) para que map_saver_cli y
        # los consumidores tipo nav2 lo lean correctamente.
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, map_topic, map_qos)
        self.belief_pub = self.create_publisher(PoseStamped, '/belief', 10)
        self.particles_pub = self.create_publisher(PoseArray, '/particles', 1)
        self.belief_path_pub = self.create_publisher(Path, '/belief_path', 1)
        self.real_path_pub = self.create_publisher(Path, '/real_path', 1)

        self.belief_path = Path(); self.belief_path.header.frame_id = self.map_frame
        self.real_path = Path();   self.real_path.header.frame_id = self.map_frame

        self.create_timer(pub_period, self.publish_outputs)

        self.get_logger().info(
            f'grid_fastslam: N={self.N}, grid={self.H}x{self.W} @ {self.res}m, '
            f'sigma_hit={self.sigma_hit}, beam_step={self.beam_step}, '
            f'scan_match={self.use_match} (win={self.match_win_xy}m/{math.degrees(self.match_win_th):.0f}°), '
            f'backend={self.backend_name}, per_particle_dt={self.per_particle_dt}'
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def cb_truth(self, msg: Odometry):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.map_frame
        ps.pose = msg.pose.pose
        self.real_path.poses.append(ps)
        if len(self.real_path.poses) > 5000:
            self.real_path.poses = self.real_path.poses[-5000:]

    def cb_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.last_odom_xyt = (x, y, yaw)
        if self.last_scan_xyt is None:
            self.last_scan_xyt = self.last_odom_xyt
        self._odom_msgs = getattr(self, '_odom_msgs', 0) + 1

    def cb_scan(self, msg: LaserScan):
        if self.last_odom_xyt is None or self.last_scan_xyt is None:
            return
        # Trigger: solo procesar si nos movimos lo suficiente desde el ultimo scan
        dx = self.last_odom_xyt[0] - self.last_scan_xyt[0]
        dy = self.last_odom_xyt[1] - self.last_scan_xyt[1]
        dth_signed = self.last_odom_xyt[2] - self.last_scan_xyt[2]
        dth = abs(math.atan2(math.sin(dth_signed), math.cos(dth_signed)))
        moved_enough = (math.hypot(dx, dy) >= self.min_d_trans
                        or dth >= self.min_d_rot)
        if not moved_enough and self.scan_count > 0:
            return

        # Motion model: aplicar UNA vez con el delta acumulado
        if self.scan_count > 0:
            dr1, dt, dr2 = odom_deltas(self.last_scan_xyt, self.last_odom_xyt)
            self.poses = sample_motion(self.poses, dr1, dt, dr2,
                                       self.alpha1, self.alpha2,
                                       self.alpha3, self.alpha4, self.rng)

        if self.scan_angles is None or len(self.scan_angles) != len(msg.ranges):
            self.scan_angles = (np.arange(len(msg.ranges)) * msg.angle_increment
                                + msg.angle_min)
        ranges = np.asarray(msg.ranges, dtype=np.float64)

        # Subsample beams para pesar (mas barato)
        step = max(1, self.beam_step)
        r_sub = ranges[::step]
        a_sub = self.scan_angles[::step]

        self._weight_and_update(r_sub, a_sub, ranges, self.scan_angles)
        self.scan_count += 1
        self.last_scan_xyt = self.last_odom_xyt

    # ── FastSLAM core ────────────────────────────────────────────────────────

    def _weight_and_update(self, r_sub, a_sub, r_full, a_full):
        """Pipeline FastSLAM con improved proposal (estilo Grisetti/gmapping):
        para cada particula -> scan-match local contra mapa de referencia ->
        pesa en la pose corregida -> actualiza mapa propio.

        Sin scan-matching el sistema cae al pipeline anterior (pesar contra
        referencia comun + resamplear). Es el comportamiento cuando
        scan_match=False.
        """
        finite = np.isfinite(r_sub) & (r_sub > 0.0) & (r_sub < self.max_range)
        log_w = np.zeros(self.N, dtype=np.float64)

        # Distance transform(s) usados para scan-match y weighting.
        # Modo shared (default): una DT, mapa de la mejor particula del paso
        #   anterior. Rapido pero pierde diversidad con scan-match activo.
        # Modo per-particle: N DTs, una por particula contra su propio mapa.
        #   Habilita improved proposal (Grisetti) y soluciona la divergencia
        #   de scan-match. Justifica backend=gpu.
        ref_idx = int(np.argmax(self.weights))
        per_particle_dts = None
        ref_dt = None
        if self.per_particle_dt and finite.any():
            # Calcula N DTs en backend (CPU loop o GPU). Mantenemos en CPU
            # para indexar despues con NumPy (scan_match + weighting).
            per_particle_dts = [None] * self.N
            for i in range(self.N):
                occ_i = self.maps[i] > 0.5
                if occ_i.any():
                    dt_i = compute_distance_transform(occ_i, self.xp, self.dt_edt)
                    per_particle_dts[i] = to_numpy(dt_i)
            # Para los outputs que usan "el mejor mapa" mas abajo:
            ref_dt = per_particle_dts[ref_idx]
        else:
            ref_occ = self.maps[ref_idx] > 0.5
            if ref_occ.any() and finite.any():
                dt = compute_distance_transform(ref_occ, self.xp, self.dt_edt)
                ref_dt = to_numpy(dt)

        # Endpoints del scan en frame del robot para scan-matching.
        scan_xy_robot = None
        if self.use_match and finite.any():
            # Con per-particle DT activamos scan-match aun con mapa sparse;
            # la diversidad la preserva el per-particle (no colapsa al optimo
            # compartido). Con shared DT esperamos a tener mapa "maduro".
            ok_to_match = self.per_particle_dt or (
                ref_dt is not None
                and int((self.maps[ref_idx] > 0.5).sum()) >= self.match_min_occ
            )
            if ok_to_match:
                scan_xy_robot = scan_endpoints_robot_frame(
                    r_sub, a_sub, self.max_range, min_range=self.min_range
                )

        for i in range(self.N):
            px, py, pth = self.poses[i]
            # DT contra el que vamos a comparar esta particula:
            #   per-particle: su propia DT (None si su mapa esta vacio)
            #   shared: la DT compartida (la del best del paso anterior)
            dt_for_i = per_particle_dts[i] if self.per_particle_dt else ref_dt

            # 1) Scan-matching local contra dt_for_i
            if scan_xy_robot is not None and scan_xy_robot.shape[0] > 0 and dt_for_i is not None:
                dx, dy, dth = scan_match_local(
                    px, py, pth, scan_xy_robot, dt_for_i,
                    self.origin_x, self.origin_y, self.res,
                    win_xy=self.match_win_xy,
                    step_xy=self.match_step_xy,
                    win_th=self.match_win_th,
                    step_th=self.match_step_th,
                    reg_xy=self.match_reg_xy,
                    reg_th=self.match_reg_th,
                )
                px += dx
                py += dy
                pth = math.atan2(math.sin(pth + dth), math.cos(pth + dth))
                self.poses[i, 0] = px
                self.poses[i, 1] = py
                self.poses[i, 2] = pth

            # 2) Weighting contra dt_for_i
            if dt_for_i is not None:
                rr = r_sub[finite]
                aa = a_sub[finite]
                ex = px + rr * np.cos(pth + aa)
                ey = py + rr * np.sin(pth + aa)
                gx, gy = world_to_grid_vec(ex, ey,
                                           self.origin_x, self.origin_y, self.res)
                valid = (gx >= 0) & (gx < self.W) & (gy >= 0) & (gy < self.H)
                if valid.any():
                    d_m = dt_for_i[gy[valid], gx[valid]] * self.res
                    ll = -0.5 * (d_m / self.sigma_hit) ** 2
                    log_w[i] = float(ll.sum())

            # 3) Actualizar mapa propio de la particula.
            update_map_from_scan(self.maps[i], px, py, pth,
                                 r_full, a_full, self.max_range,
                                 self.origin_x, self.origin_y, self.res)

        # Normalizar pesos
        log_w -= log_w.max()
        w = np.exp(log_w) * self.weights
        s = w.sum()
        if s <= 1e-300:
            w = np.ones(self.N) / self.N
        else:
            w /= s
        self.weights = w

        # Snapshot stats antes de resamplear (las relevantes para diagnostico)
        self.last_n_eff_pre = n_eff(self.weights)
        self.last_spread_pre = float(np.linalg.norm(self.poses[:, :2].std(axis=0)))

        # Resample
        if self.last_n_eff_pre < self.N / 2.0:
            idx = systematic_resample(self.weights, self.rng)
            self.poses = self.poses[idx].copy()
            self.maps = self.maps[idx].copy()
            self.weights = np.ones(self.N, dtype=np.float64) / self.N
            self.resamples += 1

        self.updates += 1

    # ── Outputs ──────────────────────────────────────────────────────────────

    def _best_index(self):
        return int(np.argmax(self.weights))

    def publish_outputs(self):
        if self.last_odom_xyt is None:
            return
        b = self._best_index()
        px, py, pth = self.poses[b]
        now = self.get_clock().now().to_msg()

        # OccupancyGrid
        msg = OccupancyGrid()
        msg.header.stamp = now
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.res
        msg.info.width = self.W
        msg.info.height = self.H
        msg.info.origin = Pose()
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        data = logodds_to_occupancy(self.maps[b])
        msg.data = data.flatten().tolist()
        self.map_pub.publish(msg)

        # Belief pose
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = self.map_frame
        ps.pose.position.x = float(px)
        ps.pose.position.y = float(py)
        ps.pose.orientation = quaternion_from_yaw(float(pth))
        self.belief_pub.publish(ps)
        self.belief_path.poses.append(ps)
        if len(self.belief_path.poses) > 5000:
            self.belief_path.poses = self.belief_path.poses[-5000:]
        self.belief_path.header.stamp = now
        self.belief_path_pub.publish(self.belief_path)
        self.real_path.header.stamp = now
        self.real_path_pub.publish(self.real_path)

        # PoseArray con todas las particulas
        pa = PoseArray()
        pa.header.stamp = now
        pa.header.frame_id = self.map_frame
        for i in range(self.N):
            p = Pose()
            p.position.x = float(self.poses[i, 0])
            p.position.y = float(self.poses[i, 1])
            p.orientation = quaternion_from_yaw(float(self.poses[i, 2]))
            pa.poses.append(p)
        self.particles_pub.publish(pa)

        lo = self.last_odom_xyt or (0, 0, 0)
        self.get_logger().info(
            f'scans={self.scan_count} resamp={self.resamples} '
            f'n_eff_pre={self.last_n_eff_pre:.1f}/{self.N} '
            f'spread_pre={self.last_spread_pre*100:.1f}cm '
            f'belief=({px:+.2f},{py:+.2f},{math.degrees(pth):+.0f}°) '
            f'odom=({lo[0]:+.2f},{lo[1]:+.2f}) '
            f'occ={(data > 50).sum()} free={((data >= 0) & (data < 50)).sum()}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GridFastSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
