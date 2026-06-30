"""Grid-Based FastSLAM (TP Final Parte A).

Cada particula mantiene su propia pose y un mapa de ocupacion en log-odds.
- Motion model: Probabilistic Robotics, sample_motion_model_odometry (Tabla 5.6).
- Measurement model: likelihood field sobre el mapa propio de cada particula.
- Resample: low-variance, cuando Neff < N/2.
- Mapa final: el de la particula con mayor peso.
"""

from dataclasses import dataclass
import math
import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False

from maze_slam._kernels import integrate_beams, HAS_NUMBA  # noqa: E402


@dataclass
class Particle:
    x: float
    y: float
    theta: float
    log_odds: np.ndarray
    weight: float = 1.0
    pid: int = 0   # id de linaje: el hijo hereda el del padre al remuestrear
    # cache del likelihood field (distancia a celda ocupada): recalcular el EDT por
    # particula en cada update es carisimo, asi que lo cacheamos y refrescamos cada
    # field_refresh updates (escalonado entre particulas).
    dist_field: object = None
    field_scan: int = -100000


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class FastSLAM:
    def __init__(
        self,
        n_particles=15,
        map_size=240,
        resolution=0.05,
        alpha=(0.04, 0.02, 0.05, 0.02),
        l_occ=0.85,
        l_free=-0.40,
        l_max=4.0,
        l_min=-2.0,
        occ_threshold=0.7,
        sigma_hit=0.08,
        scan_step_map=4,
        scan_step_weight=4,
        z_rand=0.05,
        sensor_x=0.0,
        sensor_y=0.0,
        sensor_yaw=0.0,
        use_scan_match=True,
        sm_iters=20,
        sm_lin0=0.05,
        sm_ang0=0.05,
        sm_max_ang=0.20,   # rad (~11deg): tope de correccion del scan-match por update
        sm_max_lin=0.20,   # m: idem lineal. Mas que esto = enganche ambiguo -> rechazar
        field_refresh=5,
    ):
        self.n = n_particles
        self.H = self.W = map_size
        self.res = resolution
        # Origen del mapa: lo centramos en la pose inicial del robot (0,0).
        self.origin = -map_size * resolution / 2.0
        self.alpha = alpha
        # Transformacion fija base_link -> sensor (LIDAR). En el TB4 el rplidar
        # esta montado rotado +90 deg y corrido -4cm en x. Si no lo aplicamos, el
        # scan se integra rotado y el mapa nunca cierra.
        self.sx = sensor_x
        self.sy = sensor_y
        self.syaw = sensor_yaw
        # Scan matching: refina la pose contra el mapa antes de pesar, corrigiendo la
        # deriva de odometria que engrosa/dobla paredes.
        self.use_scan_match = use_scan_match and _HAS_SCIPY
        self.sm_iters = sm_iters
        self.sm_lin0 = sm_lin0   # paso lineal inicial del hill-climb (m)
        self.sm_ang0 = sm_ang0   # paso angular inicial (rad)
        self.sm_max_ang = sm_max_ang
        self.sm_max_lin = sm_max_lin
        self.l_occ = l_occ
        self.l_free = l_free
        self.l_max = l_max
        self.l_min = l_min
        self.occ_threshold = occ_threshold  # umbral log-odds para "obstaculo"
        self.sigma_hit = sigma_hit
        self.scan_step_map = scan_step_map
        self.scan_step_weight = scan_step_weight
        self.z_rand = z_rand  # peso minimo por medicion para no colapsar a 0
        self.field_refresh = field_refresh  # cada cuantos updates recalcular el EDT
        self.scan_count = 0  # contador de updates (para escalonar el refresh)

        self.particles = [
            Particle(0.0, 0.0, 0.0, np.zeros((self.H, self.W), dtype=np.float32),
                     1.0 / n_particles, pid=i)
            for i in range(n_particles)
        ]
        self.last_odom = None  # (x, y, theta)
        self._warmup_jit()

    def _warmup_jit(self):
        """Fuerza la compilacion del kernel njit ahora (en el init del nodo), para que
        el PRIMER scan real no sufra el stall de compilacion (~0.25s) que dejaba al
        nodo atrasado y provocaba el error de TF al arranque."""
        dummy = np.zeros((4, 4), dtype=np.float32)
        idx = np.array([2], dtype=np.int32)
        integrate_beams(dummy, 1, 1, idx, idx,
                        self.l_free, self.l_occ, self.l_min, self.l_max, 4, 4)

    # ----- conversion mundo <-> grilla -----
    def world_to_grid(self, x, y):
        cx = int((x - self.origin) / self.res)
        cy = int((y - self.origin) / self.res)
        return cx, cy

    # ----- pose del LIDAR en mundo a partir de la pose (base) de la particula -----
    def lidar_pose(self, p):
        lx = p.x + self.sx * math.cos(p.theta) - self.sy * math.sin(p.theta)
        ly = p.y + self.sx * math.sin(p.theta) + self.sy * math.cos(p.theta)
        return lx, ly, p.theta + self.syaw

    # ----- odometria -> delta del modelo de movimiento -----
    def odom_delta(self, x, y, theta, min_trans=0.0, min_rot=0.0):
        """Devuelve (drot1, dtrans, drot2) cuando el movimiento acumulado supera
        los umbrales, o None si todavia no (sigue acumulando, no consume)."""
        if self.last_odom is None:
            self.last_odom = (x, y, theta)
            return None
        x0, y0, t0 = self.last_odom
        dx = x - x0
        dy = y - y0
        dtrans = math.hypot(dx, dy)
        dtheta = wrap_angle(theta - t0)
        # Umbral: no consumimos el delta hasta que el robot se movio lo suficiente.
        # Asi evitamos meter ruido del modelo de movimiento a 20 Hz (deriva rotacional).
        if dtrans < min_trans and abs(dtheta) < min_rot:
            return None
        drot1 = wrap_angle(math.atan2(dy, dx) - t0) if dtrans > 1e-3 else 0.0
        drot2 = wrap_angle(dtheta - drot1)
        self.last_odom = (x, y, theta)
        return (drot1, dtrans, drot2)

    # ----- sample_motion_model_odometry (Probabilistic Robotics, Tabla 5.6) -----
    def sample_motion(self, delta, rng):
        drot1, dtrans, drot2 = delta
        a1, a2, a3, a4 = self.alpha
        s1 = math.sqrt(a1 * drot1 * drot1 + a2 * dtrans * dtrans)
        s2 = math.sqrt(a3 * dtrans * dtrans + a4 * (drot1 * drot1 + drot2 * drot2))
        s3 = math.sqrt(a1 * drot2 * drot2 + a2 * dtrans * dtrans)
        for p in self.particles:
            d1 = drot1 - (rng.normal(0.0, s1) if s1 > 0 else 0.0)
            dt = dtrans - (rng.normal(0.0, s2) if s2 > 0 else 0.0)
            d2 = drot2 - (rng.normal(0.0, s3) if s3 > 0 else 0.0)
            p.x += dt * math.cos(p.theta + d1)
            p.y += dt * math.sin(p.theta + d1)
            p.theta = wrap_angle(p.theta + d1 + d2)

    # ----- scan matching (hill-climbing sobre el likelihood field) -----
    def scan_match(self, particle, rs, ang, dist=None):
        """Refina la pose de `particle` maximizando el calce del scan contra su
        mapa (likelihood field). Devuelve (dx, dy, dtheta) de correccion en mundo,
        o (0,0,0) si el mapa esta muy vacio. Hill-climbing con pasos decrecientes.
        `dist` opcional: campo de distancia ya calculado (se reusa el cacheado)."""
        if dist is None:
            occ = particle.log_odds > self.occ_threshold
            if int(occ.sum()) < 30:
                return 0.0, 0.0, 0.0
            dist = distance_transform_edt(~occ).astype(np.float32) * self.res
        inv2s2 = 1.0 / (2.0 * self.sigma_hit * self.sigma_hit)
        H, W, res, origin = self.H, self.W, self.res, self.origin

        ca = np.cos(ang)
        sa = np.sin(ang)

        def score(x, y, th):
            lx = x + self.sx * math.cos(th) - self.sy * math.sin(th)
            ly = y + self.sx * math.sin(th) + self.sy * math.cos(th)
            lth = th + self.syaw
            c, s = math.cos(lth), math.sin(lth)
            # endpoints (vectorizado)
            ex = lx + rs * (c * ca - s * sa)
            ey = ly + rs * (s * ca + c * sa)
            gx = ((ex - origin) / res).astype(np.int32)
            gy = ((ey - origin) / res).astype(np.int32)
            ok = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
            if not ok.any():
                return -1e9
            d = dist[gy[ok], gx[ok]]
            return float(np.exp(-d * d * inv2s2).sum())

        bx, by, bth = particle.x, particle.y, particle.theta
        best_s = score(bx, by, bth)
        lin, ang_step = self.sm_lin0, self.sm_ang0
        for _ in range(self.sm_iters):
            improved = False
            for dx, dy, dth in ((lin, 0, 0), (-lin, 0, 0), (0, lin, 0),
                                (0, -lin, 0), (0, 0, ang_step), (0, 0, -ang_step)):
                s = score(bx + dx, by + dy, bth + dth)
                if s > best_s:
                    best_s, bx, by, bth = s, bx + dx, by + dy, wrap_angle(bth + dth)
                    improved = True
            if not improved:
                lin *= 0.5
                ang_step *= 0.5
                if lin < res * 0.5:
                    break
        dx = bx - particle.x
        dy = by - particle.y
        dth = wrap_angle(bth - particle.theta)
        # El scan-match es un REFINADOR local, no un relocalizador. Si pide una
        # correccion grande, casi seguro se engancho en una alineacion ambigua
        # (p.ej. la sala simetrica rotada 90deg) -> la rechazamos. La odometria
        # manda en esos casos, asi el mapa no salta de orientacion.
        if abs(dth) > self.sm_max_ang or math.hypot(dx, dy) > self.sm_max_lin:
            return 0.0, 0.0, 0.0
        return dx, dy, dth

    def apply_correction(self, dx, dy, dth):
        for p in self.particles:
            p.x += dx
            p.y += dy
            p.theta = wrap_angle(p.theta + dth)

    # ----- un paso completo de SLAM (lo usan el nodo y el tuner) -----
    def step(self, delta, ranges, angle_min, angle_inc, range_max, rng):
        self.scan_count += 1
        if delta is not None:
            self.sample_motion(delta, rng)
        if self.use_scan_match:
            rs, ang = self._filter_beams(ranges, angle_min, angle_inc, range_max,
                                         self.scan_step_weight)
            if len(rs) > 0:
                best = self.best()
                # reusar el campo cacheado de la mejor particula (indice 0 a fines del refresh)
                dist = self._dist_field(best, 0)
                if dist is not None:
                    dx, dy, dth = self.scan_match(best, rs, ang, dist=dist)
                    self.apply_correction(dx, dy, dth)
        self.weigh(ranges, angle_min, angle_inc, range_max)
        if self.neff() < self.n / 2.0:
            self.resample(rng)
        self.integrate_scan(ranges, angle_min, angle_inc, range_max)

    # ----- preparar beams validos a partir de LaserScan -----
    def _filter_beams(self, ranges, angle_min, angle_inc, range_max, step):
        ranges = np.asarray(ranges, dtype=np.float32)
        idxs = np.arange(0, len(ranges), step)
        rs = ranges[idxs]
        ang = angle_min + idxs.astype(np.float32) * angle_inc
        valid = np.isfinite(rs) & (rs > 0.06) & (rs < min(range_max, 8.0))
        return rs[valid], ang[valid]

    # ----- endpoints del scan (vectorizado) en celdas de grilla -----
    def _endpoint_cells(self, p, rs, ca, sa):
        """Devuelve (gx, gy) int32 de las celdas endpoint para la pose de la particula."""
        lx, ly, lth = self.lidar_pose(p)
        c = math.cos(lth)
        s = math.sin(lth)
        ex = lx + rs * (c * ca - s * sa)
        ey = ly + rs * (s * ca + c * sa)
        gx = ((ex - self.origin) / self.res).astype(np.int32)
        gy = ((ey - self.origin) / self.res).astype(np.int32)
        return gx, gy

    # ----- likelihood field (distancia a ocupado) cacheado por particula -----
    def _dist_field(self, p, i):
        """EDT del mapa de la particula, con cache escalonado: solo se recalcula
        cada field_refresh updates (repartido entre particulas para no hacer 80 EDT
        de golpe). Devuelve None si el mapa esta vacio."""
        stale = (self.scan_count - p.field_scan) >= self.field_refresh
        # escalonar: en cada update solo refrescan ~N/field_refresh particulas
        due = ((self.scan_count + i) % self.field_refresh) == 0
        if p.dist_field is None or (stale and due):
            occ = p.log_odds > self.occ_threshold
            if not occ.any():
                return None
            # float16 para el cache: la mitad de RAM (80 particulas x 500x500 son
            # ~40MB en vez de 80MB). La precision alcanza de sobra para el likelihood.
            if _HAS_SCIPY:
                p.dist_field = (distance_transform_edt(~occ) * self.res).astype(np.float16)
            else:
                p.dist_field = np.where(occ, 0.0, 1.0).astype(np.float16)
            p.field_scan = self.scan_count
        return p.dist_field

    # ----- integrar scan al mapa de cada particula (bresenham njit) -----
    def integrate_scan(self, ranges, angle_min, angle_inc, range_max):
        rs, ang = self._filter_beams(ranges, angle_min, angle_inc, range_max, self.scan_step_map)
        if len(rs) == 0:
            return
        ca = np.cos(ang)
        sa = np.sin(ang)
        for p in self.particles:
            lx, ly, _ = self.lidar_pose(p)
            ox, oy = self.world_to_grid(lx, ly)
            if not (0 <= ox < self.W and 0 <= oy < self.H):
                continue
            gx, gy = self._endpoint_cells(p, rs, ca, sa)
            integrate_beams(p.log_odds, ox, oy, gx, gy,
                            self.l_free, self.l_occ, self.l_min, self.l_max,
                            self.W, self.H)

    # ----- ponderar particulas usando el likelihood field (vectorizado) -----
    def weigh(self, ranges, angle_min, angle_inc, range_max):
        rs, ang = self._filter_beams(ranges, angle_min, angle_inc, range_max, self.scan_step_weight)
        if len(rs) == 0:
            return
        ca = np.cos(ang)
        sa = np.sin(ang)
        inv2s2 = 1.0 / (2.0 * self.sigma_hit * self.sigma_hit)

        log_weights = np.zeros(self.n, dtype=np.float64)
        for i, p in enumerate(self.particles):
            dist = self._dist_field(p, i)
            if dist is None:
                log_weights[i] = 0.0   # mapa vacio: peso neutro
                continue
            gx, gy = self._endpoint_cells(p, rs, ca, sa)
            ok = (gx >= 0) & (gx < self.W) & (gy >= 0) & (gy < self.H)
            d = np.ones(len(rs), dtype=np.float32)  # fuera de mapa: penalizar (d=1)
            d[ok] = dist[gy[ok], gx[ok]]
            p_hit = np.exp(-d * d * inv2s2)
            log_weights[i] = np.log(self.z_rand + (1.0 - self.z_rand) * p_hit).sum()

        # normalizar en espacio log -> lineal
        log_weights -= log_weights.max()
        w = np.exp(log_weights)
        s = w.sum()
        if s <= 0 or not np.isfinite(s):
            w = np.full(self.n, 1.0 / self.n)
        else:
            w /= s
        for i, p in enumerate(self.particles):
            p.weight = float(w[i])

    # ----- Neff y resampling low-variance -----
    def neff(self):
        ws = np.array([p.weight for p in self.particles])
        s = (ws * ws).sum()
        return float(1.0 / s) if s > 0 else 1.0

    def resample(self, rng):
        ws = np.array([p.weight for p in self.particles])
        if ws.sum() <= 0:
            return
        ws = ws / ws.sum()
        r = rng.uniform(0.0, 1.0 / self.n)
        c = ws[0]
        i = 0
        new_particles = []
        for m in range(self.n):
            u = r + m * (1.0 / self.n)
            while u > c and i < self.n - 1:
                i += 1
                c += ws[i]
            src = self.particles[i]
            new_particles.append(
                Particle(src.x, src.y, src.theta, src.log_odds.copy(),
                         1.0 / self.n, pid=src.pid,          # hereda el linaje
                         dist_field=src.dist_field, field_scan=src.field_scan)  # cache
            )
        self.particles = new_particles

    def best(self):
        return max(self.particles, key=lambda p: p.weight)

    def best_sticky(self, prev_pid, ratio=0.5):
        """Devuelve la mejor particula con histeresis: si la del linaje `prev_pid`
        sigue viva y su peso es >= ratio * peso_max, la mantiene (evita que el mapa
        publicado salte entre hipotesis tick a tick). Si no, cambia a la de mayor peso."""
        top = self.best()
        if prev_pid is not None and top.weight > 0:
            same = [p for p in self.particles if p.pid == prev_pid]
            if same:
                keep = max(same, key=lambda p: p.weight)
                if keep.weight >= ratio * top.weight:
                    return keep
        return top

    # ----- mapa como OccupancyGrid msg (int8: -1 / 0 / 100) -----
    def occupancy_data(self, p):
        data = np.full((self.H, self.W), -1, dtype=np.int8)
        # log-odds > 0 = mas probable ocupado; usamos thresholds claros.
        data[p.log_odds > self.occ_threshold] = 100
        data[p.log_odds < (self.l_free * 0.5)] = 0
        return data
