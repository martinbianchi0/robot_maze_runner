"""Utilidades para Grid-Based FastSLAM (Parte A)."""

import math
import numpy as np
from geometry_msgs.msg import Quaternion


# ── Trigonometria / quaterniones ──────────────────────────────────────────────

def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(q):
    # yaw = atan2(2(wz+xy), 1-2(y^2+z^2))
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# ── Grilla ─────────────────────────────────────────────────────────────────────

def world_to_grid(x, y, origin_x, origin_y, res):
    gx = int(math.floor((x - origin_x) / res))
    gy = int(math.floor((y - origin_y) / res))
    return gx, gy


def world_to_grid_vec(x, y, origin_x, origin_y, res):
    gx = np.floor((x - origin_x) / res).astype(np.int32)
    gy = np.floor((y - origin_y) / res).astype(np.int32)
    return gx, gy


# ── Sensor model (log-odds) ────────────────────────────────────────────────────

L_FREE = math.log(0.3 / 0.7)    # libre
L_OCC  = math.log(0.7 / 0.3)    # ocupado
L_CLIP = 5.0


def logodds_to_occupancy(L):
    """log-odds (HxW) -> OccupancyGrid data (0-100, -1 desconocido)."""
    out = np.full(L.shape, -1, dtype=np.int8)
    known = np.abs(L) > 1e-3
    p = 1.0 / (1.0 + np.exp(-L[known]))
    out[known] = np.clip(np.round(p * 100.0), 0, 100).astype(np.int8)
    return out


# ── Ray casting (Bresenham, vectorizado por scan) ─────────────────────────────

def scan_endpoints(scan_ranges, scan_angles, max_range, robot_x, robot_y, robot_yaw):
    """Devuelve (rx, ry, ex, ey, hit) en coordenadas mundo y mascara hit/miss.

    rx, ry: posicion del robot por beam (broadcast)
    ex, ey: endpoint del beam en mundo (si miss, en max_range)
    hit:    True si el beam termino en un obstaculo (< max_range)
    """
    r = np.asarray(scan_ranges, dtype=np.float64)
    finite = np.isfinite(r)
    hit = finite & (r > 0.0) & (r < max_range)
    r_used = np.where(hit, r, max_range)

    ang = robot_yaw + scan_angles
    ex = robot_x + r_used * np.cos(ang)
    ey = robot_y + r_used * np.sin(ang)
    return ex, ey, hit


def bresenham_line(x0, y0, x1, y1):
    """Devuelve las celdas (gx, gy) en orden desde (x0,y0) hasta (x1,y1)."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    n = dx + dy + 1
    out_x = np.empty(n, dtype=np.int32)
    out_y = np.empty(n, dtype=np.int32)
    cx, cy = x0, y0
    i = 0
    while True:
        out_x[i] = cx
        out_y[i] = cy
        i += 1
        if cx == x1 and cy == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            cx += sx
        if e2 < dx:
            err += dx
            cy += sy
    return out_x[:i], out_y[:i]


def update_map_from_scan(L, robot_x, robot_y, robot_yaw,
                        ranges, angles, max_range,
                        origin_x, origin_y, res,
                        l_free=L_FREE, l_occ=L_OCC, l_clip=L_CLIP):
    """Actualiza L (HxW log-odds) in-place con un LaserScan."""
    H, W = L.shape
    ex, ey, hit = scan_endpoints(ranges, angles, max_range,
                                 robot_x, robot_y, robot_yaw)
    rx0, ry0 = world_to_grid(robot_x, robot_y, origin_x, origin_y, res)
    if not (0 <= rx0 < W and 0 <= ry0 < H):
        return
    egx, egy = world_to_grid_vec(ex, ey, origin_x, origin_y, res)

    for i in range(len(ranges)):
        x1 = int(egx[i]); y1 = int(egy[i])
        if x1 < 0 or x1 >= W or y1 < 0 or y1 >= H:
            # clipear a borde con Bresenham igual; para simple, skip
            continue
        cx, cy = bresenham_line(rx0, ry0, x1, y1)
        # ultima celda es el endpoint
        L[cy[:-1], cx[:-1]] = np.clip(L[cy[:-1], cx[:-1]] + l_free,
                                       -l_clip, l_clip)
        if hit[i]:
            L[cy[-1], cx[-1]] = np.clip(L[cy[-1], cx[-1]] + l_occ,
                                         -l_clip, l_clip)
        else:
            L[cy[-1], cx[-1]] = np.clip(L[cy[-1], cx[-1]] + l_free,
                                         -l_clip, l_clip)


# ── Motion model (Thrun, deltas de odom) ──────────────────────────────────────

def odom_deltas(prev_xyt, curr_xyt):
    """Retorna (dr1, dt, dr2) del modelo de movimiento por deltas."""
    x0, y0, t0 = prev_xyt
    x1, y1, t1 = curr_xyt
    dx = x1 - x0
    dy = y1 - y0
    dt = math.hypot(dx, dy)
    if dt > 1e-6:
        dr1 = wrap_angle(math.atan2(dy, dx) - t0)
        dr2 = wrap_angle(t1 - t0 - dr1)
    else:
        dr1 = 0.0
        dr2 = wrap_angle(t1 - t0)
    return dr1, dt, dr2


def sample_motion(particles_xyt, dr1, dt, dr2,
                  alpha1=0.05, alpha2=0.02,
                  alpha3=0.05, alpha4=0.02,
                  rng=None):
    """particles_xyt (N,3) -> nueva pose ruidosa (N,3)."""
    if rng is None:
        rng = np.random.default_rng()
    n = particles_xyt.shape[0]
    eps = 1e-9
    s1 = math.sqrt(alpha1 * dr1 * dr1 + alpha2 * dt * dt) + eps
    st = math.sqrt(alpha3 * dt * dt + alpha4 * (dr1 * dr1 + dr2 * dr2)) + eps
    s2 = math.sqrt(alpha1 * dr2 * dr2 + alpha2 * dt * dt) + eps

    dr1n = dr1 + rng.normal(0.0, s1, n)
    dtn  = dt  + rng.normal(0.0, st, n)
    dr2n = dr2 + rng.normal(0.0, s2, n)

    new = particles_xyt.copy()
    th = new[:, 2]
    new[:, 0] += dtn * np.cos(th + dr1n)
    new[:, 1] += dtn * np.sin(th + dr1n)
    new[:, 2] = np.arctan2(np.sin(th + dr1n + dr2n),
                           np.cos(th + dr1n + dr2n))
    return new


# ── Resampling sistematico ─────────────────────────────────────────────────────

def systematic_resample(weights, rng=None):
    if rng is None:
        rng = np.random.default_rng()
    n = len(weights)
    positions = (np.arange(n) + rng.uniform()) / n
    cumsum = np.cumsum(weights)
    cumsum[-1] = 1.0  # safety
    return np.searchsorted(cumsum, positions)


def n_eff(weights):
    s = float(np.sum(weights * weights))
    return 1.0 / max(s, 1e-300)
