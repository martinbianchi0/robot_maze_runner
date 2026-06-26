"""Utilidades para Grid-Based FastSLAM (Parte A)."""

import math
import numpy as np
from geometry_msgs.msg import Quaternion


# ── Backend NumPy/CuPy (GPU opcional, fallback automatico a CPU) ─────────────

def get_backend(prefer='auto', device=None, mem_limit_gb=None):
    """Devuelve (xp, dt_edt, name).

    prefer:
      'auto' -> intenta CuPy, cae a NumPy si no esta o no hay GPU.
      'cpu'  -> NumPy / scipy siempre.
      'gpu'  -> CuPy, lanza si no se puede.
    device:   indice de GPU a usar (None = default).
    mem_limit_gb: limite de VRAM via memory pool (None = sin limite).
    """
    from scipy.ndimage import distance_transform_edt as _dt_cpu

    def cpu():
        return np, _dt_cpu, 'cpu'

    if prefer == 'cpu':
        return cpu()

    try:
        import os
        # Setear device via CUDA_VISIBLE_DEVICES funciona en todos los threads
        # (cp.cuda.Device(n).use() es per-thread, no se hereda en ROS spin).
        if device is not None and device >= 0:
            os.environ['CUDA_VISIBLE_DEVICES'] = str(device)
        import cupy as cp
        from cupyx.scipy.ndimage import distance_transform_edt as _dt_gpu
        if mem_limit_gb is not None and mem_limit_gb > 0:
            pool = cp.get_default_memory_pool()
            pool.set_limit(size=int(mem_limit_gb * 1024**3))
        # smoke test
        _ = cp.asarray([0.0, 1.0]).sum()
        cp.cuda.runtime.deviceSynchronize()
        return cp, _dt_gpu, 'gpu'
    except Exception as e:
        if prefer == 'gpu':
            raise RuntimeError(f'GPU pedida pero CuPy no disponible: {e}')
        return cpu()


def to_numpy(arr):
    """Convierte array (numpy o cupy) a numpy."""
    if hasattr(arr, 'get'):
        return arr.get()
    return np.asarray(arr)


def compute_distance_transform(occ_bool, xp, dt_edt):
    """Calcula DT del complemento de occ_bool (distancia a la celda ocupada
    mas cercana). Acepta occ_bool en CPU o GPU. Devuelve DT en el backend xp.

    Cast a uint8: cupyx.scipy.ndimage.distance_transform_edt usa _pba_2d que
    no acepta bool dtype directo.
    """
    if xp is np:
        return dt_edt(~occ_bool)
    # NumPy 2.x agrego .device='cpu' a sus arrays, asi que hasattr no sirve
    # para distinguir backend. Chequear el modulo del tipo es lo confiable.
    if type(occ_bool).__module__ != 'cupy':
        occ_bool = xp.asarray(occ_bool)
    inv = (~occ_bool).astype(xp.uint8)
    return dt_edt(inv)


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


# ── Scan-matching local (correlative grid search) ─────────────────────────────

def scan_match_local(px, py, pth, scan_xy_robot, dt_grid,
                     origin_x, origin_y, res,
                     win_xy=0.10, step_xy=0.02,
                     win_th=None, step_th=None,
                     reg_xy=10.0, reg_th=2000.0):
    """Busca (dx, dy, dth) en una ventana local que minimiza el costo del scan
    contra el likelihood field (distance transform en celdas).

    Implementa correlative scan-matching estilo gmapping/Grisetti: grid search
    determinístico chico alrededor de la pose actual. Vectorizado sobre los
    candidatos.

    Parametros:
      px, py, pth: pose actual de la particula
      scan_xy_robot: (n, 2) endpoints del scan en frame del robot
      dt_grid: (H, W) distance transform en celdas (no metros)
      origin_x, origin_y, res: parametros de la grilla
      win_xy: ventana en x,y (metros, default 10cm)
      step_xy: paso en x,y (metros, default 2cm)
      win_th: ventana en theta (rad, default 5°)
      step_th: paso en theta (rad, default 1°)
      reg_xy: penalizacion por (dx,dy) — actua como prior gaussiano (sigma~5cm).
              Mas alto = mas confianza en la pose original (motion model).
      reg_th: penalizacion por dth — separada porque la escala es distinta.
              Mas alto = el scan-matching no rota tanto, evita over-correction
              cuando hay walls solo en un lado del scan.

    Retorna: (best_dx, best_dy, best_dth).
    """
    if win_th is None:
        win_th = math.radians(5.0)
    if step_th is None:
        step_th = math.radians(1.0)
    if scan_xy_robot.shape[0] == 0:
        return 0.0, 0.0, 0.0

    H, W = dt_grid.shape
    dxs = np.arange(-win_xy, win_xy + step_xy * 0.5, step_xy, dtype=np.float64)
    dys = np.arange(-win_xy, win_xy + step_xy * 0.5, step_xy, dtype=np.float64)
    dths = np.arange(-win_th, win_th + step_th * 0.5, step_th, dtype=np.float64)
    Nx, Ny, Nth = len(dxs), len(dys), len(dths)

    rx = scan_xy_robot[:, 0]                    # (n,)
    ry = scan_xy_robot[:, 1]
    n = rx.shape[0]

    # Pre-rotamos por dth: (Nth, n)
    cth = np.cos(pth + dths)
    sth = np.sin(pth + dths)
    wx_rot = cth[:, None] * rx[None, :] - sth[:, None] * ry[None, :]
    wy_rot = sth[:, None] * rx[None, :] + cth[:, None] * ry[None, :]

    dt_max = float(dt_grid.max()) if dt_grid.size else 1.0
    inv_res = 1.0 / res

    # Iteramos por dth (mas barato que materializar (Nth*Nx*Ny, n))
    best_cost = np.inf
    best = (0.0, 0.0, 0.0)
    for k in range(Nth):
        # (Nx, Ny, n) endpoints en mundo para este dth
        # x = px + dx + wx_rot[k], y = py + dy + wy_rot[k]
        # Calculamos cells y los lookups vectorizados.
        x_base = px + wx_rot[k]                # (n,)
        y_base = py + wy_rot[k]
        # candidatos (Nx, Ny):
        # gx[i, j, b] = floor((x_base[b] + dxs[i] - ox) * inv_res)
        gx = np.floor((x_base[None, None, :] + dxs[:, None, None] - origin_x) * inv_res).astype(np.int32)
        gy = np.floor((y_base[None, None, :] + dys[None, :, None] - origin_y) * inv_res).astype(np.int32)
        in_bounds = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
        # Clamping para indexar; los OOB sumamos dt_max como penalizacion.
        gxc = np.clip(gx, 0, W - 1)
        gyc = np.clip(gy, 0, H - 1)
        d = dt_grid[gyc, gxc]                  # (Nx, Ny, n)
        oob = (~in_bounds).astype(np.float32) * dt_max
        cost_xy = (np.where(in_bounds, d, 0.0) + oob).sum(axis=2)  # (Nx, Ny)
        # Regularizacion separada para xy vs theta (escalas distintas):
        # actua como prior gaussiano debil centrado en (0,0,0).
        cost_xy = (cost_xy
                   + reg_xy * (dxs[:, None] ** 2 + dys[None, :] ** 2)
                   + reg_th * dths[k] ** 2)
        idx_flat = int(np.argmin(cost_xy))
        i = idx_flat // Ny
        j = idx_flat % Ny
        c = float(cost_xy[i, j])
        if c < best_cost:
            best_cost = c
            best = (float(dxs[i]), float(dys[j]), float(dths[k]))
    return best


def scan_endpoints_robot_frame(ranges, angles, max_range, min_range=0.0):
    """Convierte ranges en endpoints (x, y) en frame del robot, filtrando
    lecturas invalidas (inf, nan, fuera de rango).
    """
    r = np.asarray(ranges, dtype=np.float64)
    valid = np.isfinite(r) & (r > min_range) & (r < max_range)
    rv = r[valid]
    av = angles[valid]
    return np.stack([rv * np.cos(av), rv * np.sin(av)], axis=1)


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
