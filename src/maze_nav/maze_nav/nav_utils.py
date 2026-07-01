"""Utilidades compartidas de la Parte B (navegacion).

Carga de mapas .pgm/.yaml (formato map_server), conversiones grilla<->mundo,
modelo de movimiento por odometria (Probabilistic Robotics, Tabla 5.6) y helpers
geometricos. Sin dependencias de ROS: se puede testear en aislado.
"""
import math
import os
import numpy as np


def wrap_angle(a):
    """Normaliza un angulo a (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def angle_mean(angles, weights=None):
    """Media circular de angulos (evita el problema del wrap en +/-pi)."""
    s = np.sin(angles)
    c = np.cos(angles)
    if weights is not None:
        s = np.sum(s * weights)
        c = np.sum(c * weights)
    else:
        s = np.mean(s)
        c = np.mean(c)
    return math.atan2(s, c)


# --------------------------------------------------------------------------- #
#   Carga de mapas (formato map_server: .yaml + .pgm P5)                        #
# --------------------------------------------------------------------------- #
def _parse_yaml(path):
    """Parser minimo del yaml de map_server (evita dependencia de pyyaml)."""
    meta = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or ':' not in line:
                continue
            key, val = line.split(':', 1)
            key = key.strip()
            val = val.strip()
            if val.startswith('['):
                val = [float(x) for x in val.strip('[]').split(',')]
            else:
                try:
                    val = float(val)
                except ValueError:
                    pass
            meta[key] = val
    return meta


def load_pgm(path):
    """Lee un PGM binario (P5). Devuelve array HxW uint8."""
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5', 'solo se soporta PGM binario (P5)'
        # saltear comentarios
        dims = f.readline()
        while dims.startswith(b'#'):
            dims = f.readline()
        w, h = map(int, dims.split())
        maxval = int(f.readline())
        data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    return data


def load_map(yaml_path):
    """Carga un mapa map_server. Devuelve un dict con:
        occ:   grilla HxW en {-1 desconocido, 0 libre, 100 ocupado}, y creciente hacia arriba
        res:   resolucion (m/celda)
        origin: (ox, oy) esquina inferior-izquierda en el frame 'map'
        H, W
    """
    meta = _parse_yaml(yaml_path)
    img_name = meta['image']
    if not os.path.isabs(img_name):
        img_name = os.path.join(os.path.dirname(yaml_path), img_name)
    img = load_pgm(img_name)
    res = float(meta['resolution'])
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    negate = int(meta.get('negate', 0))
    occ_th = float(meta.get('occupied_thresh', 0.65))
    free_th = float(meta.get('free_thresh', 0.25))

    # map_server: p = (255 - px) / 255 (o al reves si negate). p alto => ocupado.
    px = img.astype(np.float32)
    if negate:
        p = px / 255.0
    else:
        p = (255.0 - px) / 255.0

    occ = np.full(img.shape, -1, dtype=np.int8)
    occ[p > occ_th] = 100
    occ[p < free_th] = 0
    # el pgm viene con y hacia abajo (fila 0 = arriba); en 'map' y crece hacia arriba
    occ = np.flipud(occ)
    return {
        'occ': occ,
        'res': res,
        'origin': (float(origin[0]), float(origin[1])),
        'H': occ.shape[0],
        'W': occ.shape[1],
    }


# --------------------------------------------------------------------------- #
#   Conversiones grilla <-> mundo                                              #
# --------------------------------------------------------------------------- #
def world_to_grid(x, y, origin, res):
    gx = int((x - origin[0]) / res)
    gy = int((y - origin[1]) / res)
    return gx, gy


def grid_to_world(gx, gy, origin, res):
    x = origin[0] + (gx + 0.5) * res
    y = origin[1] + (gy + 0.5) * res
    return x, y


# --------------------------------------------------------------------------- #
#   Modelo de movimiento por odometria (muestreo)                              #
# --------------------------------------------------------------------------- #
def odom_deltas(p_prev, p_cur):
    """Descompone el movimiento entre dos poses de odometria en (rot1, trans, rot2)."""
    dx = p_cur[0] - p_prev[0]
    dy = p_cur[1] - p_prev[1]
    trans = math.hypot(dx, dy)
    if trans < 1e-4:
        rot1 = 0.0
    else:
        rot1 = wrap_angle(math.atan2(dy, dx) - p_prev[2])
    rot2 = wrap_angle(p_cur[2] - p_prev[2] - rot1)
    return rot1, trans, rot2


def sample_motion(particles, deltas, alphas, rng):
    """Aplica sample_motion_model_odometry a un array de particulas Nx3 in-place.

    particles: (N,3) [x,y,theta]. deltas: (rot1,trans,rot2). alphas: (a1,a2,a3,a4).
    """
    rot1, trans, rot2 = deltas
    a1, a2, a3, a4 = alphas
    n = particles.shape[0]
    sr1 = math.sqrt(a1 * rot1 * rot1 + a2 * trans * trans)
    st = math.sqrt(a3 * trans * trans + a4 * (rot1 * rot1 + rot2 * rot2))
    sr2 = math.sqrt(a1 * rot2 * rot2 + a2 * trans * trans)
    r1 = rot1 - rng.normal(0.0, sr1, n) if sr1 > 0 else np.full(n, rot1)
    tr = trans - rng.normal(0.0, st, n) if st > 0 else np.full(n, trans)
    r2 = rot2 - rng.normal(0.0, sr2, n) if sr2 > 0 else np.full(n, rot2)
    th = particles[:, 2]
    particles[:, 0] += tr * np.cos(th + r1)
    particles[:, 1] += tr * np.sin(th + r1)
    particles[:, 2] = (th + r1 + r2 + math.pi) % (2.0 * math.pi) - math.pi
    return particles
