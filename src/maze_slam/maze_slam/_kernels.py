"""Kernels numericos calientes del FastSLAM, acelerados con Numba @njit.

Si numba no esta instalado, caen a una version Python equivalente (mas lenta pero
identica en resultado), asi el paquete corre igual en entornos sin numba.
Para instalar numba:  pip install numba   (en el container Humble: pip3 install numba)
"""
import numpy as np

try:
    from numba import njit, prange
    HAS_NUMBA = True
except Exception:  # noqa: BLE001  (numba ausente o roto)
    HAS_NUMBA = False

    def njit(*args, **kwargs):
        # Decorador no-op: si se usa como @njit o @njit(...), devuelve la func tal cual.
        if len(args) == 1 and callable(args[0]) and not kwargs:
            return args[0]

        def wrap(f):
            return f
        return wrap

    prange = range


@njit(cache=True, fastmath=True)
def integrate_beams(lo, ox, oy, ex_arr, ey_arr, l_free, l_occ, l_min, l_max, W, H):
    """Integra todos los beams de UN scan en el mapa log-odds `lo` de UNA particula.
    Bresenham por beam: marca libres a lo largo del rayo y ocupada la celda final.
    `ox,oy` = celda origen (LIDAR). `ex_arr,ey_arr` = celdas endpoint (int)."""
    n = ex_arr.shape[0]
    for k in range(n):
        x1 = ex_arr[k]
        y1 = ey_arr[k]
        dx = abs(x1 - ox)
        dy = abs(y1 - oy)
        sx = 1 if ox < x1 else -1
        sy = 1 if oy < y1 else -1
        err = dx - dy
        x = ox
        y = oy
        steps = 0
        max_steps = dx + dy + 1
        while steps < max_steps:
            if x == x1 and y == y1:
                break
            if 0 <= x < W and 0 <= y < H:
                v = lo[y, x] + l_free
                if v < l_min:
                    v = l_min
                lo[y, x] = v
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
            steps += 1
        # endpoint -> ocupado
        if 0 <= x1 < W and 0 <= y1 < H:
            v = lo[y1, x1] + l_occ
            if v > l_max:
                v = l_max
            lo[y1, x1] = v
