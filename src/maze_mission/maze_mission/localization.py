"""Criterio de convergencia de la localizacion MCL (Parte C, estado LOCALIZE).

La mision no debe navegar hasta que la MCL haya convergido: un goal validado en
frame map con una pose equivocada puede mandar el robot contra una pared. El
criterio es el spread de la nube de particulas (/particlecloud):

- spread xy: RMS de la distancia de cada particula al centroide (m).
- spread yaw: desviacion estandar circular, sqrt(-2 ln R) con R el modulo de la
  resultante media de los yaws (rad). Robusta a la periodicidad (una nube
  bimodal 0/pi da spread grande aunque la media aritmetica enganche).

Modulo puro (sin ROS) para poder testearlo offline.
"""
from __future__ import annotations

import math
from typing import Sequence, Tuple


def cloud_spread(xs: Sequence[float], ys: Sequence[float],
                 yaws: Sequence[float]) -> Tuple[float, float]:
    """Devuelve (spread_xy_m, spread_yaw_rad) de una nube de particulas.

    Nube vacia -> (inf, inf): nunca se considera convergida.
    """
    n = len(xs)
    if n == 0 or len(ys) != n or len(yaws) != n:
        return math.inf, math.inf
    mx = sum(xs) / n
    my = sum(ys) / n
    var = sum((x - mx) ** 2 + (y - my) ** 2 for x, y in zip(xs, ys)) / n
    spread_xy = math.sqrt(var)
    cr = sum(math.cos(a) for a in yaws) / n
    sr = sum(math.sin(a) for a in yaws) / n
    r = math.hypot(cr, sr)
    if r <= 1e-9:
        return spread_xy, math.inf
    spread_yaw = math.sqrt(max(0.0, -2.0 * math.log(min(1.0, r))))
    return spread_xy, spread_yaw


def is_converged(spread_xy: float, spread_yaw: float,
                 xy_max: float, yaw_max: float) -> bool:
    return spread_xy <= xy_max and spread_yaw <= yaw_max
