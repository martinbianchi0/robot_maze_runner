"""Segmentacion de color rojo en espacio HSV.

El rojo esta en los extremos del canal Hue (wraparound en H=0/180), por eso se
usan DOS bandas (baja y alta) que se combinan con OR. Sin dependencias de ROS:
recibe y devuelve arrays de numpy, testeable con imagenes sinteticas o .png.
Los umbrales se calibran en la etapa C1 contra el rosbag laberinto_conos.
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class RedHSVThresholds:
    # Calibrado en C1 contra laberinto_conos (barrido de candidatos, ratio de
    # cluster LIDAR como metrica): el cono es rojo-naranja MUY saturado (S~207)
    # mientras los distractores (barreras beige/madera) son naranja poco saturado.
    # Dos hallazgos: (1) el piso de saturacion alto (160) es el discriminador clave
    # (0.74 -> 0.86 de precision); (2) el hue DEBE quedar angosto (rojo, 0-10 /
    # 170-180): ensancharlo a naranja mete objetos naranja saturados y arruina la
    # precision. S185 andaba marginalmente mejor pero recorta la cola de baja S del
    # cono; se elige S160 por margen ante variacion de iluminacion en el laboratorio.
    # Banda baja (rojo cerca de H=0)
    low1_h: int = 0
    low1_s: int = 160
    low1_v: int = 70
    high1_h: int = 10
    high1_s: int = 255
    high1_v: int = 255
    # Banda alta (rojo cerca de H=180)
    low2_h: int = 170
    low2_s: int = 160
    low2_v: int = 70
    high2_h: int = 180
    high2_s: int = 255
    high2_v: int = 255
    # Morfologia (0 desactiva)
    open_ksize: int = 3
    close_ksize: int = 5


def segment_red(bgr, thr: RedHSVThresholds = RedHSVThresholds()):
    """Devuelve una mascara binaria uint8 (0/255) del rojo en la imagen BGR."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lower1 = np.array([thr.low1_h, thr.low1_s, thr.low1_v], dtype=np.uint8)
    upper1 = np.array([thr.high1_h, thr.high1_s, thr.high1_v], dtype=np.uint8)
    lower2 = np.array([thr.low2_h, thr.low2_s, thr.low2_v], dtype=np.uint8)
    upper2 = np.array([thr.high2_h, thr.high2_s, thr.high2_v], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
    return _morph(mask, thr)


def _morph(mask, thr: RedHSVThresholds):
    if thr.open_ksize > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (thr.open_ksize, thr.open_ksize))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    if thr.close_ksize > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (thr.close_ksize, thr.close_ksize))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def thresholds_from_dict(values: dict) -> RedHSVThresholds:
    """Construye thresholds desde un dict (perfil YAML/params), con defaults.

    Solo se aceptan claves validas del dataclass; cualquier otra lanza TypeError,
    lo que sirve como validacion temprana del perfil.
    """
    valid = {f.name for f in dataclasses.fields(RedHSVThresholds)}
    unknown = set(values) - valid
    if unknown:
        raise TypeError(f'claves HSV desconocidas: {sorted(unknown)}')
    return dataclasses.replace(RedHSVThresholds(), **{k: int(v) for k, v in values.items()})
