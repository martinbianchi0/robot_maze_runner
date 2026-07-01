"""Extraccion de blobs a partir de una mascara binaria.

Encuentra contornos, calcula centroide/area/bbox y filtra distractores por
tamano (area minima y maxima) y por densidad de relleno (descarta ruido
disperso). Sin dependencias de ROS: recibe una mascara numpy y devuelve una
lista de Blob ordenada por area descendente.
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from typing import List

import cv2


@dataclass(frozen=True)
class Blob:
    u: int          # centroide, columna en pixeles
    v: int          # centroide, fila en pixeles
    area_px: int
    x: int          # bbox: esquina superior izquierda x
    y: int          # bbox: esquina superior izquierda y
    w: int          # bbox: ancho
    h: int          # bbox: alto


@dataclass(frozen=True)
class BlobFilter:
    min_area_px: int = 20        # descarta ruido pequeno
    max_area_px: int = 40000     # descarta manchas enormes (fondo/pared roja)
    min_fill_ratio: float = 0.15  # area / (w*h); descarta detecciones dispersas


def extract_blobs(mask, flt: BlobFilter = BlobFilter()) -> List[Blob]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs: List[Blob] = []
    for contour in contours:
        area = int(cv2.contourArea(contour))
        if area < flt.min_area_px or area > flt.max_area_px:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if w * h == 0:
            continue
        if area / float(w * h) < flt.min_fill_ratio:
            continue
        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            continue
        u = int(round(moments['m10'] / moments['m00']))
        v = int(round(moments['m01'] / moments['m00']))
        blobs.append(Blob(u=u, v=v, area_px=area, x=x, y=y, w=w, h=h))
    blobs.sort(key=lambda b: b.area_px, reverse=True)
    return blobs


def filter_from_dict(values: dict) -> BlobFilter:
    """Construye un BlobFilter desde un dict (perfil YAML/params), con defaults."""
    valid = {f.name for f in dataclasses.fields(BlobFilter)}
    unknown = set(values) - valid
    if unknown:
        raise TypeError(f'claves de blob desconocidas: {sorted(unknown)}')
    return dataclasses.replace(BlobFilter(), **values)
