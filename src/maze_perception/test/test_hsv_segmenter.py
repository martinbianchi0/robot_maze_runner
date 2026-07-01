"""Tests de la segmentacion HSV de rojo."""
import cv2
import numpy as np
import pytest

from maze_perception.hsv_segmenter import RedHSVThresholds, segment_red, thresholds_from_dict


def _img_rojo_y_azul():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    img[10:50, 10:50] = (0, 0, 255)   # rojo (BGR)
    img[10:50, 60:90] = (255, 0, 0)   # azul (BGR), distractor
    return img


def test_detecta_rojo_ignora_azul():
    mask = segment_red(_img_rojo_y_azul())
    assert mask.dtype == np.uint8
    assert mask.shape == (100, 100)
    assert mask[30, 30] == 255   # region roja marcada
    assert mask[30, 75] == 0     # region azul no marcada


def test_banda_alta_de_hue():
    # Rojo del extremo alto del Hue (H~178): se construye en HSV y se pasa a BGR.
    hsv = np.zeros((20, 20, 3), dtype=np.uint8)
    hsv[:, :] = (178, 200, 200)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    assert (segment_red(bgr) == 255).any()


def test_thresholds_from_dict_override_y_validacion():
    thr = thresholds_from_dict({'low1_s': 100})
    assert thr.low1_s == 100
    assert thr.high1_h == RedHSVThresholds().high1_h
    with pytest.raises(TypeError):
        thresholds_from_dict({'clave_invalida': 1})
