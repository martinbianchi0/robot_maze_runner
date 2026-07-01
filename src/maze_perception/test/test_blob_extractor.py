"""Tests de la extraccion y filtrado de blobs."""
import numpy as np
import pytest

from maze_perception.blob_extractor import BlobFilter, extract_blobs, filter_from_dict


def _mascara_dos_cuadrados():
    mask = np.zeros((100, 100), dtype=np.uint8)
    mask[10:30, 10:30] = 255   # cuadrado chico
    mask[50:90, 50:90] = 255   # cuadrado grande
    return mask


def test_blobs_ordenados_por_area_desc():
    blobs = extract_blobs(_mascara_dos_cuadrados())
    assert len(blobs) == 2
    assert blobs[0].area_px > blobs[1].area_px
    assert abs(blobs[0].u - 69) <= 2
    assert abs(blobs[0].v - 69) <= 2


def test_area_minima_descarta_ruido():
    mask = np.zeros((50, 50), dtype=np.uint8)
    mask[5:8, 5:8] = 255   # 3x3, area muy chica
    assert extract_blobs(mask, BlobFilter(min_area_px=20)) == []


def test_area_maxima_descarta_manchas_enormes():
    mask = np.full((100, 100), 255, dtype=np.uint8)
    assert extract_blobs(mask, BlobFilter(max_area_px=1000)) == []


def test_filter_from_dict_override_y_validacion():
    flt = filter_from_dict({'min_area_px': 50})
    assert flt.min_area_px == 50
    with pytest.raises(TypeError):
        filter_from_dict({'clave_invalida': 1})
