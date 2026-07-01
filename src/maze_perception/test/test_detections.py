"""Tests del contrato de detecciones (JSON perception -> mission)."""
from maze_perception.detections import ConeDetection, ConeDetections


def test_json_roundtrip_preserva_datos():
    original = ConeDetections(
        stamp_s=1.5, frame_id='cam', image_width=250, image_height=250,
        detections=[
            ConeDetection(0.1, 100, 120, 500, 0.8),
            ConeDetection(-0.2, 50, 130, 900, 0.9),
        ])
    back = ConeDetections.from_json(original.to_json())
    assert back.stamp_s == 1.5
    assert back.frame_id == 'cam'
    assert back.image_width == 250
    assert len(back.detections) == 2
    assert back.detections[0].u == 100
    assert back.detections[1].confidence == 0.9
    assert back.version == original.version


def test_best_elige_mayor_confianza():
    dets = ConeDetections(0.0, 'c', 10, 10, [
        ConeDetection(0.0, 1, 1, 100, 0.5),
        ConeDetection(0.0, 2, 2, 100, 0.9),
    ])
    assert dets.best().u == 2


def test_best_none_si_vacio():
    assert ConeDetections(0.0, 'c', 10, 10, []).best() is None
