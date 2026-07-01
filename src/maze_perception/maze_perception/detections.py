"""Contrato de deteccion de conos (perception -> mission).

Se serializa como JSON sobre std_msgs/String, siguiendo el mismo patron que
/nav_debug en maze_nav (telemetria JSON). Esto evita definir mensajes .msg
custom (que requeririan un paquete ament_cmake + rosidl, problematico en macOS)
y mantiene tanto maze_perception como maze_mission como paquetes ament_python
puros. El esquema JSON es el contrato: cualquier cambio incrementa `version`.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import List, Optional

CONTRACT_VERSION = 1


@dataclass
class ConeDetection:
    bearing_rad: float   # angulo horizontal respecto al eje optico (+ a la izquierda)
    u: int               # centroide del blob, columna en pixeles
    v: int               # centroide del blob, fila en pixeles
    area_px: int         # area del blob en pixeles
    confidence: float    # 0..1
    color: str = 'red'


@dataclass
class ConeDetections:
    stamp_s: float                 # timestamp del frame (segundos)
    frame_id: str                  # frame optico de la camara
    image_width: int
    image_height: int
    detections: List[ConeDetection] = field(default_factory=list)
    version: int = CONTRACT_VERSION

    def to_json(self) -> str:
        return json.dumps({
            'version': self.version,
            'stamp_s': self.stamp_s,
            'frame_id': self.frame_id,
            'image_width': self.image_width,
            'image_height': self.image_height,
            'detections': [asdict(d) for d in self.detections],
        })

    @staticmethod
    def from_json(text: str) -> 'ConeDetections':
        raw = json.loads(text)
        dets = [
            ConeDetection(
                bearing_rad=float(d['bearing_rad']),
                u=int(d['u']),
                v=int(d['v']),
                area_px=int(d['area_px']),
                confidence=float(d['confidence']),
                color=str(d.get('color', 'red')),
            )
            for d in raw.get('detections', [])
        ]
        return ConeDetections(
            stamp_s=float(raw['stamp_s']),
            frame_id=str(raw['frame_id']),
            image_width=int(raw['image_width']),
            image_height=int(raw['image_height']),
            detections=dets,
            version=int(raw.get('version', CONTRACT_VERSION)),
        )

    def best(self) -> Optional[ConeDetection]:
        """Deteccion mas fuerte: mayor confianza, desempatando por area."""
        if not self.detections:
            return None
        return max(self.detections, key=lambda d: (d.confidence, d.area_px))
