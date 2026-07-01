#!/usr/bin/env python3
"""Captura online del debug de percepcion del cono (Parte C, etapa C2).

Nodo rclpy que se suscribe EN VIVO a los topics que publica cone_detector con
publish_debug: /cone_detections (String JSON), /cone_debug_image (anotada) y
/cone_mask. Mide la tasa de deteccion y guarda pares anotado+mascara como
evidencia. Valida el pipeline por ROS (no offline): QoS, publicacion de debug y
consumo en tiempo real. Corre por una duracion y termina.

Uso (con cone_detector + bag corriendo):
  python scripts/cone_debug_capture.py [duracion_s] [out_dir] [save_every]
"""
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'maze_perception'))

import cv2  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from maze_perception.detections import ConeDetections  # noqa: E402

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, 'results', 'parte_c', 'C2')
SAVE_EVERY = int(sys.argv[3]) if len(sys.argv) > 3 else 15


def to_bgr(msg):
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, np.uint8).reshape(h, msg.step)
    a = buf[:, :w * 3].reshape(h, w, 3)
    return np.ascontiguousarray(a[:, :, ::-1] if msg.encoding.lower() == 'rgb8' else a)


def to_gray(msg):
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, np.uint8).reshape(h, msg.step)
    return np.ascontiguousarray(buf[:, :w].reshape(h, w))


class Capture(Node):

    def __init__(self):
        super().__init__('cone_debug_capture')
        self.n_det = 0
        self.n_with_cone = 0
        self.n_img = 0
        self.saved = 0
        self.latest_mask = None
        # BEST_EFFORT: compatible con publisher reliable o best_effort (robusto).
        q = QoSProfile(depth=10)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(String, 'cone_detections', self.on_det, 10)
        self.create_subscription(Image, 'cone_debug_image', self.on_img, q)
        self.create_subscription(Image, 'cone_mask', self.on_mask, q)
        self.get_logger().info('cone_debug_capture escuchando /cone_detections, '
                               '/cone_debug_image, /cone_mask')

    def on_det(self, msg):
        self.n_det += 1
        try:
            if ConeDetections.from_json(msg.data).detections:
                self.n_with_cone += 1
        except Exception:  # noqa: BLE001
            pass

    def on_mask(self, msg):
        self.latest_mask = msg

    def on_img(self, msg):
        self.n_img += 1
        if self.n_img % SAVE_EVERY:
            return
        bgr = cv2.resize(to_bgr(msg), (400, 400), interpolation=cv2.INTER_NEAREST)
        cv2.imwrite(os.path.join(OUT, f'live_{self.saved:02d}_annot.png'), bgr)
        if self.latest_mask is not None:
            m = cv2.resize(to_gray(self.latest_mask), (400, 400), interpolation=cv2.INTER_NEAREST)
            cv2.imwrite(os.path.join(OUT, f'live_{self.saved:02d}_mask.png'), m)
        self.saved += 1


def main():
    os.makedirs(OUT, exist_ok=True)
    rclpy.init()
    node = Capture()
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < DURATION:
        rclpy.spin_once(node, timeout_sec=0.1)
    rate = (node.n_with_cone / node.n_det) if node.n_det else 0.0
    print(f'[capture] det_msgs={node.n_det} con_cono={node.n_with_cone} ({rate:.0%}) '
          f'debug_img={node.n_img} guardados={node.saved}')
    print(f'[capture] evidencia en {OUT}')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
