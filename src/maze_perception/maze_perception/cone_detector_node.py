"""Nodo de deteccion de cono rojo.

Suscribe la imagen de la camara (sensor_msgs/Image, bgr8 250x250 en el
TurtleBot4) y camera_info, segmenta el rojo en HSV, extrae blobs, filtra
distractores por tamano y publica las detecciones como JSON (contrato en
detections.py) sobre std_msgs/String. Opcionalmente publica imagen anotada y
mascara para debug en RViz/rqt.

Solo produce informacion en el plano imagen (bearing + pixel + area): NO
estima posiciones de mundo. La geometria (fusion con LIDAR, TF, validacion
contra el mapa) vive en maze_mission. Asi la percepcion queda desacoplada del
mapa y es reutilizable/testeable de forma aislada.

Todos los topicos y el QoS de la imagen son parametros: no se hardcodea nada,
para poder correr en simulacion, contra rosbags (/tb4_0/...) y en el robot real.
"""
from __future__ import annotations

import dataclasses
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from maze_perception.blob_extractor import BlobFilter, extract_blobs
from maze_perception.detections import ConeDetection, ConeDetections
from maze_perception.hsv_segmenter import RedHSVThresholds, segment_red


def image_to_bgr(msg: Image) -> np.ndarray:
    """Convierte sensor_msgs/Image a un array BGR uint8 (HxWx3), respetando step.

    Evita la dependencia de cv_bridge; maneja bgr8/rgb8/mono8.
    """
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)
    enc = msg.encoding.lower()
    if enc == 'bgr8':
        img = buf[:, : w * 3].reshape(h, w, 3)
    elif enc == 'rgb8':
        img = buf[:, : w * 3].reshape(h, w, 3)[:, :, ::-1]
    elif enc == 'mono8':
        gray = buf[:, :w].reshape(h, w)
        img = np.repeat(gray[:, :, None], 3, axis=2)
    else:
        raise ValueError(f'encoding no soportado: {msg.encoding}')
    return np.ascontiguousarray(img)


def bgr_to_image_msg(bgr: np.ndarray, header, encoding: str = 'bgr8') -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = int(bgr.shape[0]), int(bgr.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    channels = 1 if encoding == 'mono8' else 3
    msg.step = msg.width * channels
    msg.data = bgr.tobytes()
    return msg


class ConeDetectorNode(Node):

    def __init__(self):
        super().__init__('cone_detector')

        # Topicos (parametrizables: sim / bag / real).
        self.image_topic = self.declare_parameter(
            'image_topic', '/oakd/rgb/preview/image_raw').value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/oakd/rgb/preview/camera_info').value
        self.detections_topic = self.declare_parameter(
            'detections_topic', 'cone_detections').value
        self.debug_image_topic = self.declare_parameter(
            'debug_image_topic', 'cone_debug_image').value
        self.mask_topic = self.declare_parameter('mask_topic', 'cone_mask').value

        # QoS de la imagen parametrizable (best_effort en bag/real, reliable en sim).
        image_qos_str = self.declare_parameter('image_qos', 'best_effort').value
        self.publish_debug = self.declare_parameter('publish_debug', False).value
        self.fallback_hfov_deg = self.declare_parameter('fallback_hfov_deg', 69.0).value
        self.conf_full_area = float(
            self.declare_parameter('confidence_full_area_px', 2000).value)

        # Umbrales HSV y filtro de blobs desde parametros (namespaced con puntos).
        self.thr = self._declare_thresholds()
        self.blob_filter = self._declare_blob_filter()

        image_qos = self._sensor_qos(image_qos_str)
        info_qos = self._sensor_qos(image_qos_str)

        self.cam_fx = None
        self.cam_cx = None

        self.det_pub = self.create_publisher(String, self.detections_topic, 10)
        self.debug_pub = None
        self.mask_pub = None
        if self.publish_debug:
            self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 1)
            self.mask_pub = self.create_publisher(Image, self.mask_topic, 1)

        self.create_subscription(Image, self.image_topic, self._on_image, image_qos)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, info_qos)

        self.get_logger().info(
            f'cone_detector escuchando imagen={self.image_topic} '
            f'info={self.camera_info_topic} qos={image_qos_str} debug={self.publish_debug}')

    def _sensor_qos(self, reliability: str) -> QoSProfile:
        qos = QoSProfile(depth=10)
        qos.reliability = (
            ReliabilityPolicy.BEST_EFFORT
            if str(reliability).lower() == 'best_effort'
            else ReliabilityPolicy.RELIABLE
        )
        return qos

    def _declare_thresholds(self) -> RedHSVThresholds:
        defaults = RedHSVThresholds()
        values = {}
        for field in dataclasses.fields(RedHSVThresholds):
            default = getattr(defaults, field.name)
            values[field.name] = self.declare_parameter(f'hsv.{field.name}', default).value
        return RedHSVThresholds(**values)

    def _declare_blob_filter(self) -> BlobFilter:
        defaults = BlobFilter()
        values = {}
        for field in dataclasses.fields(BlobFilter):
            default = getattr(defaults, field.name)
            values[field.name] = self.declare_parameter(f'blob.{field.name}', default).value
        return BlobFilter(**values)

    def _on_camera_info(self, msg: CameraInfo):
        if len(msg.k) >= 3 and msg.k[0] > 0.0:
            self.cam_fx = float(msg.k[0])
            self.cam_cx = float(msg.k[2])

    def _fx_cx(self, width: int):
        if self.cam_fx is not None and self.cam_cx is not None:
            return self.cam_fx, self.cam_cx
        cx = width / 2.0
        fx = (width / 2.0) / math.tan(math.radians(self.fallback_hfov_deg) / 2.0)
        return fx, cx

    def _on_image(self, msg: Image):
        try:
            bgr = image_to_bgr(msg)
        except Exception as exc:  # noqa: BLE001 - un frame malo no debe tirar el nodo
            self.get_logger().warn(f'no pude convertir la imagen: {exc}')
            return

        height, width = bgr.shape[:2]
        mask = segment_red(bgr, self.thr)
        blobs = extract_blobs(mask, self.blob_filter)
        fx, cx = self._fx_cx(width)

        detections = []
        for blob in blobs:
            bearing = math.atan2(cx - blob.u, fx)
            confidence = max(0.0, min(1.0, blob.area_px / self.conf_full_area))
            detections.append(ConeDetection(
                bearing_rad=bearing, u=blob.u, v=blob.v,
                area_px=blob.area_px, confidence=confidence, color='red'))

        stamp = msg.header.stamp
        out = ConeDetections(
            stamp_s=stamp.sec + stamp.nanosec * 1e-9,
            frame_id=msg.header.frame_id or 'camera',
            image_width=width, image_height=height, detections=detections)
        self.det_pub.publish(String(data=out.to_json()))

        if self.publish_debug:
            self._publish_debug(bgr, mask, blobs, detections, msg.header)

    def _publish_debug(self, bgr, mask, blobs, detections, header):
        import cv2
        annotated = bgr.copy()
        # blobs[i] corresponde a detections[i]; el [0] es el mas grande (elegido).
        for i, (blob, det) in enumerate(zip(blobs, detections)):
            color = (0, 255, 0) if i == 0 else (0, 170, 170)
            thick = 2 if i == 0 else 1
            cv2.rectangle(annotated, (blob.x, blob.y),
                          (blob.x + blob.w, blob.y + blob.h), color, thick)
            cv2.circle(annotated, (det.u, det.v), 3, color, -1)
        if detections:
            d = detections[0]
            cv2.putText(
                annotated, f'{math.degrees(d.bearing_rad):+.0f}deg a={d.area_px} c={d.confidence:.2f}',
                (4, annotated.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        self.debug_pub.publish(bgr_to_image_msg(annotated, header, 'bgr8'))
        self.mask_pub.publish(bgr_to_image_msg(mask, header, 'mono8'))


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
