#!/usr/bin/env python3
"""
rock_detector.py
Detecta rocas por color (rojo/azul/verde) usando la cámara RGB de la AstraPro
y las localiza en el mapa usando la profundidad + TF del rover.

Publica:
  /rock_detector/detections  → String (JSON con datos de cada roca)
  /rock_detector/fin_detected → Bool (letrero FIN encontrado)
  /rock_detector/debug_image  → Image (frame anotado para depuración)
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import json
import tf2_ros
import tf2_geometry_msgs  # registra conversores para PointStamped, PoseStamped, etc.
import math

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge


class RockDetector(Node):

    # ── Rangos HSV para cada color ────────────────────────────────
    # Ajusta estos valores si los colores en simulación difieren
    COLOR_RANGES = {
        'rojo': [
            (np.array([0,   150,  100]), np.array([12,  255, 255])),
            (np.array([168, 150,  100]), np.array([180, 255, 255])),  # rojo envuelve 180
        ],
        'azul':  [(np.array([110, 150,  100]), np.array([130, 255, 255]))],
        'verde': [(np.array([ 45,  150,  100]), np.array([ 75, 255, 255]))],
    }

    # Área mínima de contorno para considerar una roca (píxeles²)
    MIN_CONTOUR_AREA = 200
    MAX_CONTOUR_AREA = 1000
    # Zonas prohibidas: (x, y, radio) de infraestructura que NO son rocas
    # Letrero FIN, contenedor azul, y parches verdes del suelo
    FORBIDDEN_ZONES = [
        (9.0, 5.0, 1.0),   # letrero FIN
        (1.0, 4.2, 1.0),   # contenedor azul (INICIO)
        (1.0, 5.0, 1.0),   # zona inicio (marcador visual en el suelo)
     ]

    # Profundidad máxima para detección (metros)
    MAX_DEPTH = 5.0

    def __init__(self):
        super().__init__('rock_detector')

        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_depth = None
        self.fin_detected = False
        self.detected_rocks = []  # lista de posiciones ya publicadas

        # TF para transformar coordenadas cámara → mapa
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Publishers ──────────────────────────────────────────
        self.rock_pub  = self.create_publisher(String, '/rock_detector/detections',  10)
        self.fin_pub   = self.create_publisher(Bool,   '/rock_detector/fin_detected', 10)
        self.debug_pub = self.create_publisher(Image,  '/rock_detector/debug_image',  10)

        # ── Subscribers ─────────────────────────────────────────
        self.create_subscription(Image,      '/rover/camera_rgb/image_raw',
                                 self._on_rgb,   10)
        self.create_subscription(Image,      '/rover/camera_depth/depth/image_raw',
                                 self._on_depth, 10)
        self.create_subscription(CameraInfo, '/rover/camera_rgb/camera_info',
                                 self._on_cam_info, 10)

        self.get_logger().info('🎥 Rock Detector iniciado')

    # ================================================================
    #  CALLBACKS
    # ================================================================
    def _on_cam_info(self, msg: CameraInfo):
        self.camera_info = msg

    def _on_depth(self, msg: Image):
        """Guarda el último frame de profundidad, detectando su encoding."""
        try:
            enc = msg.encoding
            if enc == '32FC1':
                depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            elif enc == '16UC1':
                # algunos plugins dan mm en uint16 → convertir a metros
                depth = self.bridge.imgmsg_to_cv2(msg, '16UC1').astype(np.float32) / 1000.0
            else:
                # encoding inesperado (p.ej. rgb8) → ignorar este mensaje
                return
            self.latest_depth = depth
        except Exception as e:
            self.get_logger().warn(f'Error leyendo depth: {e}')

    def _on_rgb(self, msg: Image):
        """Procesa cada frame RGB para detectar rocas y letrero FIN."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Error leyendo RGB: {e}')
            return

        debug_frame = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# ── CALIBRACIÓN TEMPORAL: imprime HSV del centro ─────────
#        h_img, w_img = hsv.shape[:2]
# Muestrea en la mitad inferior, donde aparecen las rocas
#        sample_x, sample_y = w_img // 2, int(h_img * 0.75)
#        sample_hsv = hsv[sample_y, sample_x]    

        # también dibuja una cruz en el centro del debug
#        cv2.drawMarker(debug_frame, (sample_x, sample_y), (255, 255, 255),
#                       cv2.MARKER_CROSS, 20, 2)
#        self.get_logger().info(
#            f'HSV centro: H={sample_hsv[0]} S={sample_hsv[1]} V={sample_hsv[2]}',
#            throttle_duration_sec=0.5)

        # ── Detección de rocas por color ─────────────────────────
        for color_name, ranges in self.COLOR_RANGES.items():
            # Crear máscara combinada para el color
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for (lo, hi) in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

            # Morfología para limpiar ruido
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if not (self.MIN_CONTOUR_AREA < area < self.MAX_CONTOUR_AREA):
                    continue

                #self.get_logger().info(
                #    f'[{color_name}] contorno área={area:.0f}px PASA filtro área',
                #    throttle_duration_sec=1.0)

                M = cv2.moments(cnt)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Centroide del contorno
                M = cv2.moments(cnt)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Profundidad en ese pixel (opcional)
                depth_m = self._get_depth(cx, cy)
                if depth_m is None:
                    # Fallback: estimar distancia por tamaño aparente del contorno
                    # Roca de ~3cm de ancho real; cuanto mayor el área, más cerca
                    depth_m = max(0.4, min(self.MAX_DEPTH, 300.0 / math.sqrt(area)))
                    used_depth_fallback = True
                else:
                    used_depth_fallback = False
                if depth_m > self.MAX_DEPTH:
                    continue

                # Estimar tamaño de la roca por área en imagen
                size_cm3 = self._estimate_size(area, depth_m)

                # Proyectar a coordenadas 3D en frame de cámara
                world_pos = self._pixel_to_world(cx, cy, depth_m, msg.header)
                if world_pos is None:
                    self.get_logger().warn(
                        f'[{color_name}] descartada: _pixel_to_world devolvió None '
                        f'(TF o camera_info faltante)',
                        throttle_duration_sec=1.0)
                    continue

                wx, wy = world_pos

                # Descartar detecciones fuera de la arena (10x10m con margen)
                if not (0.0 <= wx <= 10.0 and 0.0 <= wy <= 10.0):
                    continue

                # Descartar detecciones en zonas de infraestructura conocida
                if self._in_forbidden_zone(wx, wy):
                    continue

                # Verificar si ya fue detectada esta roca
                if self._already_detected(wx, wy):
                    continue

                # Publicar detección
                rock_data = {
                    'x': round(wx, 3),
                    'y': round(wy, 3),
                    'color': color_name,
                    'size_cm3': round(size_cm3, 1),
                    'weight_g': self._estimate_weight(size_cm3),
                    'depth_m': round(depth_m, 3),
                }
                self.rock_pub.publish(String(data=json.dumps(rock_data)))
                self.detected_rocks.append((wx, wy))
                self.get_logger().info(
                    f'Roca {color_name} detectada en ({wx:.2f}, {wy:.2f}) '
                    f'área={area:.0f}px depth={depth_m:.2f}m')

                # Dibujar en debug
                color_bgr = {'rojo':(0,0,255),'azul':(255,0,0),'verde':(0,255,0)}
                cv2.drawContours(debug_frame, [cnt], -1,
                                 color_bgr.get(color_name,(255,255,0)), 2)
                cv2.circle(debug_frame, (cx, cy), 5,
                           color_bgr.get(color_name,(255,255,0)), -1)
                label = f'{color_name} {size_cm3:.0f}cm3 {depth_m:.1f}m'
                cv2.putText(debug_frame, label, (cx-30, cy-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            color_bgr.get(color_name,(255,255,0)), 1)

        # ── Detección del letrero FIN ─────────────────────────────
        if not self.fin_detected:
            self._detect_fin_sign(frame, hsv, debug_frame, msg.header.stamp)

        # Publicar frame de depuración
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, 'bgr8')
            self.debug_pub.publish(debug_msg)
        except Exception:
            pass

    # ================================================================
    #  DETECCIÓN DEL LETRERO FIN
    # ================================================================
    def _detect_fin_sign(self, frame, hsv, debug_frame, stamp):
        """
        Detecta el letrero rojo 'FIN'.
        Busca una región rectangular roja grande en la imagen.
        """
        # Máscara roja grande (el letrero es más grande que las rocas)
        mask_r1 = cv2.inRange(hsv,
                              np.array([0,   150,  80]),
                              np.array([10,  255, 255]))
        mask_r2 = cv2.inRange(hsv,
                              np.array([165, 150,  80]),
                              np.array([180, 255, 255]))
        mask = cv2.bitwise_or(mask_r1, mask_r2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 2000:  # letrero es grande
                continue

            # Verificar que sea rectangular (aspecto ratio ~2:1)
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = w / h if h > 0 else 0
            if not (1.0 < aspect < 5.0):
                continue

            # ¡Letrero FIN encontrado!
            self.fin_detected = True
            self.fin_pub.publish(Bool(data=True))
            cv2.rectangle(debug_frame, (x, y), (x+w, y+h), (0, 0, 255), 3)
            cv2.putText(debug_frame, '!!! FIN DETECTADO !!!',
                        (x, y-10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 0, 255), 2)
            self.get_logger().info('🏁 Letrero FIN detectado por visión!')
            break

    # ================================================================
    #  UTILIDADES
    # ================================================================
    def _get_depth(self, cx: int, cy: int) -> float | None:
        """Obtiene profundidad en el pixel (cx, cy)."""
        if self.latest_depth is None:
            self.get_logger().warn('latest_depth es None (no llega depth)',
                                    throttle_duration_sec=2.0)
            return None
        h, w = self.latest_depth.shape
        if not (0 <= cy < h and 0 <= cx < w):
            return None
        y1, y2 = max(0, cy-2), min(h, cy+3)
        x1, x2 = max(0, cx-2), min(w, cx+3)
        patch = self.latest_depth[y1:y2, x1:x2]
        valid = patch[(patch > 0.05) & (patch < 10.0)]
        # DIAGNÓSTICO TEMPORAL
#        self.get_logger().info(
#            f'depth en ({cx},{cy}): patch_raw={patch.flatten()[:3]} '
#            f'min={np.nanmin(patch):.2f} max={np.nanmax(patch):.2f} '
#            f'validos={len(valid)}/{patch.size}',
#            throttle_duration_sec=1.0)
        if len(valid) == 0:
            return None
        return float(np.median(valid))

    def _pixel_to_world(self, cx, cy, depth_m, header):
        """Proyecta pixel + profundidad a coordenadas en frame 'map'."""
        if self.camera_info is None:
            self.get_logger().warn('camera_info aun no recibido',
                                    throttle_duration_sec=2.0)
            return None

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        ox = self.camera_info.k[2]
        oy = self.camera_info.k[5]

        x_cam = (cx - ox) * depth_m / fx
        y_cam = (cy - oy) * depth_m / fy
        z_cam = depth_m

        try:
            point_cam = PointStamped()
            point_cam.header.frame_id = 'camera_optical_link'
            point_cam.header.stamp = rclpy.time.Time().to_msg()
            point_cam.point.x = x_cam
            point_cam.point.y = y_cam
            point_cam.point.z = z_cam

            point_map = self.tf_buffer.transform(
                point_cam, 'map',
                timeout=rclpy.duration.Duration(seconds=0.3))
            return (point_map.point.x, point_map.point.y)
        except Exception as e:
            self.get_logger().warn(f'TF fallo: {e}', throttle_duration_sec=2.0)
            return None

    def _already_detected(self, x, y, threshold=0.5):
        """Verifica si ya hay una roca detectada cerca."""
        for (dx, dy) in self.detected_rocks:
            if math.dist((x, y), (dx, dy)) < threshold:
                return True
        return False

    def _in_forbidden_zone(self, x, y):
        """True si (x,y) está cerca de infraestructura conocida (letrero, contenedor)."""
        for (fx, fy, radius) in self.FORBIDDEN_ZONES:
            if math.dist((x, y), (fx, fy)) < radius:
                return True
        return False

    def _estimate_size(self, pixel_area: float, depth_m: float) -> float:
        """Estima tamaño de roca en cm³ por área aparente y profundidad."""
        # Área real ≈ pixel_area * (depth/focal)²
        # Mapeo empírico a los tamaños del reglamento: 5, 7, 10, 12 cm³
        real_area_cm2 = pixel_area * (depth_m ** 2) * 0.001
        if real_area_cm2 < 2.0:   return 5.0
        elif real_area_cm2 < 4.0: return 7.0
        elif real_area_cm2 < 7.0: return 10.0
        else:                     return 12.0

    def _estimate_weight(self, size_cm3: float) -> float:
        """Estima peso en gramos (rango 20-50g del reglamento)."""
        weight_map = {5.0: 22.0, 7.0: 28.0, 10.0: 38.0, 12.0: 48.0}
        return weight_map.get(size_cm3, 30.0)


def main(args=None):
    rclpy.init(args=args)
    node = RockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
