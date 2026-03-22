#!/usr/bin/env python3
"""
Fake Depth Publisher — Smart Trolley V5 Simulation
===================================================
Publica un depth image sintético en /kinect/depth/image_raw que simula
una "persona" (blob gaussiano) a distancia y posición configurables.

Controles (ventana OpenCV):
  W / S   → persona más cerca / más lejos
  A / D   → persona a la izquierda / derecha
  Q       → salir

El nodo person_follower.py consume este topic exactamente igual que
el Kinect real: no requiere ningún cambio en el follower.

Uso:
  ros2 run smart_t_ai_v2 fake_depth_publisher
  ros2 run smart_t_ai_v2 fake_depth_publisher --ros-args -p init_distance:=1.5
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
import numpy as np
import threading
import time


class FakeDepthPublisher(Node):

    # ── Límites de la "persona" ─────────────────────────────────────────────
    MIN_DIST  = 0.4   # m — mínima distancia (frena si está muy cerca)
    MAX_DIST  = 4.0   # m — máxima distancia válida para el follower
    STEP_DIST = 0.05  # m por tecla
    STEP_LAT  = 0.03  # fracción de ancho por tecla (-1 … +1)

    IMG_W = 640
    IMG_H = 480

    # Tamaño de la persona en la imagen (blob gaussiano)
    BLOB_W = 120   # píxeles
    BLOB_H = 180   # píxeles

    def __init__(self):
        super().__init__('fake_depth_publisher')

        # ── Parámetros ──────────────────────────────────────────────────────
        self.declare_parameter('depth_topic',    '/kinect/depth/image_raw')
        self.declare_parameter('publish_rate',   15.0)
        self.declare_parameter('init_distance',  1.5)    # metros
        self.declare_parameter('init_lateral',   0.0)    # -1 … +1 (izq … der)
        self.declare_parameter('show_preview',   True)

        topic       = self.get_parameter('depth_topic').value
        rate        = self.get_parameter('publish_rate').value
        self._dist  = float(self.get_parameter('init_distance').value)
        self._lat   = float(self.get_parameter('init_lateral').value)
        self._preview = self.get_parameter('show_preview').value

        # ── Publisher ────────────────────────────────────────────────────────
        self._pub = self.create_publisher(Image, topic, 10)
        self._timer = self.create_timer(1.0 / rate, self._publish_cb)

        # ── Precompute gaussian kernels ──────────────────────────────────────
        self._gauss_x = self._make_gauss(self.BLOB_W)
        self._gauss_y = self._make_gauss(self.BLOB_H)

        self.get_logger().info(
            f'📷 Fake Depth Publisher activo → {topic}  |  '
            f'dist={self._dist:.2f}m  lat={self._lat:+.2f}')
        self.get_logger().info(
            '   Controles: W/S=distancia  A/D=lateral  Q=salir  '
            '(clic en la ventana de preview)')

        # ── Preview thread ───────────────────────────────────────────────────
        self._running = True
        self._last_frame: np.ndarray | None = None
        if self._preview:
            self._preview_thread = threading.Thread(
                target=self._preview_loop, daemon=True)
            self._preview_thread.start()

    # ─────────────────────────────────────────────────────────────────────────
    #  Helpers
    # ─────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _make_gauss(size: int) -> np.ndarray:
        """Kernel gaussiano 1-D, valores en [0,1], pico en el centro."""
        x = np.linspace(-2, 2, size)
        g = np.exp(-x ** 2)
        return g / g.max()

    def _build_depth_frame(self) -> np.ndarray:
        """Genera imagen de profundidad 16-bit (valores en mm) con un blob."""
        BG_DIST_M = self.MAX_DIST * 0.9   # fondo a ~90% del max
        frame_mm = np.full(
            (self.IMG_H, self.IMG_W), int(BG_DIST_M * 1000), dtype=np.uint16)

        # Centro de la persona en píxeles
        cx = int((0.5 + self._lat * 0.4) * self.IMG_W)   # ±40% lateral
        cy = int(0.45 * self.IMG_H)                       # ligeramente arriba

        # Región del blob con clipping
        x0 = max(0, cx - self.BLOB_W // 2)
        x1 = min(self.IMG_W, cx + self.BLOB_W // 2)
        y0 = max(0, cy - self.BLOB_H // 2)
        y1 = min(self.IMG_H, cy + self.BLOB_H // 2)

        bw = x1 - x0
        bh = y1 - y0
        if bw <= 0 or bh <= 0:
            return frame_mm

        # Gaussian 2-D en la región del blob
        gx = cv2.resize(self._gauss_x.reshape(1, -1), (bw, 1),
                        interpolation=cv2.INTER_LINEAR).flatten()
        gy = cv2.resize(self._gauss_y.reshape(-1, 1), (1, bh),
                        interpolation=cv2.INTER_LINEAR).flatten()
        blob = np.outer(gy, gx)   # [bh, bw], valores en [0,1]

        # Persona más cerca que el fondo
        person_mm  = int(self._dist * 1000)
        depth_drop = 400   # mm — cuánto más cerca es la persona del fondo
        blob_mm = (person_mm - blob * depth_drop).astype(np.uint16)
        blob_mm = np.clip(blob_mm, 1, 65535)

        # Solo poner los píxeles que son más cercanos que el fondo
        region = frame_mm[y0:y1, x0:x1]
        frame_mm[y0:y1, x0:x1] = np.minimum(region, blob_mm)

        return frame_mm

    # ─────────────────────────────────────────────────────────────────────────
    #  Timer callback
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_cb(self):
        depth_mm = self._build_depth_frame()

        msg = Image()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'kinect_depth_optical'
        msg.height          = self.IMG_H
        msg.width           = self.IMG_W
        msg.encoding        = '16UC1'
        msg.is_bigendian    = False
        msg.step            = self.IMG_W * 2
        msg.data            = depth_mm.tobytes()
        self._pub.publish(msg)

        self._last_frame = depth_mm

    # ─────────────────────────────────────────────────────────────────────────
    #  Preview loop (OpenCV window + keyboard control)
    # ─────────────────────────────────────────────────────────────────────────

    def _preview_loop(self):
        WIN = 'Fake Depth Sim  [W/S=dist  A/D=lateral  Q=salir]'
        try:
            cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WIN, 800, 500)
        except Exception:
            self.get_logger().warn('No se pudo abrir ventana de preview.')
            return

        while self._running and rclpy.ok():
            frame = self._last_frame
            if frame is None:
                time.sleep(0.05)
                continue

            # Normalizar para visualización (colormap)
            vis_f = frame.astype(np.float32) / 1000.0          # metros
            vis_f = np.clip(vis_f / self.MAX_DIST, 0.0, 1.0)   # [0,1]
            vis_u8 = (vis_f * 255).astype(np.uint8)
            vis_bgr = cv2.applyColorMap(255 - vis_u8, cv2.COLORMAP_JET)

            # ── Dibuja HUD ──
            h, w = vis_bgr.shape[:2]
            cx = int((0.5 + self._lat * 0.4) * w)
            cy = int(0.45 * h)

            cv2.circle(vis_bgr, (cx, cy), 12, (255, 255, 255), 2)
            cv2.line(vis_bgr, (w // 2, h // 2), (cx, cy), (0, 255, 255), 1)

            hud = [
                f'Distancia : {self._dist:.2f} m',
                f'Lateral   : {self._lat:+.2f}  (izq←  →der)',
                '',
                'W/S = acercar/alejar',
                'A/D = mover izq/der',
                'Q   = salir',
            ]
            for i, line in enumerate(hud):
                cv2.putText(vis_bgr, line, (12, 28 + i * 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (255, 255, 255), 1, cv2.LINE_AA)

            # Barra de distancia
            bar_len = int((1 - (self._dist - self.MIN_DIST) /
                           (self.MAX_DIST - self.MIN_DIST)) * (w - 40))
            bar_len = max(0, bar_len)
            color = (0, 255, 0) if self._dist > 0.8 else (0, 0, 255)
            cv2.rectangle(vis_bgr, (20, h - 30), (20 + bar_len, h - 12),
                          color, -1)
            cv2.rectangle(vis_bgr, (20, h - 30), (w - 20, h - 12),
                          (200, 200, 200), 1)
            cv2.putText(vis_bgr, f'{self._dist:.2f}m',
                        (w // 2 - 20, h - 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.imshow(WIN, vis_bgr)
            key = cv2.waitKey(50) & 0xFF

            if key in (ord('q'), ord('Q'), 27):
                self.get_logger().info('Fake Depth: saliendo...')
                self._running = False
                break
            elif key in (ord('w'), ord('W')):
                self._dist = max(self.MIN_DIST,
                                 self._dist - self.STEP_DIST)
            elif key in (ord('s'), ord('S')):
                self._dist = min(self.MAX_DIST,
                                 self._dist + self.STEP_DIST)
            elif key in (ord('a'), ord('A')):
                self._lat = max(-1.0, self._lat - self.STEP_LAT)
            elif key in (ord('d'), ord('D')):
                self._lat = min(1.0, self._lat + self.STEP_LAT)

        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FakeDepthPublisher()
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
