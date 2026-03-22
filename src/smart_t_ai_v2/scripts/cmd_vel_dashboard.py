#!/usr/bin/env python3
"""
cmd_vel Dashboard — Smart Trolley V5 Simulation
================================================
Suscribe a /cmd_vel y dibuja en tiempo real:
  - Velocidad lineal (barra)
  - Velocidad angular (barra + flecha de giro)
  - Historial de los últimos 5 s
  - Estado (AVANZANDO / RETROCEDIENDO / GIRANDO / DETENIDO)

Útil para verificar que person_follower genera los comandos correctos
sin mover el robot físico.

Uso:
  ros2 run smart_t_ai_v2 cmd_vel_dashboard
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import cv2
import numpy as np
import threading
import time
import math


# ── Colores BGR ──────────────────────────────────────────────────────────────
C_BG       = (30,  30,  30)
C_WHITE    = (240, 240, 240)
C_GREEN    = (80,  220, 80)
C_RED      = (80,  80,  220)
C_ORANGE   = (50,  165, 255)
C_YELLOW   = (50,  230, 230)
C_GRAY     = (120, 120, 120)
C_DARK     = (60,  60,  60)


class CmdVelDashboard(Node):

    HISTORY_S  = 6.0     # segundos de historial
    WIN_W      = 720
    WIN_H      = 480

    def __init__(self):
        super().__init__('cmd_vel_dashboard')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_linear',    0.5)   # m/s — escala de barras
        self.declare_parameter('max_angular',   1.5)   # rad/s

        topic        = self.get_parameter('cmd_vel_topic').value
        self._max_lin = float(self.get_parameter('max_linear').value)
        self._max_ang = float(self.get_parameter('max_angular').value)

        self._lin = 0.0
        self._ang = 0.0
        self._last_time = time.time()
        self._history: list[tuple[float, float, float]] = []   # (t, lin, ang)
        self._lock = threading.Lock()

        self._sub = self.create_subscription(
            Twist, topic, self._cb, 10)

        self.get_logger().info(
            f'📊 cmd_vel Dashboard suscrito a {topic}')

        self._running = True
        self._win_thread = threading.Thread(
            target=self._display_loop, daemon=True)
        self._win_thread.start()

    # ─────────────────────────────────────────────────────────────────────────
    def _cb(self, msg: Twist):
        with self._lock:
            self._lin = msg.linear.x
            self._ang = msg.angular.z
            now = time.time()
            self._history.append((now, self._lin, self._ang))
            cutoff = now - self.HISTORY_S
            self._history = [(t, l, a) for t, l, a in self._history
                             if t >= cutoff]
            self._last_time = now

    # ─────────────────────────────────────────────────────────────────────────
    #  Dibujo
    # ─────────────────────────────────────────────────────────────────────────

    def _draw(self, lin: float, ang: float,
              history: list) -> np.ndarray:
        W, H = self.WIN_W, self.WIN_H
        img = np.full((H, W, 3), C_BG, dtype=np.uint8)

        # ── Título ──────────────────────────────────────────────────────────
        cv2.putText(img, 'cmd_vel Dashboard — Person Follower Sim',
                    (16, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, C_WHITE, 1, cv2.LINE_AA)

        # ── Estado ──────────────────────────────────────────────────────────
        stale = (time.time() - self._last_time) > 1.5
        if stale:
            status, sc = 'SIN DATOS', C_GRAY
        elif abs(lin) < 0.01 and abs(ang) < 0.01:
            status, sc = 'DETENIDO', C_GRAY
        elif abs(ang) > 0.1 and abs(lin) < 0.05:
            status, sc = 'GIRANDO', C_YELLOW
        elif lin > 0.02:
            status, sc = 'AVANZANDO', C_GREEN
        elif lin < -0.02:
            status, sc = 'RETROCEDIENDO', C_RED
        else:
            status, sc = 'AJUSTANDO', C_ORANGE

        cv2.rectangle(img, (W - 200, 8), (W - 8, 40), sc, -1)
        cv2.putText(img, status, (W - 195, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_BG, 2, cv2.LINE_AA)

        # ── Barra velocidad lineal ───────────────────────────────────────────
        self._draw_bar(img, 'Lineal (m/s)', lin, self._max_lin,
                       y=70, w=W, c_pos=C_GREEN, c_neg=C_RED)

        # ── Barra velocidad angular ──────────────────────────────────────────
        self._draw_bar(img, 'Angular (rad/s)', ang, self._max_ang,
                       y=130, w=W, c_pos=C_YELLOW, c_neg=C_ORANGE,
                       invert_dir=True)

        # ── Robot top-view con flecha ────────────────────────────────────────
        self._draw_robot_view(img, lin, ang, cx=W // 2, cy=270, r=60)

        # ── Gráfica de historial ─────────────────────────────────────────────
        self._draw_history(img, history,
                           x0=20, y0=370, gw=W - 40, gh=90)

        # ── Valores numéricos ────────────────────────────────────────────────
        cv2.putText(img, f'lin={lin:+.3f} m/s    ang={ang:+.3f} rad/s',
                    (16, H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_GRAY, 1, cv2.LINE_AA)

        return img

    @staticmethod
    def _draw_bar(img, label: str, val: float, max_val: float,
                  y: int, w: int,
                  c_pos, c_neg, invert_dir=False):
        """Barra centrada: positivo→verde (derecha), negativo→rojo (izquierda)."""
        BAR_H  = 30
        BAR_X0 = 20
        BAR_W  = w - 40
        mid    = BAR_X0 + BAR_W // 2

        # Fondo de la barra
        cv2.rectangle(img, (BAR_X0, y), (BAR_X0 + BAR_W, y + BAR_H),
                      C_DARK, -1)
        cv2.rectangle(img, (BAR_X0, y), (BAR_X0 + BAR_W, y + BAR_H),
                      C_GRAY, 1)
        # Línea central
        cv2.line(img, (mid, y), (mid, y + BAR_H), C_GRAY, 1)

        # Relleno proporcional
        frac = np.clip(val / max_val, -1.0, 1.0)
        px   = int(frac * (BAR_W // 2))
        color = c_pos if val >= 0 else c_neg
        if invert_dir:
            color = c_neg if val >= 0 else c_pos
        if px != 0:
            x1, x2 = (mid, mid + px) if px > 0 else (mid + px, mid)
            cv2.rectangle(img, (x1, y + 2), (x2, y + BAR_H - 2), color, -1)

        # Label
        cv2.putText(img, f'{label}: {val:+.3f}',
                    (BAR_X0 + 4, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_WHITE, 1, cv2.LINE_AA)

    @staticmethod
    def _draw_robot_view(img, lin: float, ang: float,
                         cx: int, cy: int, r: int):
        """Vista superior del robot con flecha de movimiento."""
        # Círculo de fondo
        cv2.circle(img, (cx, cy), r + 20, C_DARK, -1)
        cv2.circle(img, (cx, cy), r + 20, C_GRAY, 1)

        # Robot (rectángulo)
        rect_pts = np.array([
            [cx - 18, cy - 30],
            [cx + 18, cy - 30],
            [cx + 18, cy + 30],
            [cx - 18, cy + 30],
        ], dtype=np.int32)
        cv2.fillPoly(img, [rect_pts], C_DARK)
        cv2.polylines(img, [rect_pts], True, C_GRAY, 1)
        # Frente del robot (triángulo)
        front = np.array([
            [cx - 14, cy - 30],
            [cx + 14, cy - 30],
            [cx, cy - 42],
        ], dtype=np.int32)
        cv2.fillPoly(img, [front], C_WHITE)

        # Flecha de velocidad lineal
        arrow_len = int(lin / 0.5 * 50)
        arrow_color = C_GREEN if lin >= 0 else C_RED
        if abs(arrow_len) > 3:
            cv2.arrowedLine(img, (cx, cy), (cx, cy - arrow_len),
                            arrow_color, 2, tipLength=0.3)

        # Flecha de giro
        if abs(ang) > 0.05:
            arc_r = r - 5
            ang_color = C_YELLOW
            # Dibuja un arco de 90° en la dirección del giro
            start_angle = -90
            end_angle   = -90 + int(math.degrees(ang) * 0.6)
            cv2.ellipse(img, (cx, cy), (arc_r, arc_r),
                        0, start_angle, end_angle, ang_color, 2)
            tip_a = math.radians(end_angle)
            tip_x = int(cx + arc_r * math.cos(tip_a))
            tip_y = int(cy + arc_r * math.sin(tip_a))
            cv2.circle(img, (tip_x, tip_y), 4, ang_color, -1)

        # Leyenda
        cv2.putText(img, 'Vista Superior', (cx - 45, cy + r + 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, C_GRAY, 1)

    @staticmethod
    def _draw_history(img, history: list,
                      x0: int, y0: int, gw: int, gh: int):
        """Mini-gráfica de historial de velocidades."""
        cv2.rectangle(img, (x0, y0), (x0 + gw, y0 + gh), C_DARK, -1)
        cv2.rectangle(img, (x0, y0), (x0 + gw, y0 + gh), C_GRAY, 1)
        mid_y = y0 + gh // 2
        cv2.line(img, (x0, mid_y), (x0 + gw, mid_y), C_DARK, 1)

        if len(history) < 2:
            cv2.putText(img, 'Esperando datos...',
                        (x0 + 10, mid_y + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_GRAY, 1)
            return

        now   = time.time()
        t_min = now - 6.0
        t_max = now

        def tx(t):
            return x0 + int((t - t_min) / (t_max - t_min) * gw)

        def vy(v, max_v=0.5):
            frac = np.clip(v / max_v, -1, 1)
            return mid_y - int(frac * (gh // 2 - 4))

        # Línea verde = lineal, amarillo = angular
        for i in range(1, len(history)):
            t0, l0, a0 = history[i - 1]
            t1, l1, a1 = history[i]
            cv2.line(img,
                     (tx(t0), vy(l0, 0.5)),
                     (tx(t1), vy(l1, 0.5)),
                     C_GREEN, 1)
            cv2.line(img,
                     (tx(t0), vy(a0, 1.5)),
                     (tx(t1), vy(a1, 1.5)),
                     C_YELLOW, 1)

        cv2.putText(img, '─ lineal', (x0 + 4, y0 + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, C_GREEN, 1)
        cv2.putText(img, '─ angular', (x0 + 70, y0 + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, C_YELLOW, 1)

    # ─────────────────────────────────────────────────────────────────────────
    #  Display loop
    # ─────────────────────────────────────────────────────────────────────────

    def _display_loop(self):
        WIN = 'cmd_vel Dashboard  [Q=salir]'
        try:
            cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WIN, self.WIN_W, self.WIN_H)
        except Exception:
            self.get_logger().warn('No se pudo abrir ventana del dashboard.')
            return

        while self._running and rclpy.ok():
            with self._lock:
                lin  = self._lin
                ang  = self._ang
                hist = list(self._history)

            frame = self._draw(lin, ang, hist)
            cv2.imshow(WIN, frame)
            key = cv2.waitKey(50) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                self._running = False
                break

        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelDashboard()
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
