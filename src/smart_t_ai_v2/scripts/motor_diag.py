#!/usr/bin/env python3
"""
motor_diag.py — Smart Trolley V5
Diagnóstico de motores individuales + dead-zone detection.

Secuencia:
  1. Giro puro izquierda  (→ rueda DER adelante, IZQ atrás)
  2. Giro puro derecha    (→ rueda IZQ adelante, DER atrás)
  3. Ramp lineal 0→0.30 m/s en 6 pasos (detecta dead-zone)
  4. Encoders en tiempo real para ver qué rueda cuenta pulsos

Uso:
  ros2 run smart_t_ai_v2 motor_diag
  ros2 run smart_t_ai_v2 motor_diag --ros-args -p speed:=0.25
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import threading


SEP = '─' * 55


class MotorDiag(Node):
    def __init__(self):
        super().__init__('motor_diag')
        self.declare_parameter('speed', 0.20)

        self.pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub  = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self._vl = 0.0  # velocidad lineal odometría
        self._vr = 0.0  # angular
        self._lock = threading.Lock()

        self.get_logger().info('Esperando 3s para que arduino_bridge conecte...')
        self._timer = self.create_timer(3.0, self._start)
        self._started = False

    def _odom_cb(self, msg):
        with self._lock:
            self._vl = msg.twist.twist.linear.x
            self._vr = msg.twist.twist.angular.z

    def _pub(self, lin, ang, secs, label):
        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        t_end = time.time() + secs
        self.get_logger().info(f'▶  {label}')
        while time.time() < t_end:
            self.pub.publish(msg)
            time.sleep(0.05)
        # Leer odometría al final del paso
        with self._lock:
            vl, vr = self._vl, self._vr
        self.get_logger().info(
            f'   odom → lin={vl:+.3f} m/s  ang={vr:+.3f} rad/s')

    def _stop(self, secs=0.8):
        self._pub(0.0, 0.0, secs, 'STOP')

    def _start(self):
        if self._started:
            return
        self._started = True
        self._timer.cancel()

        speed = self.get_parameter('speed').value

        self.get_logger().info(SEP)
        self.get_logger().info('DIAGNÓSTICO DE MOTORES')
        self.get_logger().info(SEP)

        # ── Test 1: giro puro izquierda ─────────────────────────────────────
        # ang>0 (ROS) → Arduino convierte a: IZQ retrocede, DER avanza
        self.get_logger().info('TEST 1: GIRO IZQ (DER debe girar sola)')
        self._pub(0.0, speed, 1.5, f'ang=+{speed:.2f} rad/s')
        self._stop()

        # ── Test 2: giro puro derecha ────────────────────────────────────────
        self.get_logger().info('TEST 2: GIRO DER (IZQ debe girar sola)')
        self._pub(0.0, -speed, 1.5, f'ang=-{speed:.2f} rad/s')
        self._stop()

        # ── Test 3: ramp lineal ──────────────────────────────────────────────
        self.get_logger().info('TEST 3: RAMP LINEAL (ambos motores, vel creciente)')
        for v in [0.08, 0.12, 0.18, 0.22, 0.28, 0.33]:
            self._pub(v, 0.0, 1.2, f'lin=+{v:.2f} m/s')
            self._stop(0.4)

        # ── Test 4: retroceso ramp ───────────────────────────────────────────
        self.get_logger().info('TEST 4: RAMP RETROCESO')
        for v in [0.08, 0.15, 0.22]:
            self._pub(-v, 0.0, 1.2, f'lin={-v:.2f} m/s')
            self._stop(0.4)

        self.get_logger().info(SEP)
        self.get_logger().info('✅ Diagnóstico completo.')
        self.get_logger().info('Si el robot giró en TEST 1/2 → motores OK en giro.')
        self.get_logger().info('Si la odometría mostró vel ≈ 0 en algún paso → dead-zone o motor muerto.')
        self.get_logger().info(SEP)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDiag()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
