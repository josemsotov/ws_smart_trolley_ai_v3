#!/usr/bin/env python3
"""
test_motors_hall.py — Smart Trolley V5
Prueba completa de motores + encoders Hall con visualización RViz.

Secuencia de pruebas:
  0. Diagnóstico de encoders (rueda izquierda sola, rueda derecha sola)
  1. Avance   N metros
  2. Retroceso N metros (regresa a origen)
  3. Giro izquierda 90°
  4. Giro derecha 90°  (restituye heading)
  5. Patrón cuadrado NxN metros
  6. Reporte de precisión (error XY, error heading, PPR efectivo)

RViz output:
  /motor_test/path      → trayectoria recorrida  (nav_msgs/Path)
  /motor_test/markers   → waypoints con etiquetas (visualization_msgs/MarkerArray)
  /motor_test/status    → estado actual del test  (std_msgs/String)

Uso:
  ros2 run smart_t_ai_v2 test_motors_hall
  ros2 run smart_t_ai_v2 test_motors_hall --ros-args -p test_distance:=1.0 -p square_side:=0.8
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import math
import time
import threading


# ── Colores para marcadores ────────────────────────────────────────────────
C_GREEN  = (0.0, 0.9, 0.2)
C_BLUE   = (0.2, 0.4, 1.0)
C_ORANGE = (1.0, 0.5, 0.0)
C_RED    = (1.0, 0.1, 0.1)
C_WHITE  = (1.0, 1.0, 1.0)
C_YELLOW = (1.0, 0.9, 0.0)


class MotorHallTest(Node):
    """Nodo ROS2 de prueba de motores + encoders Hall."""

    def __init__(self):
        super().__init__('motor_hall_test')

        # ── Parámetros ───────────────────────────────────────────────────────
        self.declare_parameter('test_distance',     0.5)    # metros por prueba lineal
        self.declare_parameter('square_side',       0.5)    # metros del cuadrado
        self.declare_parameter('linear_speed',      0.15)   # m/s velocidad de avance
        self.declare_parameter('turn_speed',        0.4)    # rad/s velocidad de giro
        self.declare_parameter('wait_between',      2.0)    # seg entre pruebas
        self.declare_parameter('run_single_wheel',  True)   # probar ruedas individualmente
        self.declare_parameter('run_square',        True)   # ejecutar cuadrado
        self.declare_parameter('auto_start_delay',  3.0)    # seg antes de iniciar
        self.declare_parameter('wheel_separation',  0.48)   # m separación de ruedas
        self.declare_parameter('wheel_radius',      0.10)   # m radio de rueda

        self.dist         = self.get_parameter('test_distance').value
        self.square_side  = self.get_parameter('square_side').value
        self.lin_spd      = self.get_parameter('linear_speed').value
        self.turn_spd     = self.get_parameter('turn_speed').value
        self.wait_t       = self.get_parameter('wait_between').value
        self.do_single    = self.get_parameter('run_single_wheel').value
        self.do_square    = self.get_parameter('run_square').value
        self.start_delay  = self.get_parameter('auto_start_delay').value
        self.wheel_sep    = self.get_parameter('wheel_separation').value
        self.wheel_rad    = self.get_parameter('wheel_radius').value

        # ── Estado de odometría ──────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.vel_l = 0.0  # velocidad rueda izquierda (rad/s) de joint_states
        self.vel_r = 0.0  # velocidad rueda derecha
        self.pos_l = 0.0  # posición rueda izquierda (rad)
        self.pos_r = 0.0  # posición rueda derecha
        self.odom_ready = False

        # ── Resultados de pruebas ────────────────────────────────────────────
        self.results     = []
        self.test_running = False
        self.marker_id   = 0

        # ── Publishers ───────────────────────────────────────────────────────
        self.cmd_pub     = self.create_publisher(Twist,        '/cmd_vel',               10)
        self.path_pub    = self.create_publisher(Path,         '/motor_test/path',        10)
        self.markers_pub = self.create_publisher(MarkerArray,  '/motor_test/markers',     10)
        self.status_pub  = self.create_publisher(String,       '/motor_test/status',      10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.odom_sub  = self.create_subscription(Odometry,   '/odom',         self._odom_cb,   10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self._joint_cb,  10)

        # ── Trayectoria acumulada ────────────────────────────────────────────
        self.path = Path()
        self.path.header.frame_id = 'odom'

        # ── Timers ───────────────────────────────────────────────────────────
        self.path_timer = self.create_timer(0.1, self._publish_path)

        self._log_banner()
        self._start_thread()

    # ── Logging helpers ──────────────────────────────────────────────────────

    def _log_banner(self):
        self.get_logger().info('━' * 62)
        self.get_logger().info('  MOTOR + HALL ENCODER TEST  —  Smart Trolley V5')
        self.get_logger().info('━' * 62)
        self.get_logger().info(f'  Distancia prueba : {self.dist:.2f} m')
        self.get_logger().info(f'  Lado cuadrado    : {self.square_side:.2f} m')
        self.get_logger().info(f'  Velocidad lineal : {self.lin_spd:.2f} m/s')
        self.get_logger().info(f'  Velocidad giro   : {math.degrees(self.turn_spd):.0f} °/s')
        self.get_logger().info(f'  Separación ruedas: {self.wheel_sep:.3f} m')
        self.get_logger().info(f'  Radio de rueda   : {self.wheel_rad:.3f} m')
        # Encoder resolution info
        enc_ppr = 85  # Profile B: Hall(45) + Opto(40) = 85 PPR
        pulse_mm = (2 * math.pi * self.wheel_rad / enc_ppr) * 1000
        self.get_logger().info(f'  Encoders         : Hall(45) + Opto(40) = {enc_ppr} PPR')
        self.get_logger().info(f'  Resolución        : {pulse_mm:.1f} mm/pulso  |  polling 50 Hz (20ms)')
        self.get_logger().info('  Esperando /odom ...')

    def _log_sep(self, char='─', width=60):
        self.get_logger().info(char * width)

    def _log(self, msg):
        self.get_logger().info(msg)

    # ── Callbacks de suscriptores ─────────────────────────────────────────────

    def _odom_cb(self, msg):
        self.x     = msg.pose.pose.position.x
        self.y     = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.theta = 2.0 * math.atan2(qz, qw)

        if not self.odom_ready:
            self.odom_ready = True
            self._log(f'  ✓ /odom recibido  x={self.x:.3f}  y={self.y:.3f}  '
                      f'θ={math.degrees(self.theta):.1f}°')

        # Añadir a trayectoria
        ps = PoseStamped()
        ps.header.stamp  = self.get_clock().now().to_msg()
        ps.header.frame_id = 'odom'
        ps.pose = msg.pose.pose
        self.path.poses.append(ps)
        if len(self.path.poses) > 5000:
            self.path.poses = self.path.poses[-5000:]

    def _joint_cb(self, msg):
        """Recoge velocidades y posiciones de ruedas desde joint_states."""
        if 'joint_lh' in msg.name:
            idx_l = msg.name.index('joint_lh')
            self.pos_l = msg.position[idx_l] if msg.position else 0.0
            self.vel_l = msg.velocity[idx_l] if msg.velocity else 0.0
        if 'joint_rh' in msg.name:
            idx_r = msg.name.index('joint_rh')
            self.pos_r = msg.position[idx_r] if msg.position else 0.0
            self.vel_r = msg.velocity[idx_r] if msg.velocity else 0.0

    def _publish_path(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)

    # ── Control de movimiento ─────────────────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _drive(self, linear, angular, duration_s):
        """Envía comando de velocidad durante duration_s segundos."""
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        t_end = time.time() + duration_s
        while time.time() < t_end and rclpy.ok():
            self.cmd_pub.publish(msg)
            time.sleep(0.05)
        self._stop()
        time.sleep(0.2)

    # ── Primitivas con feedback de odometría ──────────────────────────────────

    def _move_distance(self, distance_m, speed_m_s, label=''):
        """
        Mueve el robot `distance_m` metros (signo define dirección).
        Usa odometría como feedback de posición.
        Devuelve distancia real recorrida.
        """
        sign    = 1.0 if distance_m >= 0 else -1.0
        target  = abs(distance_m)
        sx, sy  = self.x, self.y
        timeout = target / abs(speed_m_s) * 3.0 + 5.0
        t0      = time.time()

        # Braking margin (direction-aware, calibrated for 50Hz encoder updates):
        #   FORWARD : inertia ~5.5cm @ 0.15m/s  → stop early
        #   BACKWARD: inertia ~0.3cm @ 0.15m/s  → stop near target
        if sign > 0:  # forward
            brake_factor = max(0.85, 1.0 - abs(speed_m_s) * 0.51)
        else:         # backward — minimal inertia, stop close to target
            brake_factor = max(0.95, 1.0 - abs(speed_m_s) * 0.05)

        msg = Twist()
        msg.linear.x = sign * abs(speed_m_s)

        while rclpy.ok():
            traveled = math.hypot(self.x - sx, self.y - sy)
            if traveled >= target * brake_factor:
                break
            if time.time() - t0 > timeout:
                self._log(f'  ⚠  Timeout en movimiento ({label})')
                break
            self.cmd_pub.publish(msg)
            time.sleep(0.02)

        self._stop()
        time.sleep(0.3)

        actual = math.hypot(self.x - sx, self.y - sy)
        return actual

    def _rotate_angle(self, angle_deg, speed_rad_s, label=''):
        """
        Gira angle_deg grados usando odometría de heading.
        Positivo = antihorario (IZQ), Negativo = horario (DER).

        Freno calibrado para 0.4 rad/s en superficie plana (Run 9-10):
          IZQ coast 14-19° (variación run-to-run) → frena al 78% (70°)
          DER coast 14-46° (más variable) → frena al 78% (70°)
        Micro-corrección activa si residuo > 5° (0.15 rad/s, costa ≈5° empírica).
        Un error de 5° sin corregir × 4 turnos cuadrado = 20° drift acumulado!
        """
        target_rad = math.radians(angle_deg)
        sign       = 1.0 if angle_deg >= 0 else -1.0
        start_th   = self.theta
        timeout    = abs(target_rad) / abs(speed_rad_s) * 3.0 + 5.0
        t0         = time.time()

        # Brake_factor 0.78 para ambas direcciones — motor alcanza steady-state a ~70°
        # DER tiene ~2× más costa que IZQ por asimetría de motores (motor DER más fuerte)
        brake_factor = 0.78

        msg = Twist()
        msg.angular.z = sign * abs(speed_rad_s)

        while rclpy.ok():
            rotated = math.atan2(
                math.sin(self.theta - start_th),
                math.cos(self.theta - start_th))
            if abs(rotated) >= abs(target_rad) * brake_factor:
                break
            if time.time() - t0 > timeout:
                self._log(f'  ⚠  Timeout en giro ({label})')
                break
            self.cmd_pub.publish(msg)
            time.sleep(0.02)

        self._stop()
        time.sleep(0.6)   # Esperar inercia completa antes de medir

        actual_rad = math.atan2(
            math.sin(self.theta - start_th),
            math.cos(self.theta - start_th))
        residual_deg = angle_deg - math.degrees(actual_rad)

        # Micro-corrección si residuo > 5°  (0.15 rad/s → costa empírica ≈5°)
        # Umbral bajo es CRITICO: 5° × 4 turnos cuadrado = 20° drift si no se corrige
        if abs(residual_deg) > 5.0:
            self._log(f'  ↺ Micro-corrección: {residual_deg:+.1f}°')
            micro_sign = 1.0 if residual_deg > 0 else -1.0
            micro_rad  = math.radians(residual_deg)
            MICRO_SPEED = 0.15                                           # rad/s
            # Costa a 0.15 rad/s ≈ 5° para residuos grandes, ≈3° para pequeños
            MICRO_BRAKE = max(0.50, 1.0 - 5.0 / max(abs(residual_deg), 8.0))

            msg_micro = Twist()
            msg_micro.angular.z = micro_sign * MICRO_SPEED
            start_th2 = self.theta
            t_micro   = time.time()
            timeout_micro = abs(micro_rad) / MICRO_SPEED * 3.0 + 3.0

            while rclpy.ok():
                rotated2 = math.atan2(
                    math.sin(self.theta - start_th2),
                    math.cos(self.theta - start_th2))
                if abs(rotated2) >= abs(micro_rad) * MICRO_BRAKE:
                    break
                if time.time() - t_micro > timeout_micro:
                    break
                self.cmd_pub.publish(msg_micro)
                time.sleep(0.02)

            self._stop()
            time.sleep(0.3)

        actual_rad = math.atan2(
            math.sin(self.theta - start_th),
            math.cos(self.theta - start_th))
        return math.degrees(actual_rad)

    def _single_wheel(self, wheel='left', speed=0.15, duration=2.0):
        """
        Activa UNA sola rueda.
          Rueda izquierda sola : linear =  v/2 , angular = -v/base
          Rueda derecha sola   : linear =  v/2 , angular = +v/base
        """
        v_wheel  = abs(speed)
        base     = self.wheel_sep
        linear   = v_wheel / 2.0
        angular  = (-v_wheel / base) if wheel == 'left' else (v_wheel / base)

        self._log(f'    cmd → linear={linear:.3f} m/s  angular={angular:.3f} rad/s')

        # Snapshot de encoders antes
        pos_l_0, pos_r_0 = self.pos_l, self.pos_r
        vel_l_0, vel_r_0 = self.vel_l, self.vel_r

        self._drive(linear, angular, duration)
        time.sleep(0.3)

        delta_l = abs(self.pos_l - pos_l_0)   # rad
        delta_r = abs(self.pos_r - pos_r_0)   # rad
        dist_l  = delta_l * self.wheel_rad
        dist_r  = delta_r * self.wheel_rad
        rpm_l   = abs(self.vel_l) * 60.0 / (2.0 * math.pi)
        rpm_r   = abs(self.vel_r) * 60.0 / (2.0 * math.pi)

        return {
            'delta_l_rad': delta_l,
            'delta_r_rad': delta_r,
            'dist_l_m':    dist_l,
            'dist_r_m':    dist_r,
            'rpm_l':       rpm_l,
            'rpm_r':       rpm_r,
        }

    # ── Marcadores RViz ──────────────────────────────────────────────────────

    def _add_marker(self, x, y, label, color=C_GREEN, shape=Marker.SPHERE, z=0.05):
        ma = MarkerArray()

        # Esfera / cubo
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns              = 'waypoints'
        m.id              = self.marker_id;  self.marker_id += 1
        m.type            = shape
        m.action          = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.10
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 0.9
        m.lifetime.sec = 0
        ma.markers.append(m)

        # Texto
        t = Marker()
        t.header = m.header
        t.ns     = 'labels'
        t.id     = self.marker_id;  self.marker_id += 1
        t.type   = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x = x
        t.pose.position.y = y + 0.15
        t.pose.position.z = 0.20
        t.pose.orientation.w = 1.0
        t.scale.z    = 0.07
        t.color.r = t.color.g = t.color.b = t.color.a = 1.0
        t.text       = label
        t.lifetime.sec = 0
        ma.markers.append(t)

        self.markers_pub.publish(ma)

    # ── Secuencia completa de pruebas ─────────────────────────────────────────

    def _start_thread(self):
        t = threading.Thread(target=self._run_sequence, daemon=True)
        t.start()

    def _run_sequence(self):
        # Esperar odometría
        t0 = time.time()
        while not self.odom_ready and rclpy.ok():
            if time.time() - t0 > 30.0:
                self._log('  ✗ Timeout esperando /odom. ¿arduino_bridge corriendo?')
                return
            time.sleep(0.1)

        self._log(f'  Iniciando pruebas en {self.start_delay:.0f} segundos...')
        time.sleep(self.start_delay)
        if not rclpy.ok():
            return

        self._log_sep('━')
        self._log('  INICIO DE SECUENCIA DE PRUEBAS')
        self._log_sep('━')

        origin_x, origin_y = self.x, self.y
        self._add_marker(origin_x, origin_y, 'ORIGEN', C_BLUE, Marker.CUBE)
        self.test_running = True

        # ── 0. Diagnóstico de encoders Hall (rueda individual) ──────────────
        if self.do_single:
            self._test_single_wheels()
            time.sleep(self.wait_t)

        # ── 1. Avance ────────────────────────────────────────────────────────
        self._log('')
        self._log(f'── TEST 1: AVANCE {self.dist:.2f} m ──')
        self._publish_status(f'TEST 1: Avance {self.dist:.2f}m')
        self._add_marker(self.x, self.y, 'T1 START', C_GREEN)
        pos_l_0, pos_r_0 = self.pos_l, self.pos_r

        actual_fwd = self._move_distance(self.dist, self.lin_spd, 'fwd')

        delta_l = abs(self.pos_l - pos_l_0)
        delta_r = abs(self.pos_r - pos_r_0)
        self._log_result_linear('Avance', self.dist, actual_fwd, delta_l, delta_r)
        self._add_marker(self.x, self.y,
                         f'T1 END {actual_fwd*100:.0f}cm', C_ORANGE)
        time.sleep(self.wait_t)

        # ── 2. Retroceso ─────────────────────────────────────────────────────
        self._log('')
        self._log(f'── TEST 2: RETROCESO {self.dist:.2f} m ──')
        self._publish_status(f'TEST 2: Retroceso {self.dist:.2f}m')
        self._add_marker(self.x, self.y, 'T2 START', C_GREEN)
        pos_l_0, pos_r_0 = self.pos_l, self.pos_r

        actual_bwd = self._move_distance(-self.dist, self.lin_spd, 'bwd')

        delta_l = abs(self.pos_l - pos_l_0)
        delta_r = abs(self.pos_r - pos_r_0)
        self._log_result_linear('Retroceso', self.dist, actual_bwd, delta_l, delta_r)
        self._add_marker(self.x, self.y,
                         f'T2 END {actual_bwd*100:.0f}cm', C_ORANGE)
        time.sleep(self.wait_t)

        # ── 3. Giro izquierda 90° ────────────────────────────────────────────
        self._log('')
        self._log('── TEST 3: GIRO IZQUIERDA 90° ──')
        self._publish_status('TEST 3: Giro izq 90°')
        self._add_marker(self.x, self.y, 'T3 GIRO IZQ', C_YELLOW)
        theta_before = self.theta

        actual_left = self._rotate_angle(90.0, self.turn_spd, 'left90')

        err_left = 90.0 - abs(actual_left)
        self._log_result_rotation('Giro IZQ 90°', 90.0, actual_left)
        time.sleep(self.wait_t)

        # ── 4. Giro derecha 90° (restaura heading) ───────────────────────────
        self._log('')
        self._log('── TEST 4: GIRO DERECHA 90° ──')
        self._publish_status('TEST 4: Giro der 90°')
        self._add_marker(self.x, self.y, 'T4 GIRO DER', C_YELLOW)

        actual_right = self._rotate_angle(-90.0, self.turn_spd, 'right90')

        self._log_result_rotation('Giro DER 90°', -90.0, actual_right)
        heading_restored_err = math.degrees(math.atan2(
            math.sin(self.theta - theta_before),
            math.cos(self.theta - theta_before)))
        self._log(f'    Heading restaurado: error residual = {heading_restored_err:.2f}°')
        time.sleep(self.wait_t)

        # ── 5. Cuadrado ──────────────────────────────────────────────────────
        if self.do_square:
            self._test_square()
            time.sleep(self.wait_t)

        # ── Resumen final ────────────────────────────────────────────────────
        self._print_summary(origin_x, origin_y)
        self.test_running = False

    # ── Prueba de ruedas individuales ─────────────────────────────────────────

    def _test_single_wheels(self):
        self._log('')
        self._log('── TEST 0: DIAGNÓSTICO ENCODERS HALL (ruedas individuales) ──')
        duration = 2.0  # segundos por rueda

        # ─ Rueda IZQUIERDA ──────────────────────────────────────────────────
        self._log(f'  ▷  Rueda IZQUIERDA sola ({duration:.0f}s @ {self.lin_spd:.2f}m/s)')
        self._publish_status('Diagnóstico: rueda izquierda')
        r_l = self._single_wheel('left', self.lin_spd, duration)
        self._log(f'    ΔPos L={math.degrees(r_l["delta_l_rad"]):.1f}°  '
                  f'R={math.degrees(r_l["delta_r_rad"]):.1f}°  '
                  f'(esperado L>0, R≈0)')
        self._log(f'    Dist L={r_l["dist_l_m"]*100:.1f}cm  R={r_l["dist_r_m"]*100:.1f}cm')
        self._log(f'    RPM  L={r_l["rpm_l"]:.1f}  R={r_l["rpm_r"]:.1f}')
        hall_ok_l = r_l['delta_l_rad'] > 0.2 and r_l['delta_r_rad'] < 0.3
        self._log(f'    Hall IZQ: {"✓ OK" if hall_ok_l else "⚠ REVISAR"}')
        time.sleep(self.wait_t * 0.7)

        # ─ Rueda DERECHA ─────────────────────────────────────────────────────
        self._log(f'  ▷  Rueda DERECHA sola ({duration:.0f}s @ {self.lin_spd:.2f}m/s)')
        self._publish_status('Diagnóstico: rueda derecha')
        r_r = self._single_wheel('right', self.lin_spd, duration)
        self._log(f'    ΔPos L={math.degrees(r_r["delta_l_rad"]):.1f}°  '
                  f'R={math.degrees(r_r["delta_r_rad"]):.1f}°  '
                  f'(esperado R>0, L≈0)')
        self._log(f'    Dist L={r_r["dist_l_m"]*100:.1f}cm  R={r_r["dist_r_m"]*100:.1f}cm')
        self._log(f'    RPM  L={r_r["rpm_l"]:.1f}  R={r_r["rpm_r"]:.1f}')
        hall_ok_r = r_r['delta_r_rad'] > 0.2 and r_r['delta_l_rad'] < 0.3
        self._log(f'    Hall DER: {"✓ OK" if hall_ok_r else "⚠ REVISAR"}')

        self.results.append({
            'tipo': 'hall_diag',
            'hall_izq_ok': hall_ok_l,
            'hall_der_ok': hall_ok_r,
            'rpm_izq': r_l['rpm_l'],
            'rpm_der': r_r['rpm_r'],
        })

    # ── Prueba de cuadrado ────────────────────────────────────────────────────

    def _test_square(self):
        self._log('')
        self._log(f'── TEST 5: PATRÓN CUADRADO {self.square_side:.2f}×{self.square_side:.2f} m ──')
        self._publish_status(f'TEST 5: Cuadrado {self.square_side:.2f}m')

        sx, sy, sth = self.x, self.y, self.theta
        self._add_marker(sx, sy, f'□ INICIO', C_BLUE, Marker.CUBE, z=0.08)

        for i in range(4):
            self._publish_status(f'Cuadrado: lado {i+1}/4')
            actual = self._move_distance(self.square_side, self.lin_spd, f'sq{i+1}')
            self._add_marker(self.x, self.y,
                             f'S{i+1}: {actual*100:.0f}cm', C_GREEN)
            time.sleep(self.wait_t * 0.4)

            actual_r = self._rotate_angle(90.0, self.turn_spd, f'sqt{i+1}')
            time.sleep(self.wait_t * 0.3)

        dx       = self.x - sx
        dy       = self.y - sy
        dth      = math.degrees(math.atan2(
            math.sin(self.theta - sth),
            math.cos(self.theta - sth)))
        closure  = math.hypot(dx, dy)

        self._add_marker(self.x, self.y,
                         f'□ FIN  ΔXY={closure*100:.1f}cm', C_RED, Marker.CUBE, z=0.08)

        self._log(f'  ■  CUADRADO completo:')
        self._log(f'     Error de cierre XY : {closure*100:.1f} cm')
        self._log(f'     Desplaz. X={dx*100:.1f}cm  Y={dy*100:.1f}cm')
        self._log(f'     Error de heading    : {dth:.2f}°')

        pct = (closure / (4 * self.square_side)) * 100
        self._log(f'     Precisión de cierre : {100-pct:.1f}%  '
                  f'{"✓ Buena" if pct < 5 else "⚠ Revisar PPR"}')

        self.results.append({
            'tipo':        'square',
            'side':        self.square_side,
            'closure_cm':  closure * 100,
            'heading_err': dth,
        })

    # ── Helpers de log ────────────────────────────────────────────────────────

    def _log_result_linear(self, name, target, actual, delta_l_rad, delta_r_rad):
        err_m   = abs(target - actual)
        err_pct = err_m / target * 100 if target > 0 else 0.0
        sym     = '✓' if err_pct < 8 else '⚠'
        # Convertir ΔRad → pulsos usando PPR nominal (85)
        ppr     = 85
        pulses_l = delta_l_rad / (2 * math.pi) * ppr
        pulses_r = delta_r_rad / (2 * math.pi) * ppr
        self._log(f'  {sym} {name:12s}: objetivo={target:.3f}m  '
                  f'real={actual:.3f}m  error={err_m*100:.1f}cm ({err_pct:.1f}%)')
        self._log(f'     Encoders → L={delta_l_rad:.2f}rad ({pulses_l:.0f}p)  '
                  f'R={delta_r_rad:.2f}rad ({pulses_r:.0f}p)  '
                  f'simetría={abs(delta_l_rad-delta_r_rad)/max(delta_l_rad,delta_r_rad,0.001)*100:.1f}%asim')
        self.results.append({
            'tipo':     'lineal',
            'nombre':   name,
            'target':   target,
            'actual':   actual,
            'error_m':  err_m,
            'error_pct':err_pct,
        })

    def _log_result_rotation(self, name, target_deg, actual_deg):
        err = abs(target_deg) - abs(actual_deg)
        sym = '✓' if abs(err) < 5 else '⚠'
        self._log(f'  {sym} {name:14s}: objetivo={target_deg:.1f}°  '
                  f'real={actual_deg:.1f}°  error={err:.1f}°')
        self.results.append({
            'tipo':       'rotacion',
            'nombre':     name,
            'target_deg': target_deg,
            'actual_deg': actual_deg,
            'error_deg':  err,
        })

    # ── Resumen final ─────────────────────────────────────────────────────────

    def _print_summary(self, origin_x, origin_y):
        dx       = self.x - origin_x
        dy       = self.y - origin_y
        dist_ret = math.hypot(dx, dy)

        self._log('')
        self._log_sep('━')
        self._log('  RESUMEN DE PRUEBAS — Smart Trolley V5')
        self._log_sep('━')
        self._log(f'  Posición final  : x={self.x:.3f}m  y={self.y:.3f}m  '
                  f'θ={math.degrees(self.theta):.1f}°')
        self._log(f'  Desv. del origen: {dist_ret*100:.1f} cm')
        self._log('')

        for r in self.results:
            if r['tipo'] == 'hall_diag':
                ok_l = '✓' if r['hall_izq_ok'] else '✗'
                ok_r = '✓' if r['hall_der_ok'] else '✗'
                self._log(f'  {ok_l} Hall IZQ  {r["rpm_izq"]:.1f} RPM   '
                          f'{ok_r} Hall DER  {r["rpm_der"]:.1f} RPM')

            elif r['tipo'] == 'lineal':
                sym = '✓' if r['error_pct'] < 8 else '⚠'
                self._log(f'  {sym} {r["nombre"]:12s}  '
                          f'err={r["error_m"]*100:.1f}cm ({r["error_pct"]:.1f}%)')

            elif r['tipo'] == 'rotacion':
                sym = '✓' if abs(r['error_deg']) < 5 else '⚠'
                self._log(f'  {sym} {r["nombre"]:14s}  '
                          f'err={r["error_deg"]:.1f}°')

            elif r['tipo'] == 'square':
                sym = '✓' if r['closure_cm'] < r['side'] * 100 * 0.05 else '⚠'
                self._log(f'  {sym} Cuadrado {r["side"]:.2f}m   '
                          f'cierre={r["closure_cm"]:.1f}cm  '
                          f'heading_err={r["heading_err"]:.1f}°')

        self._log_sep('━')
        self._publish_status('PRUEBAS COMPLETADAS')
        self._add_marker(self.x, self.y,
                         f'FIN dist_orig={dist_ret*100:.0f}cm',
                         C_RED, Marker.CUBE, z=0.12)


def main(args=None):
    rclpy.init(args=args)
    node = MotorHallTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        try:
            node._stop()
        except Exception:
            pass
        try:
            node._log('  Prueba interrumpida por usuario.')
        except Exception:
            pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
