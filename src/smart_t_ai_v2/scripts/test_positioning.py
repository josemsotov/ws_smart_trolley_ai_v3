#!/usr/bin/env python3
"""
test_positioning.py — Smart Trolley V5
Prueba de precisión de posicionamiento por odometría.

Secuencia de pruebas:
  1. Línea recta    → avanza D metros, mide error de distancia
  2. Viaje de vuelta → gira 180°, regresa, mide error de retorno
  3. Cuadrado        → 4 × (avanza D + gira 90° izq), mide error de cierre
  4. Reporte final   → tabla de errores (comandado vs real) + PPR efectivo

Publicaciones RViz:
  /positioning_test/path    nav_msgs/Path            — trayectoria real
  /positioning_test/markers visualization_msgs/MA    — waypoints objetivo
  /positioning_test/target  geometry_msgs/PointStamped — posición objetivo
  /positioning_test/status  std_msgs/String           — estado actual

Uso directo:
  ros2 run smart_t_ai_v2 test_positioning.py

Con launch:
  ros2 launch smart_t_ai_v2 positioning_test.launch.py

Parámetros:
  test_distance   (float, 1.0 m)
  linear_speed    (float, 0.20 m/s)
  angular_speed   (float, 0.50 rad/s)
  auto_delay      (float, 3.0 s)   — espera antes de iniciar
  run_return      (bool,  True)     — prueba de vuelta
  run_square      (bool,  True)     — prueba de cuadrado
  use_gps         (bool,  False)    — registrar waypoints GPS si disponible
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import math
import time
import threading
from dataclasses import dataclass, field
from typing import Optional

# ─────────────────────────────────────────────────────────────────────────────
#  ANSI helpers (para reporte en terminal)
# ─────────────────────────────────────────────────────────────────────────────
R  = '\033[0m'
BD = '\033[1m'
GR = '\033[92m'
YL = '\033[93m'
RD = '\033[91m'
CY = '\033[96m'
DM = '\033[2m'


def c(s, col): return f'{col}{s}{R}'


# ─────────────────────────────────────────────────────────────────────────────
#  Structures
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # rad

    def dist_to(self, other: 'Pose2D') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def __repr__(self):
        return (f'x={self.x:+.4f} y={self.y:+.4f} '
                f'θ={math.degrees(self.theta):+.2f}°')


@dataclass
class MoveResult:
    label: str
    commanded: float       # metros o grados
    actual: float          # medido desde odometría
    error: float           # actual − commanded
    error_pct: float       # |error| / |commanded| * 100
    duration: float        # segundos
    start: Pose2D = field(default_factory=Pose2D)
    end:   Pose2D = field(default_factory=Pose2D)
    gps_start: Optional[tuple] = None   # (lat, lon) si disponible
    gps_end:   Optional[tuple] = None

    def ok(self) -> bool:
        return abs(self.error_pct) < 10.0

    def status_str(self) -> str:
        col = GR if self.ok() else (YL if abs(self.error_pct) < 25 else RD)
        return c(f'{self.error_pct:+.1f}%', col)


# ─────────────────────────────────────────────────────────────────────────────
#  Main test node
# ─────────────────────────────────────────────────────────────────────────────

class PositioningTest(Node):

    def __init__(self):
        super().__init__('positioning_test')

        # ── Parámetros ───────────────────────────────────────────────────────
        self.declare_parameter('test_distance',  1.0)
        self.declare_parameter('linear_speed',   0.20)
        self.declare_parameter('angular_speed',  0.50)
        self.declare_parameter('auto_delay',     3.0)
        self.declare_parameter('run_return',     True)
        self.declare_parameter('run_square',     True)
        self.declare_parameter('use_gps',        False)

        self.dist      = self.get_parameter('test_distance').value
        self.lin_spd   = self.get_parameter('linear_speed').value
        self.ang_spd   = self.get_parameter('angular_speed').value
        self.delay     = self.get_parameter('auto_delay').value
        self.do_return = self.get_parameter('run_return').value
        self.do_square = self.get_parameter('run_square').value
        self.use_gps   = self.get_parameter('use_gps').value

        # ── Estado de odometría ──────────────────────────────────────────────
        self._pose   = Pose2D()
        self._pose_lock = threading.Lock()
        self._odom_ready = threading.Event()

        # ── GPS ──────────────────────────────────────────────────────────────
        self._gps: Optional[tuple] = None
        self._gps_lock = threading.Lock()

        # ── QoS ──────────────────────────────────────────────────────────────
        be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # ── Suscriptores ──────────────────────────────────────────────────────
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self._cb_odom, be)
        if self.use_gps:
            self.sub_gps = self.create_subscription(
                NavSatFix, '/gps/fix', self._cb_gps, be)

        # ── Publicadores ──────────────────────────────────────────────────────
        self.pub_vel    = self.create_publisher(Twist,       '/cmd_vel',                   10)
        self.pub_path   = self.create_publisher(Path,        '/positioning_test/path',     10)
        self.pub_marks  = self.create_publisher(MarkerArray, '/positioning_test/markers',  10)
        self.pub_target = self.create_publisher(PointStamped,'/positioning_test/target',   10)
        self.pub_status = self.create_publisher(String,      '/positioning_test/status',   10)

        # ── Path message (se va rellenando) ───────────────────────────────────
        self._path = Path()
        self._path.header.frame_id = 'odom'
        self._markers = MarkerArray()
        self._marker_id = 0

        # ── Resultados ────────────────────────────────────────────────────────
        self.results: list[MoveResult] = []

        # ── Arrancar en hilo separado ──────────────────────────────────────────
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ─────────────────────────────────────────────────────────────────────────
    #  Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny, cosy)
        with self._pose_lock:
            self._pose = Pose2D(p.x, p.y, theta)
        self._odom_ready.set()
        # Actualizar path en RViz
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'odom'
        ps.pose = msg.pose.pose
        self._path.header.stamp = ps.header.stamp
        self._path.poses.append(ps)
        if len(self._path.poses) % 5 == 0:    # publicar cada 5 puntos
            self.pub_path.publish(self._path)

    def _cb_gps(self, msg: NavSatFix):
        if msg.status.status >= 0:
            with self._gps_lock:
                self._gps = (msg.latitude, msg.longitude)

    # ─────────────────────────────────────────────────────────────────────────
    #  Helpers de pose y estado
    # ─────────────────────────────────────────────────────────────────────────

    def _get_pose(self) -> Pose2D:
        with self._pose_lock:
            return Pose2D(self._pose.x, self._pose.y, self._pose.theta)

    def _get_gps(self) -> Optional[tuple]:
        if not self.use_gps:
            return None
        with self._gps_lock:
            return self._gps

    def _status(self, msg: str):
        self.get_logger().info(msg)
        s = String(); s.data = msg
        self.pub_status.publish(s)

    def _stop(self):
        self.pub_vel.publish(Twist())
        time.sleep(0.15)

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Diferencia angular normalizada a [-π, π]."""
        d = a - b
        while d > math.pi:  d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return d

    # ─────────────────────────────────────────────────────────────────────────
    #  Primitivas de movimiento
    # ─────────────────────────────────────────────────────────────────────────

    def move_forward(self, distance: float,
                     speed: Optional[float] = None) -> MoveResult:
        """Mueve el robot `distance` metros (negativo = retrocede)."""
        spd = speed if speed is not None else self.lin_spd
        if distance < 0:
            spd = -spd
        target_dist = abs(distance)
        label = f'Avance {distance:+.2f}m'

        start_pose = self._get_pose()
        gps_start  = self._get_gps()
        t0 = time.monotonic()

        rate_hz = 20
        dt = 1.0 / rate_hz
        timeout = target_dist / abs(spd) * 3.0 + 5.0  # 3× el tiempo esperado

        self._publish_target(start_pose.x + math.cos(start_pose.theta) * distance,
                             start_pose.y + math.sin(start_pose.theta) * distance)
        self._status(f'▶  {label}  (speed={spd:+.3f} m/s)')

        cmd = Twist()
        traveled = 0.0

        while True:
            pose = self._get_pose()
            traveled = pose.dist_to(start_pose)

            remaining = target_dist - traveled
            if remaining <= 0.02:   # ≤ 2 cm: listo
                break
            if time.monotonic() - t0 > timeout:
                self.get_logger().warn(f'Timeout en {label}')
                break

            # Control proporcional de velocidad
            p_speed = min(abs(spd), max(0.05, abs(spd) * (remaining / 0.10))) * math.copysign(1, spd)
            cmd.linear.x  = float(p_speed)
            cmd.angular.z = 0.0
            self.pub_vel.publish(cmd)
            time.sleep(dt)

        self._stop()
        end_pose  = self._get_pose()
        gps_end   = self._get_gps()
        actual    = end_pose.dist_to(start_pose) * math.copysign(1, distance)
        duration  = time.monotonic() - t0
        error     = actual - distance
        error_pct = abs(error / distance * 100) if distance != 0 else 0.0

        r = MoveResult(label, distance, actual, error, error_pct,
                       duration, start_pose, end_pose, gps_start, gps_end)
        self.results.append(r)
        self._add_waypoint_marker(end_pose, label, r.ok())
        self._log_result(r)
        return r

    def rotate(self, degrees: float,
               speed: Optional[float] = None) -> MoveResult:
        """Rota el robot `degrees` grados (+ = izquierda, - = derecha)."""
        spd = speed if speed is not None else self.ang_spd
        angle_rad = math.radians(degrees)
        label = f'Giro {degrees:+.1f}°'

        start_pose = self._get_pose()
        target_theta = start_pose.theta + angle_rad
        t0 = time.monotonic()
        timeout = abs(angle_rad) / abs(spd) * 3.0 + 5.0

        self._status(f'↺  {label}  (speed={spd:+.3f} rad/s)')

        cmd = Twist()

        while True:
            pose   = self._get_pose()
            diff   = self._angle_diff(target_theta, pose.theta)

            if abs(diff) < math.radians(2.0):   # ≤ 2°: listo
                break
            if time.monotonic() - t0 > timeout:
                self.get_logger().warn(f'Timeout en {label}')
                break

            p_spd = math.copysign(
                min(abs(spd), max(0.10, abs(spd) * abs(diff) / math.radians(15))),
                diff
            )
            cmd.angular.z = float(p_spd)
            cmd.linear.x  = 0.0
            self.pub_vel.publish(cmd)
            time.sleep(1.0 / 20)

        self._stop()
        end_pose  = self._get_pose()
        actual_deg = math.degrees(self._angle_diff(end_pose.theta, start_pose.theta))
        duration   = time.monotonic() - t0
        error      = actual_deg - degrees
        error_pct  = abs(error / degrees * 100) if degrees != 0 else 0.0

        r = MoveResult(label, degrees, actual_deg, error, error_pct,
                       duration, start_pose, end_pose)
        self.results.append(r)
        self._log_result(r)
        return r

    # ─────────────────────────────────────────────────────────────────────────
    #  Secuencias de prueba
    # ─────────────────────────────────────────────────────────────────────────

    def _test_straight(self):
        """Test 1: Línea recta adelante."""
        self._status(f'═══ TEST 1: LÍNEA RECTA ({self.dist} m) ═══')
        origin = self._get_pose()
        self._add_waypoint_marker(origin, 'Origen', True, color=(0.0, 0.8, 1.0))
        self.move_forward(self.dist)

    def _test_return(self):
        """Test 2: Vuelta al origen (180° + misma distancia)."""
        self._status(f'═══ TEST 2: VIAJE DE VUELTA ═══')
        origin_before_turn = self._get_pose()
        self.rotate(180.0)
        time.sleep(0.5)
        self.move_forward(self.dist)
        end = self._get_pose()

        # Error de cierre del viaje ida+vuelta
        close_err = end.dist_to(Pose2D(0.0, 0.0))   # distancia al origen original
        self._status(f'↩  Error de cierre: {close_err*100:.1f} cm  ({end})')
        self._add_waypoint_marker(end, f'Cierre {close_err*100:.1f}cm',
                                  close_err < 0.05, color=(1.0, 0.5, 0.0))

    def _test_square(self):
        """Test 3: Cuadrado (4 lados + 4 giros de 90°)."""
        self._status(f'═══ TEST 3: CUADRADO {self.dist}m × {self.dist}m ═══')
        start = self._get_pose()
        for i in range(4):
            self.move_forward(self.dist)
            time.sleep(0.4)
            self.rotate(90.0)
            time.sleep(0.4)
        end = self._get_pose()

        close_err  = end.dist_to(start)
        heading_err = abs(math.degrees(
            self._angle_diff(end.theta, start.theta)))
        self._status(
            f'□  Error de cierre: {close_err*100:.1f} cm  '
            f'|  Error de heading: {heading_err:.1f}°  |  {end}')
        self._add_waypoint_marker(end,
            f'Cierre □ {close_err*100:.1f}cm/{heading_err:.1f}°',
            close_err < 0.10, color=(1.0, 0.9, 0.0))

    # ─────────────────────────────────────────────────────────────────────────
    #  Thread principal
    # ─────────────────────────────────────────────────────────────────────────

    def _run(self):
        # Esperar odometría
        self._status('Esperando odometría…')
        if not self._odom_ready.wait(timeout=15.0):
            self.get_logger().error('Odometría no disponible. Abortando.')
            return

        # Delay inicial
        self._status(f'Iniciando en {self.delay:.0f}s…')
        time.sleep(self.delay)

        t_total = time.monotonic()

        # ── Ejecutar secuencia ────────────────────────────────────────────────
        self._test_straight()
        time.sleep(1.0)

        if self.do_return:
            self._test_return()
            time.sleep(1.0)

        if self.do_square:
            self._test_square()
            time.sleep(1.0)

        self._stop()
        total_time = time.monotonic() - t_total

        # ── Publicar path final ───────────────────────────────────────────────
        self.pub_path.publish(self._path)
        self.pub_marks.publish(self._markers)

        # ── Imprimir reporte ──────────────────────────────────────────────────
        time.sleep(0.5)
        self._print_report(total_time)

    # ─────────────────────────────────────────────────────────────────────────
    #  Reporte
    # ─────────────────────────────────────────────────────────────────────────

    def _log_result(self, r: MoveResult):
        col = GR if r.ok() else (YL if abs(r.error_pct) < 25 else RD)
        unit = '°' if '°' in r.label else 'm'
        self.get_logger().info(
            f'  {r.label:<20}  cmd={r.commanded:+.3f}{unit}  '
            f'real={r.actual:+.3f}{unit}  '
            f'err={r.error:+.3f}{unit}  [{c(f"{r.error_pct:.1f}%", col)}]  '
            f'{r.duration:.2f}s'
        )

    def _print_report(self, total_time: float):
        w = 78
        print()
        print(c('═' * w, CY))
        print(c('  SMART TROLLEY V5 — REPORTE PRUEBA DE POSICIONAMIENTO', BD + CY))
        print(c('═' * w, CY))
        print(f'  Distancia de prueba : {self.dist} m')
        print(f'  Vel. lineal          : {self.lin_spd} m/s')
        print(f'  Vel. angular         : {self.ang_spd} rad/s')
        print(f'  Tiempo total         : {total_time:.1f} s')
        print(c('─' * w, DM))

        # Cabecera tabla
        print(f'  {c("PRUEBA", BD):<26}'
              f'  {c("COMANDADO", BD):>12}'
              f'  {c("REAL", BD):>12}'
              f'  {c("ERROR", BD):>10}'
              f'  {c("%ERR", BD):>8}'
              f'  {c("TIEMPO", BD):>7}')
        print(c('─' * w, DM))

        for r in self.results:
            unit = '°' if '°' in r.label or 'Giro' in r.label else 'm'
            print(f'  {r.label:<24}'
                  f'  {r.commanded:>10.3f}{unit}'
                  f'  {r.actual:>10.3f}{unit}'
                  f'  {r.error:>+8.3f}{unit}'
                  f'  {r.status_str():>8}'
                  f'  {r.duration:>5.2f}s')

        print(c('─' * w, DM))

        # Resumen global
        lin_results = [r for r in self.results if 'Avance' in r.label]
        rot_results = [r for r in self.results if 'Giro' in r.label]

        if lin_results:
            avg_lin = sum(abs(r.error_pct) for r in lin_results) / len(lin_results)
            print(f'  Error lineal promedio : {avg_lin:.1f}%')
        if rot_results:
            avg_rot = sum(abs(r.error_pct) for r in rot_results) / len(rot_results)
            print(f'  Error angular promedio: {avg_rot:.1f}%')

        # Calidad global
        n_ok = sum(1 for r in self.results if r.ok())
        n_total = len(self.results)
        pct = n_ok / n_total * 100 if n_total else 0
        col = GR if pct >= 80 else (YL if pct >= 50 else RD)
        print(c('═' * w, CY))
        print(f'  Precisión global: {c(f"{n_ok}/{n_total} pruebas OK ({pct:.0f}%)", col)}')

        # GPS si disponible
        if self.use_gps:
            gps_results = [(r.label, r.gps_start, r.gps_end)
                           for r in self.results
                           if r.gps_start and r.gps_end]
            if gps_results:
                print(c('─' * w, DM))
                print(f'  {c("GPS WAYPOINTS:", BD)}')
                for label, gs, ge in gps_results:
                    print(f'    {label}: inicio=({gs[0]:.6f},{gs[1]:.6f})'
                          f'  fin=({ge[0]:.6f},{ge[1]:.6f})')

        print(c('═' * w, CY))
        print()
        self._status('PRUEBA COMPLETADA — ver reporte en terminal')

    # ─────────────────────────────────────────────────────────────────────────
    #  Helpers de visualización RViz
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_target(self, x: float, y: float):
        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = 'odom'
        pt.point.x = x
        pt.point.y = y
        self.pub_target.publish(pt)

    def _add_waypoint_marker(self, pose: Pose2D, label: str, ok: bool,
                             color: tuple = None):
        """Agrega una esfera + texto al MarkerArray."""
        if color is None:
            color = (0.1, 0.9, 0.1) if ok else (1.0, 0.3, 0.1)

        stamp = self.get_clock().now().to_msg()

        # Esfera
        sphere = Marker()
        sphere.header.stamp     = stamp
        sphere.header.frame_id  = 'odom'
        sphere.ns               = 'waypoints'
        sphere.id               = self._marker_id
        self._marker_id        += 1
        sphere.type             = Marker.SPHERE
        sphere.action           = Marker.ADD
        sphere.pose.position.x  = pose.x
        sphere.pose.position.y  = pose.y
        sphere.pose.position.z  = 0.05
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.12
        sphere.color.r, sphere.color.g, sphere.color.b = color
        sphere.color.a = 0.9
        sphere.lifetime.sec = 0   # permanente
        self._markers.markers.append(sphere)

        # Texto
        txt = Marker()
        txt.header.stamp     = stamp
        txt.header.frame_id  = 'odom'
        txt.ns               = 'labels'
        txt.id               = self._marker_id
        self._marker_id     += 1
        txt.type             = Marker.TEXT_VIEW_FACING
        txt.action           = Marker.ADD
        txt.pose.position.x  = pose.x
        txt.pose.position.y  = pose.y
        txt.pose.position.z  = 0.25
        txt.pose.orientation.w = 1.0
        txt.scale.z          = 0.09
        txt.color.r, txt.color.g, txt.color.b = (1.0, 1.0, 1.0)
        txt.color.a          = 1.0
        txt.text             = label
        txt.lifetime.sec     = 0
        self._markers.markers.append(txt)

        self.pub_marks.publish(self._markers)


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = PositioningTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
