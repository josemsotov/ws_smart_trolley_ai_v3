#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
  SMART TROLLEY V5 — Diagnóstico ROS2 de Periféricos en Tiempo Real
═══════════════════════════════════════════════════════════════════════════════

Suscribe a todos los topics del sistema y muestra un dashboard terminal
en tiempo real con la tasa de mensajes, valores actuales y estado de cada
periférico.

Periféricos monitorizados:
  ┌─ HARDWARE BASE ─────────────────────────────────────────────────────────┐
  │  Arduino / ros2_control  │ /odom · /joint_states · /arduino_status      │
  │  IMU MPU9250             │ /imu/raw                                      │
  │  GPS                     │ /gps/fix · /gps/status                       │
  ├─ SENSORES ──────────────────────────────────────────────────────────────┤
  │  LiDAR LD06              │ /scan                                        │
  │  Kinect Xbox 360         │ /kinect/rgb · /kinect/depth · /kinect/scan   │
  │  Cámara USB              │ /camera/image_raw                            │
  ├─ AI / VISIÓN ───────────────────────────────────────────────────────────┤
  │  Coral EdgeTPU           │ /coral/detections                            │
  │  Hailo AI HAT2+          │ /hailo/status · /hailo/annotated             │
  ├─ CONTROL ───────────────────────────────────────────────────────────────┤
  │  Stadia Joystick         │ /joy                                         │
  │  cmd_vel / mode          │ /cmd_vel · /robot/mode · /gesture/status     │
  └─────────────────────────────────────────────────────────────────────────┘

Uso:
  # Modo estándar (dashboard 2 Hz)
  ros2 run smart_t_ai_v2 test_ros_diagnostics.py

  # O directamente (con el entorno ROS2 activo)
  python3 tests/hardware/test_ros_diagnostics.py

  # Modo verboso: imprime cada mensaje recibido
  python3 tests/hardware/test_ros_diagnostics.py --verbose

  # Solo mostrar topics activos
  python3 tests/hardware/test_ros_diagnostics.py --active-only

  # Refresh rate personalizado (Hz del dashboard)
  python3 tests/hardware/test_ros_diagnostics.py --rate 5

Requiere: ROS2 Jazzy + source install/setup.bash
═══════════════════════════════════════════════════════════════════════════════
"""

import sys
import os
import time
import math
import argparse
import json
import threading
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import (
    JointState, Imu, NavSatFix, LaserScan,
    Image, Joy, CameraInfo
)
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ─────────────────────────────────────────────────────────────────────────────
#  ANSI helpers
# ─────────────────────────────────────────────────────────────────────────────
R  = '\033[0m'
BD = '\033[1m'
DM = '\033[2m'
RD = '\033[91m'
GR = '\033[92m'
YL = '\033[93m'
BL = '\033[94m'
MG = '\033[95m'
CY = '\033[96m'
WH = '\033[97m'

def clr(text: str, color: str) -> str:
    return f'{color}{text}{R}'

def move_cursor_home():
    sys.stdout.write('\033[H')

def clear_screen():
    sys.stdout.write('\033[2J\033[H')

def _hz(timestamps: deque, window: float = 3.0) -> float:
    """Compute Hz from a deque of timestamps (last `window` seconds)."""
    now = time.monotonic()
    while timestamps and now - timestamps[0] > window:
        timestamps.popleft()
    n = len(timestamps)
    if n < 2:
        return 0.0
    return (n - 1) / (timestamps[-1] - timestamps[0])

def _age(t: float) -> float:
    """Seconds since last message."""
    return time.monotonic() - t if t > 0 else 9999.0

def _status(age: float, hz: float, pub_count: int,
            expected_min: float = 0.5) -> str:
    """Return colored status string."""
    if pub_count == 0:
        return clr('NO PUB ', RD)
    if age > 5.0:
        return clr('MISSING', YL)
    if age > 2.0 or hz < expected_min:
        return clr('STALE  ', YL)
    return clr(' OK    ', GR)

def _bar(val: float, lo: float, hi: float, width: int = 10) -> str:
    """ASCII bar chart."""
    frac = max(0.0, min(1.0, (val - lo) / (hi - lo) if hi != lo else 0.5))
    filled = int(frac * width)
    return f"[{'█' * filled}{'░' * (width - filled)}]"

# ─────────────────────────────────────────────────────────────────────────────
#  Per-topic channel: stores timestamps, latest value, custom extractor
# ─────────────────────────────────────────────────────────────────────────────
class Channel:
    def __init__(self, topic: str, msg_type, expected_hz: float,
                 extractor=None, qos_reliable: bool = False):
        self.topic = topic
        self.msg_type = msg_type
        self.expected_hz = expected_hz
        self.extractor = extractor       # fn(msg) → str
        self.qos_reliable = qos_reliable

        self.ts: deque = deque(maxlen=200)
        self.last_t: float = 0.0
        self.last_val: str = '—'
        self.count: int = 0
        self.pub_count: int = -1   # -1 = unknown yet
        self.lock = threading.Lock()

    def callback(self, msg):
        t = time.monotonic()
        with self.lock:
            self.ts.append(t)
            self.last_t = t
            self.count += 1
            if self.extractor:
                try:
                    self.last_val = self.extractor(msg)
                except Exception:
                    self.last_val = '?'

    @property
    def hz(self) -> float:
        with self.lock:
            return _hz(deque(self.ts))

    @property
    def age(self) -> float:
        with self.lock:
            return _age(self.last_t)

    @property
    def status(self) -> str:
        pc = self.pub_count if self.pub_count >= 0 else 1  # unknown → assume ok
        return _status(self.age, self.hz, pc, self.expected_hz * 0.3)

    @property
    def value(self) -> str:
        with self.lock:
            return self.last_val

# ─────────────────────────────────────────────────────────────────────────────
#  Extractor functions — return a compact 1-line string from a message
# ─────────────────────────────────────────────────────────────────────────────

def _odom(msg: Odometry) -> str:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    # yaw from quaternion
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    vx = msg.twist.twist.linear.x
    wz = msg.twist.twist.angular.z
    return (f'x={p.x:+.3f} y={p.y:+.3f} θ={math.degrees(yaw):+.1f}°  '
            f'vx={vx:+.3f} wz={wz:+.3f}')

def _joint_states(msg: JointState) -> str:
    if not msg.name:
        return '—'
    parts = []
    for n, p, v in zip(msg.name,
                       msg.position if msg.position else [0]*len(msg.name),
                       msg.velocity if msg.velocity else [0]*len(msg.name)):
        short = n.replace('joint_', '')
        parts.append(f'{short}: p={p:.2f} v={v:.2f}')
    return '  '.join(parts)

def _imu(msg: Imu) -> str:
    gz = msg.angular_velocity.z
    ax = msg.linear_acceleration.x
    bar_g = _bar(gz, -3.0, 3.0)
    bar_a = _bar(ax, -20.0, 20.0)
    return f'gz={gz:+.3f}r/s {bar_g}  ax={ax:+.4f}m/s² {bar_a}'

def _gps(msg: NavSatFix) -> str:
    fix = {-1: 'NO_FIX', 0: 'NO_FIX', 1: 'GPS', 2: 'DGPS'}.get(
        msg.status.status, '?')
    clr_fix = GR if msg.status.status >= 1 else RD
    return (f'lat={msg.latitude:.6f} lon={msg.longitude:.6f} '
            f'alt={msg.altitude:.1f}m  [{clr(fix, clr_fix)}]')

def _scan(msg: LaserScan) -> str:
    valid = [r for r in msg.ranges
             if msg.range_min < r < msg.range_max and not math.isinf(r)]
    if not valid:
        return 'no valid ranges'
    mn = min(valid)
    mx = max(valid)
    mean = sum(valid) / len(valid)
    front_idx = len(msg.ranges) // 2
    front = msg.ranges[front_idx]
    front_s = f'{front:.2f}m' if not math.isinf(front) else 'inf'
    bar = _bar(mn, 0.0, 5.0)
    return (f'{len(valid)}/{len(msg.ranges)} pts  '
            f'min={mn:.2f}m mean={mean:.2f}m max={mx:.2f}m  '
            f'front={front_s} {bar}')

def _image(msg: Image) -> str:
    return f'{msg.width}×{msg.height} {msg.encoding}  step={msg.step}'

def _joy(msg: Joy) -> str:
    axes  = [f'{a:+.2f}' for a in msg.axes[:6]]
    btns  = ''.join(str(b) for b in msg.buttons[:12])
    return f'axes=[{",".join(axes)}]  btns={btns}'

def _twist(msg: Twist) -> str:
    bar_v = _bar(msg.linear.x, -0.5, 0.5)
    bar_w = _bar(msg.angular.z, -1.0, 1.0)
    return f'vx={msg.linear.x:+.3f} {bar_v}  wz={msg.angular.z:+.3f} {bar_w}'

def _string(msg: String) -> str:
    s = msg.data.strip()
    return s[:72] if len(s) > 72 else s

def _coral(msg: String) -> str:
    try:
        d = json.loads(msg.data)
        dets = d.get('detections', [])
        if not dets:
            return 'no detections'
        parts = [f"{det['label']}({det['score']:.2f})" for det in dets[:5]]
        return '  '.join(parts)
    except Exception:
        return _string(msg)

def _hailo_status(msg: String) -> str:
    try:
        d = json.loads(msg.data)
        face = d.get('face', 'none')
        gesture = d.get('gesture', 'none')
        mode = d.get('mode', '?')
        return f'face={face}  gesture={gesture}  mode={mode}'
    except Exception:
        return _string(msg)

# ─────────────────────────────────────────────────────────────────────────────
#  Diagnostic Node
# ─────────────────────────────────────────────────────────────────────────────
class RosDiagnosticNode(Node):

    def __init__(self, args: argparse.Namespace):
        super().__init__('trolley_ros_diagnostics')
        self.args = args
        self._start_time = time.monotonic()

        # QoS profiles
        best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5
        )
        reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # ── Define channels ────────────────────────────────────────────────
        self.channels = [

            # ── ODOMETRÍA / CONTROL BASE ───────────────────────────────────
            Channel('/odom',
                    Odometry, expected_hz=50.0,
                    extractor=_odom),
            Channel('/diff_drive_controller/odom',
                    Odometry, expected_hz=50.0,
                    extractor=_odom),
            Channel('/joint_states',
                    JointState, expected_hz=50.0,
                    extractor=_joint_states),
            Channel('/arduino_status',
                    String, expected_hz=1.0,
                    extractor=_string, qos_reliable=True),
            Channel('/cmd_vel',
                    Twist, expected_hz=0.0,   # on-demand
                    extractor=_twist),

            # ── IMU ────────────────────────────────────────────────────────
            Channel('/imu/raw',
                    Imu, expected_hz=10.0,
                    extractor=_imu),

            # ── GPS ────────────────────────────────────────────────────────
            Channel('/gps/fix',
                    NavSatFix, expected_hz=1.0,
                    extractor=_gps),
            Channel('/gps/status',
                    String, expected_hz=1.0,
                    extractor=_string),

            # ── LIDAR ──────────────────────────────────────────────────────
            Channel('/scan',
                    LaserScan, expected_hz=10.0,
                    extractor=_scan),

            # ── KINECT ─────────────────────────────────────────────────────
            Channel('/kinect/rgb/image_raw',
                    Image, expected_hz=15.0,
                    extractor=_image),
            Channel('/kinect/depth/image_raw',
                    Image, expected_hz=15.0,
                    extractor=_image),
            Channel('/kinect/scan',
                    LaserScan, expected_hz=15.0,
                    extractor=_scan),

            # ── CÁMARA USB ─────────────────────────────────────────────────
            Channel('/camera/image_raw',
                    Image, expected_hz=30.0,
                    extractor=_image),

            # ── AI ─────────────────────────────────────────────────────────
            Channel('/coral/detections',
                    String, expected_hz=10.0,
                    extractor=_coral),
            Channel('/hailo/status',
                    String, expected_hz=5.0,
                    extractor=_hailo_status),
            Channel('/hailo/annotated',
                    Image, expected_hz=10.0,
                    extractor=_image),

            # ── CONTROL ────────────────────────────────────────────────────
            Channel('/joy',
                    Joy, expected_hz=20.0,
                    extractor=_joy),
            Channel('/robot/mode',
                    String, expected_hz=0.0,
                    extractor=_string),
            Channel('/gesture/status',
                    String, expected_hz=5.0,
                    extractor=_string),
        ]

        # ── Subscribe ─────────────────────────────────────────────────────
        self._subs = []
        for ch in self.channels:
            qos = best_effort
            sub = self.create_subscription(
                ch.msg_type, ch.topic, ch.callback, qos
            )
            self._subs.append(sub)

        # ── Refresh timer ─────────────────────────────────────────────────
        self._render_lock = threading.Lock()
        interval = 1.0 / max(0.5, min(10.0, args.rate))
        self.create_timer(interval, self._render)
        self.create_timer(2.0, self._update_pub_counts)  # check graph every 2s
        self._first_render = True

        # Initial publisher count (synchronous, before first render)
        self._update_pub_counts()

    # ── Publisher count refresh ───────────────────────────────────────────

    def _update_pub_counts(self):
        for ch in self.channels:
            try:
                ch.pub_count = self.count_publishers(ch.topic)
            except Exception:
                ch.pub_count = -1

    # ── Rendering ─────────────────────────────────────────────────────────

    def _section(self, title: str, width: int = 92) -> str:
        left = f'─── {title} '
        right = '─' * (width - len(left))
        return clr(left + right, CY)

    def _render(self):
        with self._render_lock:
            lines = self._build_dashboard()
            if self._first_render:
                clear_screen()
                self._first_render = False
            else:
                move_cursor_home()
            sys.stdout.write('\n'.join(lines))
            sys.stdout.write('\n')
            sys.stdout.flush()

    def _build_dashboard(self) -> list:
        uptime = time.monotonic() - self._start_time
        now_str = datetime.now().strftime('%H:%M:%S')
        active   = sum(1 for ch in self.channels if ch.age < 3.0)
        n_pubbed = sum(1 for ch in self.channels if ch.pub_count > 0)
        total  = len(self.channels)

        cols = 92
        lines = []

        # ── Header ────────────────────────────────────────────────────────
        lines.append(clr('═' * cols, CY))
        lines.append(
            clr(f'  SMART TROLLEY V5 — DIAGNÓSTICO ROS2 DE PERIFÉRICOS', BD + CY) +
            clr(f'  [{now_str}  up {int(uptime//60)}m{int(uptime%60):02d}s]', DM)
        )
        lines.append(
            f'  Recibiendo: {clr(str(active), GR)}/{total}   '
            f'Con publisher: {clr(str(n_pubbed), CY)}/{total}   '
            f'Presiona {clr("Ctrl+C", YL)} para salir'
        )
        lines.append(clr('═' * cols, CY))

        # ── Column headers ─────────────────────────────────────────────────
        header = (
            f'  {clr("TOPIC", BD):<36}'
            f'  {clr("PUB", BD):>3}'
            f'  {clr("Hz", BD):>5}'
            f'  {clr("AGE(s)", BD):>7}'
            f'  {clr("ESTADO", BD):<14}'
            f'  {clr("VALOR", BD)}'
        )
        lines.append(header)
        lines.append(clr('─' * cols, DM))

        # ── Group sections ─────────────────────────────────────────────────
        groups = [
            ('ODOMETRÍA / CONTROL BASE',
             ['/odom',
              '/diff_drive_controller/odom',
              '/joint_states',
              '/arduino_status',
              '/cmd_vel']),
            ('IMU  MPU9250',
             ['/imu/raw']),
            ('GPS',
             ['/gps/fix', '/gps/status']),
            ('LIDAR  LD06',
             ['/scan']),
            ('KINECT  Xbox 360',
             ['/kinect/rgb/image_raw',
              '/kinect/depth/image_raw',
              '/kinect/scan']),
            ('CÁMARA  USB',
             ['/camera/image_raw']),
            ('AI  Coral EdgeTPU + Hailo',
             ['/coral/detections',
              '/hailo/status',
              '/hailo/annotated']),
            ('CONTROL  Stadia + Modos',
             ['/joy',
              '/robot/mode',
              '/gesture/status']),
        ]

        ch_map = {ch.topic: ch for ch in self.channels}

        for section_title, topics in groups:
            has_active = any(ch_map[t].age < 3.0 for t in topics if t in ch_map)
            if self.args.active_only and not has_active:
                continue

            lines.append(self._section(section_title, cols))

            for topic in topics:
                ch = ch_map.get(topic)
                if ch is None:
                    continue
                if self.args.active_only and ch.age > 3.0:
                    continue

                age  = ch.age
                hz   = ch.hz
                val  = ch.value

                # Hz display
                hz_s = f'{hz:.1f}' if hz > 0.05 else '—'

                # Age display
                if age > 999:
                    age_s = '—'
                else:
                    age_s = f'{age:.2f}'

                # Color the topic name
                pub = ch.pub_count
                if pub == 0:
                    topic_c = clr(f'{topic:<34}', DM)
                    pub_s   = clr(f'{pub:>3}', RD)
                elif age > 2.0:
                    topic_c = clr(f'{topic:<34}', YL)
                    pub_s   = clr(f'{pub:>3}', YL)
                else:
                    topic_c = clr(f'{topic:<34}', GR)
                    pub_s   = clr(f'{pub:>3}', GR)

                # Truncate value to fit
                max_val = cols - 34 - 5 - 7 - 14 - 14
                if len(val) > max_val:
                    val = val[:max_val - 1] + '…'

                line = (
                    f'  {topic_c}'
                    f'  {pub_s}'
                    f'  {hz_s:>5}'
                    f'  {age_s:>7}'
                    f'  {ch.status}'
                    f'  {val}'
                )
                lines.append(line)

        # ── Summary ───────────────────────────────────────────────────────
        lines.append(clr('═' * cols, CY))
        self._append_summary(lines)

        return lines

    def _append_summary(self, lines: list):
        """Add a 1-line summary + odom detail box."""
        # Find odom channel for position summary
        odom_ch = next((c for c in self.channels if c.topic == '/odom'), None)
        r2c_ch  = next((c for c in self.channels
                        if c.topic == '/diff_drive_controller/odom'), None)

        active_odom = odom_ch and odom_ch.age < 3.0
        active_r2c  = r2c_ch  and r2c_ch.age  < 3.0

        if active_odom:
            lines.append(f'  {clr("ODOM:", BD)}  {odom_ch.value}')
        if active_r2c:
            lines.append(f'  {clr("R2C ODOM:", BD)}  {r2c_ch.value}')

        # Active nodes count summary
        cats = {
            'Base': ['/odom', '/joint_states'],
            'IMU': ['/imu/raw'],
            'GPS': ['/gps/fix'],
            'LiDAR': ['/scan'],
            'Kinect': ['/kinect/rgb/image_raw', '/kinect/depth/image_raw'],
            'Camera': ['/camera/image_raw'],
            'Coral': ['/coral/detections'],
            'Hailo': ['/hailo/status'],
            'Joy': ['/joy'],
        }
        ch_map = {ch.topic: ch for ch in self.channels}
        parts = []
        for label, topics in cats.items():
            receiving = any(ch_map.get(t) and ch_map[t].age < 3.0 for t in topics)
            has_pub   = any(ch_map.get(t) and ch_map[t].pub_count > 0 for t in topics)
            if receiving:
                color = GR
            elif has_pub:
                color = YL
            else:
                color = DM
            parts.append(clr(label, color))

        lines.append('  ' + '  │  '.join(parts))
        lines.append(clr('─' * 92, DM))
        lines.append(
            f'  {clr("Msgs recibidos:", DM)} ' +
            '  '.join(
                f'{ch.topic.split("/")[-1]}={ch.count}'
                for ch in self.channels if ch.count > 0
            )[:85]
        )


# ─────────────────────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Smart Trolley V5 — Diagnóstico ROS2 de Periféricos')
    parser.add_argument('--rate', type=float, default=2.0,
                        help='Frecuencia del dashboard (Hz, default 2)')
    parser.add_argument('--active-only', action='store_true',
                        help='Solo mostrar topics con datos recientes')
    parser.add_argument('--verbose', action='store_true',
                        help='Mostrar log extra por cada mensaje')
    args = parser.parse_args()

    rclpy.init()
    node = RosDiagnosticNode(args)

    clear_screen()
    print(clr('\n  Iniciando diagnóstico ROS2... esperando topics\n', CY))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    print(clr('\n  Diagnóstico terminado.\n', CY))


if __name__ == '__main__':
    main()
