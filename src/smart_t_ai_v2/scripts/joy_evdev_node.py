#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════
  Joy Evdev Node — Stadia Controller via evdev
═══════════════════════════════════════════════════════════

Reemplazo de ros2 joy_node que lee directamente de /dev/input/eventX
via la API evdev de Linux, evitando el bug del driver joydev (js0) que
deja de enviar eventos tras reconexiones BT del Stadia.

Publica /joy con el MISMO formato que joy_node estándar para ser
100% compatible con stadia_teleop_node.py:

  axes[0] = Stick Izq X  (izq=+1, der=-1)
  axes[1] = Stick Izq Y  (arriba=+1, abajo=-1)
  axes[2] = Stick Der X  (izq=+1, der=-1)
  axes[3] = Stick Der Y  (arriba=+1, abajo=-1)
  axes[4] = Trigger L2   (reposo=+1, presionado=-1)
  axes[5] = Trigger R2   (reposo=+1, presionado=-1)
  axes[6] = D-Pad X      (izq=+1, der=-1)
  axes[7] = D-Pad Y      (arriba=+1, abajo=-1)

  buttons[0]  = A          buttons[1]  = B
  buttons[2]  = X          buttons[3]  = Y
  buttons[4]  = L1         buttons[5]  = R1
  buttons[6]  = Select     buttons[7]  = Start
  buttons[8]  = Stadia     buttons[9]  = L3
  buttons[10] = R3         buttons[11] = Capture
  buttons[12] = Assistant  buttons[13] = TriggerHappy3
  buttons[14] = TriggerHappy4

Suscribe: /dev/input/eventX  (evdev kernel interface)
Publica:  /joy               (sensor_msgs/Joy)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from threading import Thread
import os
import struct
import select
import glob
import time


# ── evdev constants ──
EV_SYN = 0x00
EV_KEY = 0x01
EV_ABS = 0x03

# Axis codes
ABS_X      = 0x00   # Left Stick X
ABS_Y      = 0x01   # Left Stick Y
ABS_Z      = 0x02   # Right Stick X
ABS_RZ     = 0x05   # Right Stick Y
ABS_GAS    = 0x09   # Right Trigger (R2)
ABS_BRAKE  = 0x0A   # Left Trigger (L2)
ABS_HAT0X  = 0x10   # D-Pad X
ABS_HAT0Y  = 0x11   # D-Pad Y

# Button codes (Stadia specific)
BTN_A           = 304
BTN_B           = 305
BTN_NORTH       = 307   # Y
BTN_WEST        = 308   # X
BTN_TL          = 310   # L1
BTN_TR          = 311   # R1
BTN_SELECT      = 314
BTN_START       = 315
BTN_MODE        = 316   # Stadia button
BTN_THUMBL      = 317   # L3
BTN_THUMBR      = 318   # R3
BTN_TRIGGER_HAPPY  = 704  # Capture
BTN_TRIGGER_HAPPY2 = 705  # Assistant
BTN_TRIGGER_HAPPY3 = 706
BTN_TRIGGER_HAPPY4 = 707


def _find_stadia_event() -> str:
    """Auto-detect Stadia controller event device."""
    for path in sorted(glob.glob('/sys/class/input/event*/device/name')):
        try:
            name = open(path).read().strip()
            if 'Stadia' in name or 'stadia' in name:
                evnum = path.split('/event')[1].split('/')[0]
                return f'/dev/input/event{evnum}'
        except Exception:
            continue
    return ''


class JoyEvdevNode(Node):

    # Stadia axes: evdev_code → (joy_index, min, max, invert)
    # Sticks: range [1,255], center=128
    # Triggers: range [0,255], resting=0
    # D-Pad: range [-1,1], discrete
    AXIS_MAP = {
        ABS_X:     (0, 1, 255, True),    # Stick L X: izq=+1 (invert)
        ABS_Y:     (1, 1, 255, True),    # Stick L Y: arriba=+1 (invert)
        ABS_Z:     (2, 1, 255, True),    # Stick R X: izq=+1 (invert)
        ABS_RZ:    (3, 1, 255, True),    # Stick R Y: arriba=+1 (invert)
        ABS_BRAKE: (4, 0, 255, False),   # L2 trigger: reposo=+1
        ABS_GAS:   (5, 0, 255, False),   # R2 trigger: reposo=+1
        ABS_HAT0X: (6, -1, 1, True),    # D-Pad X: izq=+1 (invert)
        ABS_HAT0Y: (7, -1, 1, True),    # D-Pad Y: arriba=+1 (invert)
    }

    # Stadia buttons: evdev_code → joy_index
    # Matches the mapping that stadia_teleop_node.py expects:
    #   btn 0=A, 1=B, 2=X, 3=Y, 4=L1, 5=R1, ...
    BUTTON_MAP = {
        BTN_A:              0,   # A
        BTN_B:              1,   # B
        BTN_WEST:           2,   # X  (evdev WEST = physical X on Stadia)
        BTN_NORTH:          3,   # Y  (evdev NORTH = physical Y on Stadia)
        BTN_TL:             4,   # L1
        BTN_TR:             5,   # R1
        BTN_SELECT:         6,   # Select / ···
        BTN_START:          7,   # Start / ≡
        BTN_MODE:           8,   # Stadia button
        BTN_THUMBL:         9,   # L3
        BTN_THUMBR:        10,   # R3
        BTN_TRIGGER_HAPPY: 11,   # Capture
        BTN_TRIGGER_HAPPY2:12,   # Assistant
        BTN_TRIGGER_HAPPY3:13,
        BTN_TRIGGER_HAPPY4:14,
    }

    NUM_AXES = 8
    NUM_BUTTONS = 15

    def __init__(self):
        super().__init__('joy_evdev_node')

        # ── Parameters ──
        self.declare_parameter('device_path', '')
        self.declare_parameter('autorepeat_rate', 20.0)
        self.declare_parameter('deadzone', 0.12)

        device_path = self.get_parameter('device_path').value
        self._rate = self.get_parameter('autorepeat_rate').value
        self._deadzone = self.get_parameter('deadzone').value

        # Auto-detect if not specified
        if not device_path:
            device_path = _find_stadia_event()
            if not device_path:
                self.get_logger().error(
                    '✗ No se encontró el Stadia. Conecta el control por BT.')
                raise RuntimeError('Stadia controller not found')

        # Open device
        try:
            self._fd = os.open(device_path, os.O_RDONLY | os.O_NONBLOCK)
            self.get_logger().info(f'✓ Joystick abierto: {device_path}')
        except Exception as e:
            self.get_logger().error(f'✗ Error abriendo {device_path}: {e}')
            raise

        # Read device name from sysfs
        try:
            evnum = device_path.split('event')[1]
            name = open(f'/sys/class/input/event{evnum}/device/name').read().strip()
            self.get_logger().info(f'  Device: {name}')
        except Exception:
            pass

        # ── State ──
        self._axes = [0.0] * self.NUM_AXES
        # Triggers start at +1.0 (resting position)
        self._axes[4] = 1.0  # L2
        self._axes[5] = 1.0  # R2
        self._buttons = [0] * self.NUM_BUTTONS

        # ── Publisher ──
        self._pub = self.create_publisher(Joy, 'joy', 10)

        # ── Publish timer ──
        if self._rate > 0:
            self.create_timer(1.0 / self._rate, self._publish)

        # ── Event reader thread ──
        self._running = True
        self._thread = Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'  deadzone: {self._deadzone:.3f}  rate: {self._rate:.0f} Hz')
        self.get_logger().info('✓ Joy evdev node listo — publicando en /joy')

    # ─────────────────────────────────────────────
    #  Normalize axis value to [-1.0, +1.0]
    # ─────────────────────────────────────────────
    def _normalize(self, code: int, raw: int) -> float:
        if code not in self.AXIS_MAP:
            return 0.0
        idx, vmin, vmax, invert = self.AXIS_MAP[code]

        # Triggers: resting=0 → +1.0, fully pressed=255 → -1.0
        if code in (ABS_GAS, ABS_BRAKE):
            val = 1.0 - 2.0 * (raw - vmin) / (vmax - vmin)
        # D-Pad: discrete -1/0/+1
        elif code in (ABS_HAT0X, ABS_HAT0Y):
            val = float(raw)
            if invert:
                val = -val
            return val
        # Sticks: center at 128, range [1, 255]
        else:
            center = (vmin + vmax) / 2.0
            half = (vmax - vmin) / 2.0
            val = (raw - center) / half
            if invert:
                val = -val

        # Apply deadzone
        if abs(val) < self._deadzone:
            val = 0.0

        return max(-1.0, min(1.0, val))

    # ─────────────────────────────────────────────
    #  Read events from kernel (background thread)
    # ─────────────────────────────────────────────
    def _read_loop(self):
        # input_event struct: uint64 sec, uint64 usec, uint16 type, uint16 code, int32 value
        EVENT_SIZE = 24
        EVENT_FMT = 'llHHi'

        while self._running:
            try:
                r, _, _ = select.select([self._fd], [], [], 0.1)
                if not r:
                    continue
                data = os.read(self._fd, EVENT_SIZE * 16)
                offset = 0
                while offset + EVENT_SIZE <= len(data):
                    _sec, _usec, ev_type, ev_code, ev_value = \
                        struct.unpack_from(EVENT_FMT, data, offset)
                    offset += EVENT_SIZE

                    if ev_type == EV_ABS and ev_code in self.AXIS_MAP:
                        idx = self.AXIS_MAP[ev_code][0]
                        self._axes[idx] = self._normalize(ev_code, ev_value)

                    elif ev_type == EV_KEY and ev_code in self.BUTTON_MAP:
                        idx = self.BUTTON_MAP[ev_code]
                        self._buttons[idx] = 1 if ev_value else 0

            except OSError:
                if self._running:
                    self.get_logger().warn('Joystick desconectado, reintentando...')
                    time.sleep(1.0)
            except Exception as e:
                if self._running:
                    self.get_logger().error(f'Error leyendo eventos: {e}')
                    time.sleep(0.5)

    # ─────────────────────────────────────────────
    #  Publish Joy message (timer callback)
    # ─────────────────────────────────────────────
    def _publish(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joy'
        msg.axes = list(self._axes)
        msg.buttons = list(self._buttons)
        self._pub.publish(msg)

    # ─────────────────────────────────────────────
    #  Cleanup
    # ─────────────────────────────────────────────
    def destroy_node(self):
        self._running = False
        try:
            os.close(self._fd)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = JoyEvdevNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
