#!/usr/bin/env python3
"""
Joy Evdev Node — Stadia Controller via python-evdev
Usa la librería python-evdev (probada y funcional) con reconexión BT automática.

Mapeo de botones (Stadia BT en Linux):
  buttons[4] = L1 (deadman)   buttons[5] = R1 (speed up)
  buttons[13] = speed down    axes[0/1] = Stick Izq   axes[2/3] = Stick Der

Publica: /joy (sensor_msgs/Joy)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from threading import Thread
import time

import evdev
from evdev import ecodes as e

STADIA_NAMES = ['stadia', 'Stadia']
STADIA_IDS   = [(0x18d1, 0x9400)]

# evdev button code → joy buttons[] index
BUTTON_MAP = {
    e.BTN_A:      0,   # A
    e.BTN_B:      1,   # B
    e.BTN_WEST:   2,   # X
    e.BTN_NORTH:  3,   # Y
    e.BTN_TL:     4,   # L1  ← deadman
    e.BTN_TR:     5,   # R1  ← speed up
    e.BTN_SELECT: 6,   # Select
    e.BTN_START:  7,   # Start
    e.BTN_MODE:   8,   # Stadia
    e.BTN_THUMBL: 9,   # L3
    e.BTN_THUMBR: 10,  # R3
    704: 11,   # Capture
    705: 12,   # Assistant
    706: 13,   # speed down ← btn_speed_down en stadia_teleop
    707: 14,
}

# evdev axis code → (joy_index, invert)
AXIS_MAP = {
    e.ABS_X:     (0, True),   # Stick L X  izq=+1
    e.ABS_Y:     (1, True),   # Stick L Y  arriba=+1
    e.ABS_Z:     (2, True),   # Stick R X  izq=+1
    e.ABS_RZ:    (3, True),   # Stick R Y  arriba=+1
    e.ABS_BRAKE: (4, False),  # L2 trigger reposo=+1
    e.ABS_GAS:   (5, False),  # R2 trigger reposo=+1
    e.ABS_HAT0X: (6, True),   # D-Pad X    izq=+1
    e.ABS_HAT0Y: (7, True),   # D-Pad Y    arriba=+1
}

NUM_AXES    = 8
NUM_BUTTONS = 15


def find_stadia():
    try:
        devices = [evdev.InputDevice(p) for p in evdev.list_devices()]
    except Exception:
        return None
    for d in devices:
        if (d.info.vendor, d.info.product) in STADIA_IDS:
            return d
    for d in devices:
        if any(n.lower() in d.name.lower() for n in STADIA_NAMES):
            return d
    return None


class JoyEvdevNode(Node):

    def __init__(self):
        super().__init__('joy_evdev_node')
        self.declare_parameter('device_path', '')
        self.declare_parameter('autorepeat_rate', 20.0)
        self.declare_parameter('deadzone', 0.12)

        device_path_param = self.get_parameter('device_path').value
        self._rate     = self.get_parameter('autorepeat_rate').value
        self._deadzone = self.get_parameter('deadzone').value

        self._device   = None
        self._abs_info = {}

        if device_path_param:
            self._device = evdev.InputDevice(device_path_param)
        else:
            self._device = find_stadia()
            if self._device is None:
                self.get_logger().error(
                    '✗ Stadia no encontrado. Conéctalo por BT.')
                raise RuntimeError('Stadia not found')

        self._build_abs_info()
        self.get_logger().info(f'✓ Stadia abierto: {self._device.name}')
        self.get_logger().info(f'  Path: {self._device.path}')

        self._axes    = [0.0] * NUM_AXES
        self._buttons = [0]   * NUM_BUTTONS
        self._axes[4] = 1.0   # L2 trigger reposo
        self._axes[5] = 1.0   # R2 trigger reposo

        self._pub = self.create_publisher(Joy, 'joy', 10)
        if self._rate > 0:
            self.create_timer(1.0 / self._rate, self._publish)

        self._running = True
        self._thread  = Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'  deadzone={self._deadzone:.2f}  rate={self._rate:.0f}Hz')
        self.get_logger().info('✓ Joy evdev listo — publicando /joy')

    # ── Leer rangos de ejes del propio dispositivo ────────────────────────
    def _build_abs_info(self):
        self._abs_info = {}
        if not self._device:
            return
        try:
            caps = self._device.capabilities(absinfo=True)
            for code, info in caps.get(e.EV_ABS, []):
                if code in AXIS_MAP:
                    rng = (info.max - info.min) or 1.0
                    self._abs_info[code] = {
                        'min':    info.min,
                        'max':    info.max,
                        'center': (info.min + info.max) / 2.0,
                        'half':   rng / 2.0,
                    }
        except Exception:
            pass

    # ── Normalizar eje a [-1, +1] ─────────────────────────────────────────
    def _norm(self, code, raw):
        if code not in AXIS_MAP:
            return 0.0
        idx, invert = AXIS_MAP[code]
        info = self._abs_info.get(code)

        if code in (e.ABS_BRAKE, e.ABS_GAS):
            vmin = info['min'] if info else 0
            vmax = info['max'] if info else 255
            val  = 1.0 - 2.0 * (raw - vmin) / ((vmax - vmin) or 1.0)
        elif code in (e.ABS_HAT0X, e.ABS_HAT0Y):
            val = -float(raw) if invert else float(raw)
            return max(-1.0, min(1.0, val))
        elif info:
            val = (raw - info['center']) / info['half']
            if invert:
                val = -val
        else:
            val = (raw - 128.0) / 127.0
            if invert:
                val = -val

        if abs(val) < self._deadzone:
            val = 0.0
        return max(-1.0, min(1.0, val))

    # ── Hilo de lectura con reconexión automática ─────────────────────────
    def _read_loop(self):
        while self._running:
            if self._device is None:
                dev = find_stadia()
                if dev is None:
                    self.get_logger().warn('Stadia no encontrado, reintentando...')
                    time.sleep(2.0)
                    continue
                self._device = dev
                self._build_abs_info()
                self._axes[4] = 1.0
                self._axes[5] = 1.0
                self.get_logger().info(f'✓ Stadia reconectado: {dev.name}')

            try:
                for event in self._device.read_loop():
                    if not self._running:
                        return
                    if event.type == e.EV_ABS and event.code in AXIS_MAP:
                        idx, _ = AXIS_MAP[event.code]
                        self._axes[idx] = self._norm(event.code, event.value)
                    elif event.type == e.EV_KEY and event.code in BUTTON_MAP:
                        self._buttons[BUTTON_MAP[event.code]] = 1 if event.value else 0

            except OSError:
                if self._running:
                    self.get_logger().warn('Stadia desconectado, reconectando...')
                try:
                    self._device.close()
                except Exception:
                    pass
                self._device = None
                time.sleep(1.0)
            except Exception as ex:
                if self._running:
                    self.get_logger().error(f'Error: {ex}')
                time.sleep(0.5)

    # ── Publicar /joy ─────────────────────────────────────────────────────
    def _publish(self):
        msg = Joy()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joy'
        msg.axes    = list(self._axes)
        msg.buttons = list(self._buttons)
        self._pub.publish(msg)

    def destroy_node(self):
        self._running = False
        try:
            if self._device:
                self._device.close()
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
    except Exception as ex:
        print(f'Error: {ex}')
    finally:
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
