#!/usr/bin/env python3
"""
ROS2 Kinect v1 Node using libfreenect (ctypes)

Publishes:
  /kinect/rgb/image_raw      (sensor_msgs/Image)        — 640x480 RGB @ 15Hz
  /kinect/depth/image_raw    (sensor_msgs/Image)        — 640x480 16UC1 depth @ 15Hz
  /kinect/rgb/camera_info    (sensor_msgs/CameraInfo)   — RGB intrinsics
  /kinect/depth/camera_info  (sensor_msgs/CameraInfo)   — Depth intrinsics

Tilt control via /joy topic:
  Button Y (btn 3) = toggle tilt control mode
  Right stick Y (axis 3) = tilt up/down while Y held or toggled

Frames:
  RGB:   kinect_rgb_optical
  Depth: kinect_depth_optical

Requires: libfreenect0.5 (apt install freenect)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Joy
from builtin_interfaces.msg import Time

import ctypes
import ctypes.util
import numpy as np
import threading
import time


# ═══════════════════════════════════════════════════════════
# libfreenect ctypes bindings (minimal subset)
# ═══════════════════════════════════════════════════════════

_lib = ctypes.CDLL('libfreenect.so')
_sync = ctypes.CDLL('libfreenect_sync.so')

# freenect_sync_get_video(void **video, uint32_t *timestamp, int index, int fmt)
_sync.freenect_sync_get_video.argtypes = [
    ctypes.POINTER(ctypes.c_void_p),  # video
    ctypes.POINTER(ctypes.c_uint32),   # timestamp
    ctypes.c_int,                      # index
    ctypes.c_int,                      # format
]
_sync.freenect_sync_get_video.restype = ctypes.c_int

# freenect_sync_get_depth(void **depth, uint32_t *timestamp, int index, int fmt)
_sync.freenect_sync_get_depth.argtypes = [
    ctypes.POINTER(ctypes.c_void_p),  # depth
    ctypes.POINTER(ctypes.c_uint32),   # timestamp
    ctypes.c_int,                      # index
    ctypes.c_int,                      # format
]
_sync.freenect_sync_get_depth.restype = ctypes.c_int

# freenect_sync_stop()
_sync.freenect_sync_stop.argtypes = []
_sync.freenect_sync_stop.restype = None

# Constants
FREENECT_VIDEO_RGB = 0           # 640x480 RGB (3 bytes/pixel)
FREENECT_DEPTH_11BIT = 0        # 640x480 11-bit depth
FREENECT_DEPTH_REGISTERED = 4   # 640x480 registered depth (mm)

WIDTH = 640
HEIGHT = 480

# Tilt calibration for our mount:
#   Motor command 35° = physically horizontal (recto)
#   Motor hardware range: -27° to 35° (command values)
#   Logical range (user-facing): 0° = recto, negative = mira abajo
#     0°  → motor 35° (horizontal, tope arriba)
#   -62°  → motor -27° (máximo abajo, tope hardware)
KINECT_TILT_OFFSET = 35.0
KINECT_TILT_MAX = 0.0     # Logical: recto (motor 35°)
KINECT_TILT_MIN = -62.0   # Logical: max abajo (motor -27°)


# ═══════════════════════════════════════════════════════════
# Motor tilt control via pyusb (direct USB control transfer)
# ═══════════════════════════════════════════════════════════
# The freenect sync API (for video/depth) claims ALL USB interfaces
# including the motor, so libfreenect-based motor control always fails
# with "Could not claim interface on motor: -6".
#
# Solution: use pyusb to talk directly to the Kinect Motor USB device
# (VID=045e PID=02b0) via vendor control transfers. This is a separate
# USB device from the camera, so there's ZERO conflict.
#
# Kinect motor protocol (reverse-engineered):
#   bmRequestType = 0x40 (vendor, host-to-device)
#   bRequest      = 0x31 (SET_TILT)
#   wValue        = tilt angle in degrees (signed 16-bit, two's complement)
#   wIndex        = 0x00
# ═══════════════════════════════════════════════════════════

KINECT_MOTOR_VID = 0x045e
KINECT_MOTOR_PID = 0x02b0


def set_kinect_tilt(logical_angle_deg):
    """Set Kinect tilt (one-shot) via pyusb. Angle: 0°=horizontal, negative=down.
    Internally applies KINECT_TILT_OFFSET to convert to motor angle."""
    import usb.core
    import usb.util
    motor_angle = int(logical_angle_deg + KINECT_TILT_OFFSET)
    try:
        dev = usb.core.find(idVendor=KINECT_MOTOR_VID, idProduct=KINECT_MOTOR_PID)
        if dev is None:
            return False
        wValue = motor_angle & 0xFFFF  # two's complement for negative
        dev.ctrl_transfer(0x40, 0x31, wValue, 0x00, timeout=1000)
        usb.util.dispose_resources(dev)
        return True
    except Exception:
        return False


class KinectMotor:
    """Motor tilt controller using pyusb direct USB control transfers.

    Talks directly to the Kinect Motor USB device (045e:02b0) which is
    a SEPARATE USB device from the camera (045e:02ae). This completely
    avoids the USB conflict with the freenect sync API.

    No subprocess, no libfreenect for motor — just a single USB control
    transfer per tilt command. Instant, reliable, no -6 errors.
    """

    def __init__(self, device_index=0, logger=None):
        import usb.core
        self._logger = logger
        self._current_tilt = 0.0
        self._connected = False
        self._lock = threading.Lock()
        self._last_cmd_time = 0.0
        self._dev = None

    def connect(self):
        """Open the Kinect Motor USB device."""
        import usb.core
        try:
            self._dev = usb.core.find(
                idVendor=KINECT_MOTOR_VID, idProduct=KINECT_MOTOR_PID)
            if self._dev is None:
                self._log('error', 'Motor USB (045e:02b0) no encontrado')
                return False
            self._connected = True
            self._log('info',
                f'Motor tilt: pyusb directo (Bus {self._dev.bus} '
                f'Dev {self._dev.address})')
            return True
        except Exception as e:
            self._log('error', f'Motor USB error: {e}')
            return False

    def set_tilt(self, logical_deg):
        """Set tilt via USB control transfer. Rate-limited, non-blocking."""
        logical_deg = max(KINECT_TILT_MIN, min(KINECT_TILT_MAX, logical_deg))

        with self._lock:
            # Skip if difference < 2° to avoid motor jitter
            if abs(logical_deg - self._current_tilt) < 2.0:
                return True

            # Rate limit: max 1 command per 0.3s
            now = time.time()
            if (now - self._last_cmd_time) < 0.3:
                return True

            self._current_tilt = logical_deg
            self._last_cmd_time = now

        # Direct USB control transfer (instant, no subprocess)
        motor_angle = int(logical_deg + KINECT_TILT_OFFSET)
        try:
            if self._dev is None:
                return False
            wValue = motor_angle & 0xFFFF
            self._dev.ctrl_transfer(0x40, 0x31, wValue, 0x00, timeout=1000)
            return True
        except Exception as e:
            self._log('warn', f'Motor tilt error: {e}')
            return False

    @property
    def current_tilt(self):
        return self._current_tilt

    @property
    def connected(self):
        return self._connected

    def disconnect(self):
        """Set tilt to horizontal on shutdown."""
        import usb.util
        self._log('info', 'Motor: returning to horizontal on shutdown')
        try:
            if self._dev is not None:
                motor_angle = int(0.0 + KINECT_TILT_OFFSET)
                self._dev.ctrl_transfer(0x40, 0x31, motor_angle & 0xFFFF, 0x00,
                                        timeout=1000)
                time.sleep(1.0)
                usb.util.dispose_resources(self._dev)
        except Exception:
            pass

    def _log(self, level, msg):
        if self._logger:
            getattr(self._logger, level)(msg)


# Stadia controller mapping
JOY_BTN_Y = 3        # Button Y = toggle tilt control
JOY_AXIS_RSTICK_Y = 3  # Right stick Y = tilt up/down (evdev: axis 3)
TILT_SPEED = 15.0     # Degrees per second of stick movement


def get_rgb(index=0):
    """Capture one RGB frame. Returns numpy array (480, 640, 3) uint8 or None."""
    video_ptr = ctypes.c_void_p()
    timestamp = ctypes.c_uint32()
    ret = _sync.freenect_sync_get_video(
        ctypes.byref(video_ptr), ctypes.byref(timestamp),
        index, FREENECT_VIDEO_RGB)
    if ret != 0 or not video_ptr.value:
        return None, 0
    # Map the buffer (freenect owns the memory, valid until next call)
    buf = (ctypes.c_uint8 * (WIDTH * HEIGHT * 3)).from_address(video_ptr.value)
    arr = np.frombuffer(buf, dtype=np.uint8).reshape((HEIGHT, WIDTH, 3)).copy()
    return arr, timestamp.value


def get_depth(index=0):
    """Capture one depth frame. Returns numpy array (480, 640) uint16 or None."""
    depth_ptr = ctypes.c_void_p()
    timestamp = ctypes.c_uint32()
    ret = _sync.freenect_sync_get_depth(
        ctypes.byref(depth_ptr), ctypes.byref(timestamp),
        index, FREENECT_DEPTH_REGISTERED)
    if ret != 0 or not depth_ptr.value:
        return None, 0
    buf = (ctypes.c_uint16 * (WIDTH * HEIGHT)).from_address(depth_ptr.value)
    arr = np.frombuffer(buf, dtype=np.uint16).reshape((HEIGHT, WIDTH)).copy()
    return arr, timestamp.value


# ═══════════════════════════════════════════════════════════
# Kinect Camera Info (approximate factory calibration)
# ═══════════════════════════════════════════════════════════

def make_camera_info(frame_id, stamp):
    """Create CameraInfo with Kinect v1 approximate intrinsics."""
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width = WIDTH
    msg.height = HEIGHT
    msg.distortion_model = 'plumb_bob'
    # Approximate Kinect v1 intrinsics (fx, fy, cx, cy)
    fx = 525.0
    fy = 525.0
    cx = 319.5
    cy = 239.5
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [fx, 0.0, cx,
             0.0, fy, cy,
             0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0,
             0.0, fy, cy, 0.0,
             0.0, 0.0, 1.0, 0.0]
    return msg


# ═══════════════════════════════════════════════════════════
# ROS2 Node
# ═══════════════════════════════════════════════════════════

class KinectNode(Node):
    def __init__(self):
        super().__init__('kinect_node')

        # Parameters
        self.declare_parameter('device_index', 0)
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('rgb_frame_id', 'kinect_rgb_optical')
        self.declare_parameter('depth_frame_id', 'kinect_depth_optical')
        self.declare_parameter('publish_rgb', True)
        self.declare_parameter('publish_depth', True)

        self.device_index = self.get_parameter('device_index').value
        rate = self.get_parameter('publish_rate').value
        self.rgb_frame = self.get_parameter('rgb_frame_id').value
        self.depth_frame = self.get_parameter('depth_frame_id').value
        self.pub_rgb = self.get_parameter('publish_rgb').value
        self.pub_depth = self.get_parameter('publish_depth').value

        # Publishers
        if self.pub_rgb:
            self.rgb_pub = self.create_publisher(Image, 'kinect/rgb/image_raw', 5)
            self.rgb_info_pub = self.create_publisher(CameraInfo, 'kinect/rgb/camera_info', 5)

        if self.pub_depth:
            self.depth_pub = self.create_publisher(Image, 'kinect/depth/image_raw', 5)
            self.depth_info_pub = self.create_publisher(CameraInfo, 'kinect/depth/camera_info', 5)

        # ── Motor tilt (subprocess-based, no USB conflict) ──
        self.declare_parameter('tilt_degrees', 0.0)
        self.declare_parameter('tilt_btn', JOY_BTN_Y)
        self.declare_parameter('tilt_axis', JOY_AXIS_RSTICK_Y)
        self.declare_parameter('tilt_speed', TILT_SPEED)

        initial_tilt = self.get_parameter('tilt_degrees').value
        self._tilt_btn = self.get_parameter('tilt_btn').value
        self._tilt_axis = self.get_parameter('tilt_axis').value
        self._tilt_speed = self.get_parameter('tilt_speed').value

        self._motor = KinectMotor(self.device_index, self.get_logger())
        self._motor.connect()

        # Set initial tilt (subprocess, non-blocking)
        if initial_tilt != 0.0:
            self._motor.set_tilt(initial_tilt)
        self.get_logger().info(
            f'Kinect tilt: {initial_tilt}° (motor={initial_tilt + KINECT_TILT_OFFSET}°)')

        # Tilt control state
        self._tilt_active = False
        self._prev_btn_y = 0
        self._current_tilt = initial_tilt

        # Subscribe to /joy for tilt control
        self.create_subscription(Joy, '/joy', self._joy_callback, 10)
        self.get_logger().info(
            f'Tilt control: btn Y (idx {self._tilt_btn}) toggles, '
            f'right stick Y (axis {self._tilt_axis}) moves')

        # Capture thread
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        # Publish timer
        self._latest_rgb = None
        self._latest_depth = None
        self._lock = threading.Lock()
        self.timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'Kinect node started: device={self.device_index}, '
            f'rate={rate}Hz, rgb={self.pub_rgb}, depth={self.pub_depth}')

    def _capture_loop(self):
        """Background thread: continuously grab frames from freenect."""
        self.get_logger().info('Kinect capture thread started')
        fail_count = 0
        while self._running:
            try:
                rgb, rgb_ts = get_rgb(self.device_index) if self.pub_rgb else (None, 0)
                depth, depth_ts = get_depth(self.device_index) if self.pub_depth else (None, 0)

                with self._lock:
                    if rgb is not None:
                        self._latest_rgb = (rgb, rgb_ts)
                        fail_count = 0
                    if depth is not None:
                        self._latest_depth = (depth, depth_ts)
                        fail_count = 0

                if rgb is None and depth is None:
                    fail_count += 1
                    if fail_count > 30:
                        self.get_logger().warn('Kinect: no frames for 30 iterations, retrying...')
                        fail_count = 0
                    time.sleep(0.1)

            except Exception as e:
                self.get_logger().error(f'Kinect capture error: {e}')
                time.sleep(1.0)

    def _joy_callback(self, msg):
        """Handle joystick input for tilt control.
        Button Y toggles tilt mode. Right stick Y moves tilt while active."""
        # Read button Y (toggle on press)
        btn_y = msg.buttons[self._tilt_btn] if len(msg.buttons) > self._tilt_btn else 0

        # Toggle on rising edge
        if btn_y == 1 and self._prev_btn_y == 0:
            self._tilt_active = not self._tilt_active
            state = 'ACTIVADO' if self._tilt_active else 'DESACTIVADO'
            self.get_logger().info(f'🎮 Tilt control {state} (tilt={self._current_tilt:.0f}°)')
        self._prev_btn_y = btn_y

        # Move tilt while active
        if self._tilt_active and len(msg.axes) > self._tilt_axis:
            stick_y = msg.axes[self._tilt_axis]

            # Deadzone
            if abs(stick_y) < 0.15:
                return

            # stick_y > 0 = stick up = tilt arriba (positive logical = towards 0°)
            # stick_y < 0 = stick down = tilt abajo (negative logical)
            # Rate: degrees per joy message (~30Hz → speed * stick * dt)
            delta = stick_y * self._tilt_speed * 0.033  # ~30fps
            new_tilt = self._current_tilt + delta
            new_tilt = max(KINECT_TILT_MIN, min(KINECT_TILT_MAX, new_tilt))

            if self._motor.connected:
                self._motor.set_tilt(new_tilt)
            self._current_tilt = new_tilt

    def _publish(self):
        """Timer callback: publish latest frames."""
        now = self.get_clock().now().to_msg()

        with self._lock:
            rgb_data = self._latest_rgb
            depth_data = self._latest_depth
            self._latest_rgb = None
            self._latest_depth = None

        if rgb_data is not None and self.pub_rgb:
            arr, _ = rgb_data
            msg = Image()
            msg.header.stamp = now
            msg.header.frame_id = self.rgb_frame
            msg.height = HEIGHT
            msg.width = WIDTH
            msg.encoding = 'rgb8'
            msg.is_bigendian = False
            msg.step = WIDTH * 3
            msg.data = arr.tobytes()
            self.rgb_pub.publish(msg)
            self.rgb_info_pub.publish(make_camera_info(self.rgb_frame, now))

        if depth_data is not None and self.pub_depth:
            arr, _ = depth_data
            msg = Image()
            msg.header.stamp = now
            msg.header.frame_id = self.depth_frame
            msg.height = HEIGHT
            msg.width = WIDTH
            msg.encoding = '16UC1'
            msg.is_bigendian = False
            msg.step = WIDTH * 2
            msg.data = arr.tobytes()
            self.depth_pub.publish(msg)
            self.depth_info_pub.publish(make_camera_info(self.depth_frame, now))

    def destroy_node(self):
        self._running = False
        # Disconnect motor first
        try:
            self._motor.disconnect()
        except Exception:
            pass
        try:
            _sync.freenect_sync_stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KinectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
