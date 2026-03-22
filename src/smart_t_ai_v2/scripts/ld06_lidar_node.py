#!/usr/bin/env python3
"""
ROS 2 node for the LDRobot LD06 LiDAR.

Reads raw serial data from the LD06, decodes the proprietary protocol,
and publishes sensor_msgs/LaserScan on /scan.

Protocol reference (LD06 / LD19):
  - Baud rate : 230 400
  - Packet    : 47 bytes
    [0]       : 0x54  header
    [1]       : 0x2C  ver_len  (ver=1, points=12)
    [2-3]     : speed          (uint16 LE, 0.01 °/s)
    [4-5]     : start_angle    (uint16 LE, 0.01 °)
    [6-41]    : 12 × {distance (uint16 LE, mm), intensity (uint8)}
    [42-43]   : end_angle      (uint16 LE, 0.01 °)
    [44-45]   : timestamp      (uint16 LE, ms)
    [46]      : CRC8

Author : GitHub Copilot – Smart Trolley project
"""

import math
import struct
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import serial


# ──────────────────────── CRC-8 table (poly 0x4D) ────────────────────────
_CRC_TABLE = [
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE,
    0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
    0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07,
    0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
    0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1,
    0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18,
    0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
    0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90,
    0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
    0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39,
    0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F,
    0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
    0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26,
    0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
    0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2,
    0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B,
    0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
    0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD,
    0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
    0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64,
    0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC,
    0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
    0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
    0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3,
    0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A,
    0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8,
]


def _crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


# ──────────────────────── LD06 constants ─────────────────────────────────
HEADER = 0x54
VER_LEN = 0x2C          # version 1, 12 points
PACKET_SIZE = 47
POINTS_PER_PACKET = 12
ANGLE_RANGE = 360       # full rotation (degrees)
DEG2RAD = math.pi / 180.0


class LD06LidarNode(Node):
    """ROS 2 node that reads an LD06 LiDAR and publishes LaserScan."""

    def __init__(self):
        super().__init__('ld06_lidar_node')

        # ── Parameters ──
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('topic_name', 'scan')
        self.declare_parameter('range_min', 0.02)      # 20 mm
        self.declare_parameter('range_max', 12.0)       # 12 m
        self.declare_parameter('scan_direction', True)   # True = CCW (standard)
        self.declare_parameter('angle_min', 0.0)         # degrees
        self.declare_parameter('angle_max', 360.0)       # degrees
        self.declare_parameter('enable_angle_crop', False)
        self.declare_parameter('angle_crop_min', 0.0)
        self.declare_parameter('angle_crop_max', 360.0)

        self._port = self.get_parameter('port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._frame_id = self.get_parameter('frame_id').value
        self._topic = self.get_parameter('topic_name').value
        self._range_min = self.get_parameter('range_min').value
        self._range_max = self.get_parameter('range_max').value
        self._scan_dir = self.get_parameter('scan_direction').value
        self._enable_crop = self.get_parameter('enable_angle_crop').value
        self._crop_min = self.get_parameter('angle_crop_min').value * DEG2RAD
        self._crop_max = self.get_parameter('angle_crop_max').value * DEG2RAD

        # ── Publisher (sensor_data QoS: Best Effort for RViz compatibility) ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self._pub = self.create_publisher(LaserScan, self._topic, sensor_qos)

        # ── Accumulator for one full 360° scan ──
        self._angle_data = {}       # angle_index -> (distance, intensity)
        self._last_start_angle = 0.0
        self._scan_count = 0
        self._packet_count = 0
        self._crc_errors = 0

        # ── Serial port ──
        self._serial = None
        self._running = False
        self._thread = None

        self.get_logger().info(
            f'LD06 LiDAR node starting — port={self._port}, '
            f'baud={self._baudrate}, frame={self._frame_id}'
        )
        self._open_serial()

    # ──────────────────── serial helpers ──────────────────────────────────

    def _open_serial(self):
        """Open serial port and start reader thread."""
        try:
            self._serial = serial.Serial(
                self._port,
                self._baudrate,
                timeout=1.0,
            )
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            self.get_logger().info(f'✅ Serial port {self._port} opened')
        except serial.SerialException as e:
            self.get_logger().error(f'❌ Cannot open {self._port}: {e}')
            raise SystemExit(1)

    def _read_loop(self):
        """Background thread that reads serial data and processes packets."""
        buf = bytearray()
        while self._running and rclpy.ok():
            try:
                chunk = self._serial.read(512)
                if not chunk:
                    continue
                buf.extend(chunk)

                # Process all complete packets in buffer
                while len(buf) >= PACKET_SIZE:
                    # Find header
                    idx = buf.find(bytes([HEADER]))
                    if idx < 0:
                        buf.clear()
                        break
                    if idx > 0:
                        del buf[:idx]       # discard bytes before header
                    if len(buf) < PACKET_SIZE:
                        break

                    # Verify ver_len
                    if buf[1] != VER_LEN:
                        del buf[:1]
                        continue

                    packet = bytes(buf[:PACKET_SIZE])
                    del buf[:PACKET_SIZE]

                    # CRC check
                    if _crc8(packet[:-1]) != packet[-1]:
                        self._crc_errors += 1
                        continue

                    self._process_packet(packet)

            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}')
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().error(f'Read loop error: {e}')
                time.sleep(0.1)

    # ──────────────────── packet processing ──────────────────────────────

    def _process_packet(self, pkt: bytes):
        """Decode one 47-byte LD06 packet and accumulate points."""
        speed = struct.unpack_from('<H', pkt, 2)[0] / 100.0       # °/s
        start_angle = struct.unpack_from('<H', pkt, 4)[0] / 100.0  # °
        end_angle = struct.unpack_from('<H', pkt, 42)[0] / 100.0   # °
        # timestamp = struct.unpack_from('<H', pkt, 44)[0]

        # Detect full rotation (angle wraps past 360°)
        if start_angle < self._last_start_angle and self._angle_data:
            self._publish_scan(speed)
        self._last_start_angle = start_angle

        # Compute per-point angular step
        if end_angle >= start_angle:
            angle_span = end_angle - start_angle
        else:
            angle_span = (360.0 - start_angle) + end_angle

        step = angle_span / (POINTS_PER_PACKET - 1) if POINTS_PER_PACKET > 1 else 0

        for i in range(POINTS_PER_PACKET):
            offset = 6 + i * 3
            dist_mm = struct.unpack_from('<H', pkt, offset)[0]
            intensity = pkt[offset + 2]

            angle_deg = (start_angle + step * i) % 360.0
            angle_rad = angle_deg * DEG2RAD
            dist_m = dist_mm / 1000.0

            # Angle crop
            if self._enable_crop:
                if self._crop_min <= self._crop_max:
                    if self._crop_min <= angle_rad <= self._crop_max:
                        continue
                else:
                    if angle_rad >= self._crop_min or angle_rad <= self._crop_max:
                        continue

            # Store using quantized angle index (0.5° resolution → 720 bins)
            idx = int(round(angle_deg * 2)) % 720
            # Keep closest valid measurement per bin
            if dist_m >= self._range_min:
                if idx not in self._angle_data or dist_m < self._angle_data[idx][0]:
                    self._angle_data[idx] = (dist_m, intensity)

        self._packet_count += 1

    # ──────────────────── publish ─────────────────────────────────────────

    # TF latency compensation: the dynamic TF (odom→base_link) from
    # arduino_bridge arrives ~13-18 ms behind the current clock.  RViz's
    # MessageFilter looks up the TF at the exact scan stamp; if that stamp
    # is newer than the latest TF in the cache, tf2 cannot extrapolate and
    # the message is dropped.  By stamping the scan 50 ms in the past we
    # guarantee the TF is already cached even under CPU load (8+ nodes).
    _TF_LATENCY_OFFSET_NS = 50_000_000   # 50 ms

    def _publish_scan(self, speed_deg_s: float):
        """Build and publish a LaserScan message from accumulated points."""
        if not self._angle_data:
            return

        # Stamp slightly in the past so TF is already in cache for RViz
        now_ns = self.get_clock().now().nanoseconds - self._TF_LATENCY_OFFSET_NS
        stamp_sec = int(now_ns // 1_000_000_000)
        stamp_nsec = int(now_ns % 1_000_000_000)

        num_bins = 720   # 0.5° per bin → 360°
        angle_increment = (2.0 * math.pi) / num_bins   # ~0.00873 rad

        msg = LaserScan()
        msg.header.stamp.sec = stamp_sec
        msg.header.stamp.nanosec = stamp_nsec
        msg.header.frame_id = self._frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi - angle_increment
        msg.angle_increment = angle_increment
        msg.range_min = float(self._range_min)
        msg.range_max = float(self._range_max)

        # scan_time ≈ time for one full rotation
        if speed_deg_s > 0:
            msg.scan_time = 360.0 / speed_deg_s
        else:
            msg.scan_time = 0.1
        msg.time_increment = msg.scan_time / num_bins

        ranges = [float('inf')] * num_bins
        intensities = [0.0] * num_bins

        for idx, (dist, inten) in self._angle_data.items():
            if 0 <= idx < num_bins:
                if self._range_min <= dist <= self._range_max:
                    ranges[idx] = dist
                    intensities[idx] = float(inten)

        if not self._scan_dir:
            ranges.reverse()
            intensities.reverse()

        msg.ranges = ranges
        msg.intensities = intensities

        self._pub.publish(msg)
        self._scan_count += 1

        if self._scan_count % 50 == 1:
            valid = sum(1 for r in ranges if r != float('inf'))
            self.get_logger().info(
                f'Scan #{self._scan_count}: {valid}/{num_bins} points, '
                f'speed={speed_deg_s:.0f} °/s ({speed_deg_s/6:.0f} RPM), '
                f'CRC errors={self._crc_errors}'
            )

        self._angle_data.clear()

    # ──────────────────── shutdown ────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LD06LidarNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
