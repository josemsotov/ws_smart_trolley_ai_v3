#!/usr/bin/env python3
"""
ROS2 Arduino Bridge - Smart Trolley Hardware Interface
Compatible with MOTOR-INTERFACE-V-13 Arduino firmware

Sensor Profiles:
  Profile A (default): Hall (45 PPR) + MPU9250 IMU → encoder odom + gyro heading
  Profile B:           Hall (45 PPR) + Opto (72 PPR) = 117 PPR combined → higher res odom

Subscribes: /cmd_vel (geometry_msgs/Twist)
Publishes: /odom (nav_msgs/Odometry)
          /joint_states (sensor_msgs/JointState)
          /arduino_status (std_msgs/String)
          /imu/raw (sensor_msgs/Imu) [Profile A only]

Architecture: Dedicated serial I/O thread handles all read/write.
ROS callbacks only enqueue commands; never block on serial.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import serial
import time
import math
import threading
import queue
import os
import sys

# Importar Bluetooth Helper (si está disponible)
try:
    from .bluetooth_helper import BluetoothHelper
    BLUETOOTH_AVAILABLE = True
except ImportError:
    BLUETOOTH_AVAILABLE = False


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Declarar parámetros
        # Connection type
        self.declare_parameter('connection_type', 'usb')  # 'usb' or 'bluetooth'
        
        # USB/Serial parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        
        # Bluetooth parameters
        self.declare_parameter('bluetooth_address', '')  # MAC address
        self.declare_parameter('bluetooth_port', '/dev/rfcomm0')  # RFCOMM port
        self.declare_parameter('bluetooth_channel', 1)
        self.declare_parameter('bluetooth_device_name', 'HC-05')
        self.declare_parameter('auto_reconnect', True)
        self.declare_parameter('reconnect_interval', 5.0)
        
        # Robot parameters
        self.declare_parameter('wheel_separation', 0.48)
        self.declare_parameter('wheel_radius', 0.10)
        self.declare_parameter('encoder_ppr', 45)  # 45 for Profile A, 117 for Profile B (Hall 45 + Opto 72)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('cmd_vel_timeout', 1.0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('encoder_read_rate', 10.0)
        self.declare_parameter('sensor_profile', 'A')  # 'A' = Hall+MPU9250, 'B' = Hall+Opto
        
        # Motor bias compensation (angular offset para compensar asimetría entre motores)
        # Positivo → gira a la izquierda (motor derecho más débil)
        # Negativo → gira a la derecha (motor izquierdo más débil)
        self.declare_parameter('motor_bias_angular', 0.0)  # rad/s

        # Debug: log every raw line received from Arduino (useful to verify protocol)
        self.declare_parameter('debug_serial', False)

        # PID Heading Control Parameters
        self.declare_parameter('pid_heading_enabled', True)
        self.declare_parameter('pid_heading_kp', 1.0)
        self.declare_parameter('pid_heading_ki', 0.05)
        self.declare_parameter('pid_heading_kd', 0.1)
        self.declare_parameter('pid_heading_max_correction', 0.3)  # rad/s max angular correction
        self.declare_parameter('pid_heading_deadband', 0.01)  # rad - ignore errors smaller than this
        
        # Obtener parámetros de conexión
        self.connection_type = self.get_parameter('connection_type').value.lower()
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        
        # Bluetooth parameters
        self.bt_address = self.get_parameter('bluetooth_address').value
        self.bt_port = self.get_parameter('bluetooth_port').value
        self.bt_channel = self.get_parameter('bluetooth_channel').value
        self.bt_device_name = self.get_parameter('bluetooth_device_name').value
        self.auto_reconnect = self.get_parameter('auto_reconnect').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        
        # Robot parameters
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.encoder_ppr = self.get_parameter('encoder_ppr').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.cmd_timeout = self.get_parameter('cmd_vel_timeout').value
        self.motor_bias_angular = self.get_parameter('motor_bias_angular').value
        self.debug_serial = self.get_parameter('debug_serial').value

        # Sensor Profile
        self.sensor_profile = self.get_parameter('sensor_profile').value.upper()
        self.use_imu = (self.sensor_profile == 'A')
        
        # IMU data (Profile A: Hall + MPU9250)
        self.imu_yaw_rate = 0.0    # rad/s from gyro Z
        self.imu_accel_x = 0.0     # m/s² from accel X
        self.imu_theta = 0.0       # Integrated heading from gyro
        self.last_imu_time = time.time()
        self.imu_heading_weight = 0.98  # Complementary filter weight for gyro
        
        # PID Heading Control
        self.pid_heading_enabled = self.get_parameter('pid_heading_enabled').value
        self.pid_heading_kp = self.get_parameter('pid_heading_kp').value
        self.pid_heading_ki = self.get_parameter('pid_heading_ki').value
        self.pid_heading_kd = self.get_parameter('pid_heading_kd').value
        self.pid_heading_max_correction = self.get_parameter('pid_heading_max_correction').value
        self.pid_heading_deadband = self.get_parameter('pid_heading_deadband').value
        
        # PID Heading state
        self.desired_theta = 0.0           # Target heading (rad)
        self.heading_integral = 0.0        # PID integral term
        self.heading_prev_error = 0.0      # PID derivative term
        self.heading_locked = False        # True when driving straight
        self.last_pid_time = time.time()
        self.pid_correction = 0.0          # Latest angular correction
        self.theta_updated = False         # Flag: new encoder data arrived
        self.last_theta_for_pid = 0.0      # Last theta used in PID computation
        
        # Variables de odometría
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoder_left = 0
        self.last_encoder_right = 0
        self.last_encoder_time = time.time()
        
        # Wheel positions (radians) for joint_states
        self.wheel_pos_left = 0.0
        self.wheel_pos_right = 0.0
        
        # Variables de velocidad (latest commanded)
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.last_cmd_time = time.time()
        
        # Measured velocities from encoders
        self.measured_vl = 0.0
        self.measured_vr = 0.0
        
        # Serial command queue (non-blocking for ROS callbacks)
        self.cmd_queue = queue.Queue(maxsize=5)
        
        # Estado de conexión
        self.arduino = None
        self.connected = False
        self._running = True
        
        # Arduino status
        self.arduino_status = "DISCONNECTED"
        
        # Conectar Arduino
        self.connect_arduino()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/arduino_status', 10)
        
        # IMU publisher (Profile A only)
        if self.use_imu:
            self.imu_pub = self.create_publisher(Imu, '/imu/raw', 10)
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timers (publishing at fixed rates - never blocks on serial)
        pub_rate = self.get_parameter('publish_rate').value
        self.timer_pub = self.create_timer(1.0 / pub_rate, self.publish_odom)
        self.timer_status = self.create_timer(1.0, self.publish_status)
        self.timer_watchdog = self.create_timer(0.1, self.cmd_vel_watchdog)
        self.timer_pid_log = self.create_timer(2.0, self.log_pid_status)
        
        # PID control timer - decoupled from encoder rate, capped at 10Hz
        # At 50Hz encoder rate, PID at 50Hz would saturate the serial queue
        # and interfere with direct rotation commands
        _enc_rate_raw = self.get_parameter('encoder_read_rate').value
        pid_control_rate = min(_enc_rate_raw, 10.0)
        self.timer_pid = self.create_timer(1.0 / pid_control_rate, self.pid_heading_tick)
        
        # Start dedicated serial I/O thread
        self.serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self.serial_thread.start()
        
        self.get_logger().info('Arduino Bridge initialized')
        self.get_logger().info(f'Connection: {self.connection_type.upper()}')
        self.get_logger().info(f'Sensor Profile: {self.sensor_profile} '
                              f'({"Hall+MPU9250" if self.use_imu else "Hall+Opto 85PPR"})')
        if self.connection_type == 'bluetooth':
            self.get_logger().info(f'Bluetooth: {self.bt_device_name} → {self.bt_port} @ {self.baud_rate}')
        else:
            self.get_logger().info(f'Serial: {self.serial_port} @ {self.baud_rate}')
        self.get_logger().info(f'Wheels: sep={self.wheel_separation}m, r={self.wheel_radius}m, ppr={self.encoder_ppr}')
        self.get_logger().info(f'PID Heading: enabled={self.pid_heading_enabled}, '
                              f'Kp={self.pid_heading_kp}, Ki={self.pid_heading_ki}, '
                              f'Kd={self.pid_heading_kd}, max_corr={self.pid_heading_max_correction}')
    
    def _setup_bluetooth_connection(self):
        """
        Configura conexión Bluetooth con el Arduino.
        
        Returns:
            str: Puerto serial a usar o None si falla
        """
        if not BLUETOOTH_AVAILABLE:
            self.get_logger().error('Bluetooth helper no disponible. Instala: pip install pybluez')
            return None
        
        # Si ya existe el puerto bluetooth, intentar usarlo directamente
        if os.path.exists(self.bt_port):
            self.get_logger().info(f'Puerto Bluetooth existente: {self.bt_port}')
            return self.bt_port
        
        # Crear helper
        bt_helper = BluetoothHelper(logger=self.get_logger())
        
        # Setup completo: buscar, emparejar, conectar, crear puerto
        if self.bt_address:
            # Usar dirección MAC especificada
            self.get_logger().info(f'Conectando a dirección Bluetooth: {self.bt_address}')
            port = bt_helper.setup_bluetooth_connection(
                address=self.bt_address,
                channel=self.bt_channel,
                port=self.bt_port
            )
        else:
            # Buscar por nombre de dispositivo
            self.get_logger().info(f'Buscando dispositivo Bluetooth: {self.bt_device_name}')
            port = bt_helper.setup_bluetooth_connection(
                device_name=self.bt_device_name,
                channel=self.bt_channel,
                port=self.bt_port
            )
        
        if not port:
            self.get_logger().error('No se pudo establecer conexión Bluetooth')
            self.get_logger().info('Tip: Ejecuta manualmente:')
            self.get_logger().info(f'  python3 scripts/bluetooth_helper.py scan')
            self.get_logger().info(f'  python3 scripts/bluetooth_helper.py connect {self.bt_device_name}')
        
        return port
    
    def connect_arduino(self):
        """Conectar al Arduino (USB o Bluetooth)"""
        try:
            if self.arduino and self.arduino.is_open:
                self.arduino.close()
            
            # Determinar puerto según tipo de conexión
            if self.connection_type == 'bluetooth':
                port = self._setup_bluetooth_connection()
                if not port:
                    raise serial.SerialException("No se pudo establecer conexión Bluetooth")
            else:
                port = self.serial_port
            
            # Conectar serial
            # Timeout: 50ms en USB (latencia <5ms), 200ms en BT (latencia 50-150ms)
            serial_timeout = 0.20 if self.connection_type == 'bluetooth' else 0.05
            self.arduino = serial.Serial(
                port,
                self.baud_rate,
                timeout=serial_timeout
            )
            # BT no hace reset automático al abrir el puerto (no DTR)
            if self.connection_type != 'bluetooth':
                time.sleep(2)  # Esperar reset Arduino por USB
            else:
                time.sleep(0.5)  # BT: solo limpiar buffer
            
            # Limpiar buffer
            self.arduino.reset_input_buffer()
            self.arduino.reset_output_buffer()
            
            self.connected = True
            self.arduino_status = "CONNECTED"
            self.get_logger().info(f'Arduino connected successfully ({self.connection_type.upper()}): {port}')
            
        except serial.SerialException as e:
            self.connected = False
            self.arduino_status = f"ERROR: {str(e)}"
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
        except Exception as e:
            self.connected = False
            self.arduino_status = f"ERROR: {str(e)}"
            self.get_logger().error(f'Unexpected error: {e}')
    
    def _serial_loop(self):
        """
        Dedicated serial I/O thread.
        Handles all serial read/write in a single thread, 
        never competing with ROS callbacks for locks.
        
        Loop rate: ~50Hz (20ms per cycle)
        Each cycle: flush command queue → request encoders → read response
        """
        enc_rate = self.get_parameter('encoder_read_rate').value
        # BT: latencia 50-150ms → máximo 5Hz para evitar backpressure en rfcomm
        if self.connection_type == 'bluetooth':
            enc_rate = min(enc_rate, 5.0)
            self.get_logger().warn(
                f'BT mode: encoder_read_rate limitado a {enc_rate}Hz '
                f'(latencia BT ~100ms, USB recomendado para control en tiempo real)')
        enc_interval = 1.0 / enc_rate
        last_enc_request = 0.0
        
        while self._running:
            if not self.connected:
                time.sleep(1.0)
                continue
            
            try:
                # 1. Send any pending velocity commands from the queue
                while not self.cmd_queue.empty():
                    try:
                        cmd = self.cmd_queue.get_nowait()
                        self.arduino.write((cmd + '\n').encode())
                        self.arduino.flush()
                    except queue.Empty:
                        break
                    except Exception as e:
                        self.get_logger().error(f'Serial write error: {e}')
                        self.connected = False
                        break
                
                # 2. Request encoder data at configured rate
                now = time.time()
                if now - last_enc_request >= enc_interval:
                    last_enc_request = now
                    try:
                        self.arduino.write(b'e\n')
                        self.arduino.flush()
                        # Profile A: also request IMU data
                        if self.use_imu:
                            self.arduino.write(b'i\n')
                            self.arduino.flush()
                    except Exception as e:
                        self.get_logger().error(f'Encoder request error: {e}')
                        self.connected = False
                        continue
                
                # 3. Read all available serial responses
                while self.arduino.in_waiting > 0:
                    try:
                        line = self.arduino.readline().decode('utf-8', errors='replace').strip()
                        if self.debug_serial and line:
                            self.get_logger().info(f'[SERIAL RX] {repr(line)}')
                        if line.startswith('e '):
                            parts = line.split()
                            if len(parts) == 3:
                                left_count = int(parts[1])
                                right_count = int(parts[2])
                                self.update_odometry(left_count, right_count)
                            elif self.debug_serial:
                                self.get_logger().warn(
                                    f'[SERIAL] "e" line mal formateada: {repr(line)} '
                                    f'(esperado: "e LEFT RIGHT")')
                        elif line.startswith('i ') and self.use_imu:
                            parts = line.split()
                            if len(parts) == 3:
                                self.imu_yaw_rate = float(parts[1])
                                self.imu_accel_x = float(parts[2])
                                self._update_imu_heading()
                    except Exception as e:
                        self.get_logger().warn(f'Serial read error: {e}')
                        break
                
                # Sleep to maintain ~50Hz loop rate
                time.sleep(0.02)
                
            except Exception as e:
                self.get_logger().error(f'Serial loop error: {e}')
                self.connected = False
                # Auto-fallback: si USB falla y BT está disponible, intentar BT
                if self.connection_type == 'usb' and self.auto_reconnect:
                    if os.path.exists(self.bt_port):
                        self.get_logger().warn(
                            f'USB perdido → intentando fallback BT ({self.bt_port} @ 38400)')
                        self.connection_type = 'bluetooth'
                        self.baud_rate = 38400
                        time.sleep(2.0)
                        self.connect_arduino()
                    else:
                        self.get_logger().warn(
                            f'USB perdido. Para activar BT: sudo rfcomm bind rfcomm0 {self.bt_address} 1')
                        time.sleep(self.reconnect_interval)
                        self.connect_arduino()
                else:
                    time.sleep(self.reconnect_interval)
                    self.connect_arduino()
    
    def _send_velocity(self, cmd):
        """Enqueue a velocity command (non-blocking, drops old if full)."""
        try:
            # Clear old commands - only latest velocity matters
            while not self.cmd_queue.empty():
                try:
                    self.cmd_queue.get_nowait()
                except queue.Empty:
                    break
            self.cmd_queue.put_nowait(cmd)
        except queue.Full:
            pass  # Skip if queue is full
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def _compute_heading_pid(self):
        """Compute PID heading correction for straight driving.

        Called from pid_heading_tick ONLY when heading_locked=True.
        heading_locked is managed exclusively by cmd_vel_callback / watchdog.
        This function NEVER modifies heading_locked.
        """
        if not self.pid_heading_enabled:
            return 0.0

        now = time.time()
        dt = now - self.last_pid_time
        self.last_pid_time = now

        if dt <= 0 or dt > 1.0:
            return self.pid_correction

        # Only compute PID when we have new encoder data
        if not self.theta_updated:
            return self.pid_correction  # Return last correction
        self.theta_updated = False
        
        # Compute heading error
        error = self._normalize_angle(self.desired_theta - self.theta)
        
        # Deadband - ignore very small errors
        if abs(error) < self.pid_heading_deadband:
            self.pid_correction = 0.0
            return 0.0
        
        # PID computation
        self.heading_integral += error * dt
        # Anti-windup: clamp integral
        max_integral = self.pid_heading_max_correction / max(self.pid_heading_ki, 0.01)
        self.heading_integral = max(-max_integral, min(max_integral, self.heading_integral))
        
        derivative = (error - self.heading_prev_error) / dt
        self.heading_prev_error = error
        
        correction = (self.pid_heading_kp * error + 
                      self.pid_heading_ki * self.heading_integral + 
                      self.pid_heading_kd * derivative)
        
        # Negar: Arduino motor derecho tiene DIR invertido físicamente
        # (convención Arduino: angular+ en firmware = giro derecha físico)
        # Python envía -angular para que +angular.z ROS = giro izquierda físico
        correction = -correction
        
        # Clamp correction
        correction = max(-self.pid_heading_max_correction, 
                         min(self.pid_heading_max_correction, correction))
        
        self.pid_correction = correction
        return correction
    
    def pid_heading_tick(self):
        """PID control timer - runs at encoder_read_rate.
        Computes heading correction and sends corrected velocity to Arduino.
        Only sends if robot is moving and heading is locked."""
        if not self.pid_heading_enabled:
            return
        
        # Only act when robot is moving straight (heading locked)
        if not self.heading_locked:
            return
        
        # Check cmd_vel timeout
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            return
        
        corrected_angular = self._compute_heading_pid()
        # Add motor_bias_angular to PID output to compensate mechanical imbalance
        biased_angular = corrected_angular + self.motor_bias_angular
        cmd = f"v {self.linear_vel:.3f} {biased_angular:.3f}"
        self._send_velocity(cmd)
    
    def cmd_vel_callback(self, msg):
        """Callback para comandos de velocidad (non-blocking).
        Stores desired velocities. PID correction is applied in pid_heading_tick."""
        # Limitar velocidades
        linear = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        
        # Actualizar tiempo de último comando
        self.last_cmd_time = time.time()
        
        # Store desired velocities
        self.linear_vel = linear
        self.angular_vel = angular
        
        # Determine PID mode based on commanded velocities
        abs_linear = abs(linear)
        abs_angular = abs(angular)
        
        if abs_linear < 0.01 and abs_angular < 0.01:
            # Stopped: reset PID, send stop
            self.heading_locked = False
            self.heading_integral = 0.0
            self.heading_prev_error = 0.0
            self.pid_correction = 0.0
            self._send_velocity("v 0.000 0.000")
        elif abs_angular > 0.01:
            # Turning: pass through directly, no PID
            # Negar angular: Arduino motor DER tiene DIR invertido físicamente.
            # Convención: python envía -angular para que +angular.z ROS = giro izquierda.
            # Validado en commit 2a0ea06: test 5m→-180°→5m→+180° con encoders OK.
            self.heading_locked = False
            self.heading_integral = 0.0
            self.heading_prev_error = 0.0
            self.desired_theta = self.theta
            self.pid_correction = 0.0
            cmd = f"v {linear:.3f} {-angular:.3f}"
            self._send_velocity(cmd)
        else:
            # Driving straight: lock heading and let PID timer handle corrections
            # Apply motor_bias_angular to compensate for mechanical motor imbalance
            if not self.heading_locked:
                self.desired_theta = self.theta
                self.heading_locked = True
                self.heading_integral = 0.0
                self.heading_prev_error = 0.0
                self.last_pid_time = time.time()
                self.get_logger().info(
                    f'PID Heading LOCKED at {math.degrees(self.desired_theta):.1f}°')
                # Send initial velocity with bias (negar angular: convención Arduino)
                biased_angular = -angular + self.motor_bias_angular
                cmd = f"v {linear:.3f} {biased_angular:.3f}"
                self._send_velocity(cmd)
    
    def cmd_vel_watchdog(self):
        """Watchdog para detener el robot si no hay comandos."""
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            if self.linear_vel != 0.0 or self.angular_vel != 0.0:
                self._send_velocity("v 0.0 0.0")
                self.linear_vel = 0.0
                self.angular_vel = 0.0
                self.heading_locked = False
                self.heading_integral = 0.0
                self.heading_prev_error = 0.0
                self.pid_correction = 0.0
                self.get_logger().warn('cmd_vel timeout - stopping robot')
    
    def update_odometry(self, left_count, right_count):
        """Actualizar odometría basada en encoders (called from serial thread)."""
        current_time = time.time()
        dt = current_time - self.last_encoder_time
        
        if dt < 0.001:
            return
        
        # Calcular deltas
        delta_left = left_count - self.last_encoder_left
        delta_right = right_count - self.last_encoder_right
        
        # Convertir pulsos a metros
        dist_left = (delta_left / self.encoder_ppr) * (2.0 * math.pi * self.wheel_radius)
        dist_right = (delta_right / self.encoder_ppr) * (2.0 * math.pi * self.wheel_radius)
        
        # Actualizar posiciones angulares de las ruedas (radianes)
        self.wheel_pos_left += (delta_left / self.encoder_ppr) * (2.0 * math.pi)
        self.wheel_pos_right += (delta_right / self.encoder_ppr) * (2.0 * math.pi)
        
        # Measured wheel velocities
        self.measured_vl = dist_left / dt
        self.measured_vr = dist_right / dt
        
        # Cinemática diferencial
        delta_dist = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_separation
        
        # Actualizar pose
        if abs(delta_theta) < 0.0001:
            self.x += delta_dist * math.cos(self.theta)
            self.y += delta_dist * math.sin(self.theta)
        else:
            radius = delta_dist / delta_theta
            self.x += radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y += -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))
        
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Profile A: fuse encoder theta with IMU gyro via complementary filter
        if self.use_imu and abs(self.imu_yaw_rate) > 0.001:
            # Complementary filter: trust gyro for fast changes, encoders for drift
            gyro_theta = self.imu_theta  # Already integrated in _update_imu_heading
            encoder_theta = self.theta
            self.theta = (self.imu_heading_weight * gyro_theta + 
                         (1.0 - self.imu_heading_weight) * encoder_theta)
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Signal PID that new encoder data is available
        self.theta_updated = True
        
        # Actualizar valores anteriores
        self.last_encoder_left = left_count
        self.last_encoder_right = right_count
        self.last_encoder_time = current_time
    
    def _update_imu_heading(self):
        """Integrate gyro Z to update IMU heading estimate (Profile A only).
        Called from serial thread when 'i' response arrives."""
        now = time.time()
        dt = now - self.last_imu_time
        self.last_imu_time = now
        
        if dt <= 0 or dt > 1.0:
            return
        
        # Integrate yaw rate to get heading
        self.imu_theta += self.imu_yaw_rate * dt
        self.imu_theta = math.atan2(math.sin(self.imu_theta), math.cos(self.imu_theta))
    
    def _publish_imu(self):
        """Publish raw IMU data (Profile A only)."""
        if not self.use_imu:
            return
        
        current_time = self.get_clock().now()
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Angular velocity (gyro Z)
        imu_msg.angular_velocity.z = self.imu_yaw_rate
        
        # Linear acceleration (accel X)
        imu_msg.linear_acceleration.x = self.imu_accel_x
        
        # Orientation from integrated gyro
        qz = math.sin(self.imu_theta / 2.0)
        qw = math.cos(self.imu_theta / 2.0)
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        
        # Covariance (approximate)
        imu_msg.orientation_covariance[0] = 0.01
        imu_msg.orientation_covariance[4] = 0.01
        imu_msg.orientation_covariance[8] = 0.005
        imu_msg.angular_velocity_covariance[8] = 0.001
        imu_msg.linear_acceleration_covariance[0] = 0.01
        
        self.imu_pub.publish(imu_msg)
    
    def log_pid_status(self):
        """Log PID heading status periodically for diagnostics."""
        if self.pid_heading_enabled and self.heading_locked:
            error = self._normalize_angle(self.desired_theta - self.theta)
            self.get_logger().info(
                f'PID Heading: target={math.degrees(self.desired_theta):.1f}° '
                f'current={math.degrees(self.theta):.1f}° '
                f'error={math.degrees(error):.2f}° '
                f'correction={self.pid_correction:.4f} rad/s')
    
    def publish_odom(self):
        """Publicar odometría y TF (never blocks on serial)."""
        current_time = self.get_clock().now()
        
        # Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # Use measured velocities from encoders
        v_linear = (self.measured_vl + self.measured_vr) / 2.0
        v_angular = (self.measured_vr - self.measured_vl) / self.wheel_separation
        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = v_angular
        
        # Covariance (helps nav2 and other consumers)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.03  # theta
        
        self.odom_pub.publish(odom)
        
        # TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)
        
        # Joint states
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['joint_lh', 'joint_rh']
        joint_state.position = [self.wheel_pos_left, self.wheel_pos_right]
        joint_state.velocity = [self.measured_vl / self.wheel_radius,
                                self.measured_vr / self.wheel_radius]
        joint_state.effort = []
        
        self.joint_states_pub.publish(joint_state)
        
        # Publish IMU data (Profile A only)
        if self.use_imu:
            self._publish_imu()
    
    def publish_status(self):
        """Publicar estado de Arduino."""
        msg = String()
        msg.data = self.arduino_status
        self.status_pub.publish(msg)
        
        if not self.connected:
            self.connect_arduino()
    
    def destroy_node(self):
        """Cerrar conexión al destruir nodo."""
        self._running = False
        if self.arduino and self.arduino.is_open:
            try:
                self.arduino.write(b'v 0.0 0.0\n')
                self.arduino.flush()
                time.sleep(0.1)
                self.arduino.close()
            except Exception:
                pass
            self.get_logger().info('Arduino connection closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    
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


if __name__ == '__main__':
    main()
