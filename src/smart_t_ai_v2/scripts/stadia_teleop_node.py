#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════
  Custom Stadia Teleop Node — Smart Trolley V5
═══════════════════════════════════════════════════════════

Nodo de teleoperación avanzado para el control Google Stadia
conectado por Bluetooth.

Controles:
  L1 (mantener)      = Deadman switch (activa movimiento sticks)
  Stick Izq ↕        = Avance / retroceso
  Stick Izq ↔        = Giro (combinado con avance)
  Stick Der ↔        = Rotación pura sobre Z (sin avance)
  D-Pad ↑↓           = Avance / retroceso (sin deadman, velocidad fija)
  D-Pad ←→           = Rotación izq / der (sin deadman, velocidad fija)
  R1 (pulsar)        = Aumentar nivel de velocidad
  R2 (pulsar)        = Disminuir nivel de velocidad
  Soltar L1           = PARADA inmediata (no afecta D-pad)

Mapeo verificado (Stadia BT/USB, via /joy topic):
  L1 = btn 4 | R1 = btn 5 | R2 = btn 13
  Stick Izq Y = eje 1 (invertido)  ← NO eje 4 (eje 4 = trigger L2)
  Stick Izq X = eje 0 (invertido)
  Stick Der X = eje 2 (invertido)
  D-Pad X     = eje 6 (HAT0X: -1=izq, +1=der)
  D-Pad Y     = eje 7 (HAT0Y: -1=arriba, +1=abajo)

Suscribe: /joy  (sensor_msgs/Joy)
Publica:  /cmd_vel (geometry_msgs/Twist)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class StadiaTeleopNode(Node):

    def __init__(self):
        super().__init__('stadia_teleop')

        # ── Parámetros (override via YAML) ──
        self.declare_parameter('btn_deadman', 4)        # L1
        self.declare_parameter('btn_speed_up', 5)       # R1
        self.declare_parameter('btn_speed_down', 13)    # R2
        self.declare_parameter('axis_linear_y', 1)      # Stick Izq Y (joy topic)
        self.declare_parameter('axis_angular_x', 0)     # Stick Izq X (joy topic)
        self.declare_parameter('axis_rotate_x', 2)      # Stick Der X (joy topic)
        self.declare_parameter('axis_dpad_y', 7)            # D-Pad Y (HAT0Y)
        self.declare_parameter('axis_dpad_x', 6)            # D-Pad X (HAT0X)
        self.declare_parameter('invert_linear', False)   # arriba=+1 → adelante ✓
        self.declare_parameter('invert_angular', True)    # Negar eje angular
        self.declare_parameter('invert_rotate', True)     # Negar eje rotación
        self.declare_parameter('deadzone', 0.10)         # Zona muerta adicional
        self.declare_parameter('speed_levels_linear',
                               [0.1, 0.2, 0.3, 0.4, 0.5])
        self.declare_parameter('speed_levels_angular',
                               [0.3, 0.5, 0.7, 0.9, 1.2])
        self.declare_parameter('default_speed_level', 2)  # Índice (0-based)
        self.declare_parameter('safety_timeout', 0.5)     # Segundos sin /joy → stop

        # ── Leer parámetros ──
        self.btn_deadman   = self.get_parameter('btn_deadman').value
        self.btn_speed_up  = self.get_parameter('btn_speed_up').value
        self.btn_speed_down = self.get_parameter('btn_speed_down').value
        self.axis_linear   = self.get_parameter('axis_linear_y').value
        self.axis_angular  = self.get_parameter('axis_angular_x').value
        self.axis_rotate   = self.get_parameter('axis_rotate_x').value
        self.axis_dpad_y   = self.get_parameter('axis_dpad_y').value
        self.axis_dpad_x   = self.get_parameter('axis_dpad_x').value
        self.inv_lin       = -1.0 if self.get_parameter('invert_linear').value else 1.0
        self.inv_ang       = -1.0 if self.get_parameter('invert_angular').value else 1.0
        self.inv_rot       = -1.0 if self.get_parameter('invert_rotate').value else 1.0
        self.deadzone      = self.get_parameter('deadzone').value
        self.levels_lin    = list(self.get_parameter('speed_levels_linear').value)
        self.levels_ang    = list(self.get_parameter('speed_levels_angular').value)
        self.speed_idx     = self.get_parameter('default_speed_level').value
        self.safety_timeout = self.get_parameter('safety_timeout').value

        # Validar
        n_levels = len(self.levels_lin)
        if len(self.levels_ang) != n_levels:
            self.get_logger().warn(
                'speed_levels_linear y angular tienen distinto largo, '
                'se usará el mínimo')
            n_levels = min(n_levels, len(self.levels_ang))
            self.levels_lin = self.levels_lin[:n_levels]
            self.levels_ang = self.levels_ang[:n_levels]
        self.speed_idx = max(0, min(self.speed_idx, n_levels - 1))

        # ── Estado ──
        self._prev_btn_up = 0
        self._prev_btn_down = 0
        self._deadman_active = False
        self._dpad_active = False
        self._last_joy_stamp = self.get_clock().now()
        self._dbg_time = 0.0

        # ── Pub / Sub ──
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_joy = self.create_subscription(Joy, 'joy', self._joy_cb, 10)

        # Timer de seguridad (10 Hz)
        self.create_timer(0.1, self._safety_cb)

        # ── Mensaje de inicio ──
        self._log_speed('🎮 Stadia Teleop iniciado')
        self.get_logger().info(
            '   L1=activar │ Izq=mover │ Der=rotar │ D-Pad=mover(sin L1) │ R1/R2=vel')

    # ─────────────────────────────────────────────
    #  Callback de /joy
    # ─────────────────────────────────────────────
    def _joy_cb(self, msg: Joy):
        self._last_joy_stamp = self.get_clock().now()
        now_s = self._last_joy_stamp.nanoseconds / 1e9

        # --- Helpers ---
        def btn(idx):
            return msg.buttons[idx] if idx < len(msg.buttons) else 0

        def axis(idx):
            return msg.axes[idx] if idx < len(msg.axes) else 0.0

        deadman = btn(self.btn_deadman)
        btn_up  = btn(self.btn_speed_up)
        btn_dn  = btn(self.btn_speed_down)

        # --- Cambio de velocidad (flanco de subida) ---
        if btn_up == 1 and self._prev_btn_up == 0:
            if self.speed_idx < len(self.levels_lin) - 1:
                self.speed_idx += 1
                self._log_speed('▲ Velocidad')
            else:
                self.get_logger().info('⚠  Ya estás en velocidad MÁXIMA')

        if btn_dn == 1 and self._prev_btn_down == 0:
            if self.speed_idx > 0:
                self.speed_idx -= 1
                self._log_speed('▼ Velocidad')
            else:
                self.get_logger().info('⚠  Ya estás en velocidad MÍNIMA')

        self._prev_btn_up = btn_up
        self._prev_btn_down = btn_dn

        # --- Deadman transitions ---
        was_active = self._deadman_active
        twist = Twist()

        if deadman == 1:
            if not was_active:
                # L1 recién presionado → activar pero NO mover (gracia 1 frame)
                self._deadman_active = True
                axes_dbg = ', '.join(f'{i}:{v:+.3f}' for i, v in enumerate(msg.axes))
                self.get_logger().info(f'🔓 L1 ON — axes=[{axes_dbg}]')
                # Publicar cero en el primer frame (previene picos)
            else:
                # L1 mantenido → procesar sticks
                raw_lin = axis(self.axis_linear) * self.inv_lin
                raw_ang = axis(self.axis_angular) * self.inv_ang
                raw_rot = axis(self.axis_rotate) * self.inv_rot

                # Deadzone
                if abs(raw_lin) < self.deadzone:
                    raw_lin = 0.0
                if abs(raw_ang) < self.deadzone:
                    raw_ang = 0.0
                if abs(raw_rot) < self.deadzone:
                    raw_rot = 0.0

                # Escalas actuales
                max_lin = self.levels_lin[self.speed_idx]
                max_ang = self.levels_ang[self.speed_idx]

                # Stick izquierdo: avance + giro
                twist.linear.x = raw_lin * max_lin
                twist.angular.z = raw_ang * max_ang

                # Stick derecho: rotación pura (se suma al angular)
                twist.angular.z += raw_rot * max_ang

                # Clamp angular
                twist.angular.z = max(-max_ang, min(max_ang, twist.angular.z))

                # Debug periódico (cada 2s si hay movimiento)
                if now_s - self._dbg_time > 2.0:
                    if abs(twist.linear.x) > 0.01 or abs(twist.angular.z) > 0.01:
                        self._dbg_time = now_s
                        self.get_logger().info(
                            f'📊 cmd: lin={twist.linear.x:+.3f} ang={twist.angular.z:+.3f}'
                            f'  (raw: {axis(self.axis_linear):+.3f}, '
                            f'{axis(self.axis_angular):+.3f}, '
                            f'{axis(self.axis_rotate):+.3f})')
        else:
            # L1 suelto → parada (sticks)
            if was_active:
                self._deadman_active = False
                self.get_logger().info('🔒 L1 OFF — STOP')

        # --- D-Pad: movimiento sin deadman ---
        dpad_y = axis(self.axis_dpad_y)  # -1=up, +1=down
        dpad_x = axis(self.axis_dpad_x)  # -1=left, +1=right

        if abs(dpad_y) > 0.5 or abs(dpad_x) > 0.5:
            # D-Pad activo → override con velocidad del nivel actual
            max_lin = self.levels_lin[self.speed_idx]
            max_ang = self.levels_ang[self.speed_idx]

            # D-Pad Y: arriba(-1)=adelante, abajo(+1)=atras
            twist.linear.x = -dpad_y * max_lin
            # D-Pad X: izq(-1)=girar izq(+), der(+1)=girar der(-)
            twist.angular.z = -dpad_x * max_ang

            if not self._dpad_active:
                self._dpad_active = True
                self.get_logger().info('🎯 D-Pad activo')
        else:
            if self._dpad_active:
                self._dpad_active = False
                # Si no hay deadman activo, parar
                if not self._deadman_active:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

        self.pub_cmd.publish(twist)

    # ─────────────────────────────────────────────
    #  Timer de seguridad
    # ─────────────────────────────────────────────
    def _safety_cb(self):
        dt = (self.get_clock().now() - self._last_joy_stamp).nanoseconds / 1e9
        if dt > self.safety_timeout:
            self.pub_cmd.publish(Twist())

    # ─────────────────────────────────────────────
    #  Log de velocidad
    # ─────────────────────────────────────────────
    def _log_speed(self, prefix: str):
        level = self.speed_idx + 1
        total = len(self.levels_lin)
        lin = self.levels_lin[self.speed_idx]
        ang = self.levels_ang[self.speed_idx]
        bar = '█' * level + '░' * (total - level)
        self.get_logger().info(
            f'{prefix}: [{bar}] {level}/{total}  '
            f'({lin:.1f} m/s, {ang:.1f} rad/s)')


def main(args=None):
    rclpy.init(args=args)
    node = StadiaTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_cmd.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
