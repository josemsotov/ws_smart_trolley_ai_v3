#!/usr/bin/env python3
"""
ROS2 Node: Robot Mode Manager — Árbitro de modos y cmd_vel
===========================================================

Gestiona los modos de operación del robot y arbitra las velocidades
provenientes de múltiples fuentes (joystick, gestos, voz, seguidor).

Modos de operación:
  ┌─────────┬──────────────────────────────────────────────────────┐
  │  IDLE   │ Robot detenido, esperando comando                    │
  │  TELEOP │ Control manual con joystick/Stadia                   │
  │  GESTURE│ Control por gestos de mano (Hailo + MediaPipe)       │
  │  VOICE  │ Control por comandos de voz (Vosk ES)                │
  │  FOLLOW │ Seguir persona detectada (person_follower)           │
  └─────────┴──────────────────────────────────────────────────────┘

Transiciones de modo:
  Cualquiera → IDLE:    voz "para/stop" | puño 2s | timeout seguridad
  IDLE → TELEOP:        joystick activado (L1 presionado)
  Cualquiera → GESTURE: voz "modo gesto" | gesto thumb_up 2s
  Cualquiera → VOICE:   voz "modo voz"
  Cualquiera → FOLLOW:  voz "sígueme" | palma 5 dedos 2s
  Cualquiera → TELEOP:  voz "modo manual"

Arbitración de cmd_vel:
  TELEOP  → pasa /joy_cmd_vel (del stadia_teleop o teleop_twist_keyboard)
  GESTURE → pasa /hailo/cmd_vel
  VOICE   → pasa /voice/cmd_vel
  FOLLOW  → pasa /person_follower/cmd_vel
  IDLE    → publica velocidad cero

Topics publicados:
  /robot/mode         (std_msgs/String)    - modo actual del robot
  /robot/status       (std_msgs/String)    - estado JSON completo
  /cmd_vel            (geometry_msgs/Twist) - velocidad final al robot
  /robot/tts_say      (std_msgs/String)    - texto para TTS (espeak)

Topics suscritos:
  /voice/command      (std_msgs/String)    - comandos de voz
  /hailo/gesture_cmd  (std_msgs/String)    - comandos de gesto
  /hailo/faces        (std_msgs/String)    - caras detectadas
  /hailo/cmd_vel      (geometry_msgs/Twist) - velocidad de gestos
  /voice/cmd_vel      (geometry_msgs/Twist) - velocidad de voz
  /person_follower/cmd_vel (geometry_msgs/Twist) - velocidad de seguimiento
  /joy_cmd_vel        (geometry_msgs/Twist) - velocidad de joystick
  /joy                (sensor_msgs/Joy)    - estado del joystick

Author: Smart Trolley V5 Project
"""

import os
import json
import time
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# ─── Modos ───────────────────────────────────────────────────────────────────
MODE_IDLE    = 'IDLE'
MODE_TELEOP  = 'TELEOP'
MODE_GESTURE = 'GESTURE'
MODE_VOICE   = 'VOICE'
MODE_FOLLOW  = 'FOLLOW'

ALL_MODES = [MODE_IDLE, MODE_TELEOP, MODE_GESTURE, MODE_VOICE, MODE_FOLLOW]

# Mensajes TTS para cada transición de modo
MODE_MESSAGES = {
    MODE_IDLE:    'Modo reposo. Robot detenido.',
    MODE_TELEOP:  'Modo manual activado. Usando joystick.',
    MODE_GESTURE: 'Modo gestos activado. Muestra tu rostro para comenzar.',
    MODE_VOICE:   'Modo voz activado. Di un comando.',
    MODE_FOLLOW:  'Modo seguimiento activado. Me mantendré cerca de ti.',
}

# Respuestas TTS para comandos de interacción
CMD_RESPONSES = {
    'greet':    '¡Hola! Soy Smart Trolley, listo para ayudarte.',
    'identify': 'Soy Smart Trolley versión cinco, un robot de acompañamiento inteligente.',
    'status':   'Sistema operativo. Todos los sensores funcionando correctamente.',
    'help':     ('Puedo responder a gestos de mano y comandos de voz. '
                 'Di modo gesto, modo voz, sígueme, o para.'),
}

# ─── Nodo ────────────────────────────────────────────────────────────────────

class RobotModeManager(Node):
    """Árbitro de modos y velocidades del Smart Trolley."""

    def __init__(self):
        super().__init__('robot_mode_manager')

        # ─── Parámetros ───
        self.declare_parameter('default_mode',       MODE_IDLE)
        self.declare_parameter('joy_deadband',        0.15)
        self.declare_parameter('joy_timeout',         3.0)
        self.declare_parameter('cmd_timeout_gesture', 1.5)
        self.declare_parameter('cmd_timeout_voice',   3.5)
        self.declare_parameter('cmd_timeout_follow',  5.0)
        self.declare_parameter('safety_stop_timeout', 2.0)
        self.declare_parameter('tts_enabled',         True)
        self.declare_parameter('tts_voice',           'es')
        self.declare_parameter('publish_rate',        20.0)
        self.declare_parameter('joy_deadman_btn',     4)    # L1 Stadia
        self.declare_parameter('joy_mode_btn',        3)    # Y Stadia → ciclo modo
        self.declare_parameter('allowed_modes',       [MODE_IDLE, MODE_TELEOP,
                                                       MODE_GESTURE, MODE_VOICE,
                                                       MODE_FOLLOW])

        p = self.get_parameter
        self._default_mode      = p('default_mode').value
        self._joy_deadband      = p('joy_deadband').value
        self._joy_timeout       = p('joy_timeout').value
        self._cmd_to_gesture    = p('cmd_timeout_gesture').value
        self._cmd_to_voice      = p('cmd_timeout_voice').value
        self._cmd_to_follow     = p('cmd_timeout_follow').value
        self._safety_timeout    = p('safety_stop_timeout').value
        self._tts_enabled       = p('tts_enabled').value
        self._tts_voice         = p('tts_voice').value
        self._pub_rate          = p('publish_rate').value
        self._joy_deadman_btn   = p('joy_deadman_btn').value
        self._joy_mode_btn      = p('joy_mode_btn').value
        self._allowed_modes     = p('allowed_modes').value

        # ─── Estado ───
        self._mode     = self._default_mode
        self._prev_mode = self._default_mode

        # Últimas velocidades por fuente
        self._vel: dict[str, Twist] = {
            'gesture': Twist(),
            'voice':   Twist(),
            'follow':  Twist(),
            'joy':     Twist(),
        }
        # Timestamps de última recepción de velocidad
        self._vel_t: dict[str, float] = {k: 0.0 for k in self._vel}

        # Timestamp de última cara autorizada
        self._last_face_t   = 0.0
        self._authorized_user = ''

        # Joystick
        self._joy_active    = False
        self._joy_last_t    = 0.0
        self._prev_mode_btn = 0
        self._prev_deadman  = 0

        # TTS
        self._tts_lock   = threading.Lock()
        self._tts_thread = None

        # ─── Publishers ───
        self._mode_pub   = self.create_publisher(String, '/robot/mode',   10)
        self._status_pub = self.create_publisher(String, '/robot/status', 10)
        self._cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',      10)
        self._tts_pub    = self.create_publisher(String, '/robot/tts_say', 10)

        # ─── Subscribers ───
        self.create_subscription(String, '/voice/command',         self._voice_cb,   10)
        self.create_subscription(String, '/hailo/gesture_cmd',     self._gesture_cb, 10)
        self.create_subscription(String, '/hailo/faces',           self._faces_cb,   10)
        self.create_subscription(Twist,  '/hailo/cmd_vel',         self._vel_gesture_cb, 5)
        self.create_subscription(Twist,  '/voice/cmd_vel',         self._vel_voice_cb,   5)
        self.create_subscription(Twist,  '/person_follower/cmd_vel', self._vel_follow_cb, 5)
        self.create_subscription(Twist,  '/joy_cmd_vel',           self._vel_joy_cb,   5)
        self.create_subscription(Joy,    'joy',                    self._joy_cb,      10)

        # ─── Timers ───
        period = 1.0 / self._pub_rate
        self.create_timer(period, self._arbitration_cb)
        self.create_timer(1.0,    self._status_cb)

        # ─── Publicar modo inicial ───
        self._publish_mode()
        self.get_logger().info(
            f'🎛️  Robot Mode Manager activo | modo={self._mode} | '
            f'TTS={"✅" if self._tts_enabled else "❌"}')

    # ══════════════════════════════════════════════
    #  Velocity callbacks
    # ══════════════════════════════════════════════

    def _vel_gesture_cb(self, msg: Twist):
        self._vel['gesture'] = msg
        self._vel_t['gesture'] = time.time()

    def _vel_voice_cb(self, msg: Twist):
        self._vel['voice'] = msg
        self._vel_t['voice'] = time.time()

    def _vel_follow_cb(self, msg: Twist):
        self._vel['follow'] = msg
        self._vel_t['follow'] = time.time()

    def _vel_joy_cb(self, msg: Twist):
        self._vel['joy'] = msg
        self._vel_t['joy'] = time.time()
        # Detectar actividad del joystick
        if (abs(msg.linear.x) > self._joy_deadband or
                abs(msg.angular.z) > self._joy_deadband):
            self._joy_active = True
            self._joy_last_t = time.time()

    # ══════════════════════════════════════════════
    #  Joystick callback
    # ══════════════════════════════════════════════

    def _joy_cb(self, msg: Joy):
        now = time.time()

        # Botón ciclador de modo (Y / btn 3)
        if len(msg.buttons) > self._joy_mode_btn:
            btn_val = msg.buttons[self._joy_mode_btn]
            if btn_val == 1 and self._prev_mode_btn == 0:
                self._cycle_mode_via_joy()
            self._prev_mode_btn = btn_val

        # Deadman (L1 / btn 4) — activar TELEOP al presionar
        if len(msg.buttons) > self._joy_deadman_btn:
            dl = msg.buttons[self._joy_deadman_btn]
            if dl == 1 and self._prev_deadman == 0:
                if self._mode != MODE_TELEOP:
                    self._set_mode(MODE_TELEOP, source='joystick')
            self._prev_deadman = dl

        self._joy_last_t = now

    def _cycle_mode_via_joy(self):
        """Cicla entre modos disponibles con el botón Y del joystick."""
        cycle = [MODE_IDLE, MODE_TELEOP, MODE_GESTURE, MODE_VOICE, MODE_FOLLOW]
        cycle = [m for m in cycle if m in self._allowed_modes]
        try:
            idx = cycle.index(self._mode)
            next_mode = cycle[(idx + 1) % len(cycle)]
        except ValueError:
            next_mode = cycle[0]
        self._set_mode(next_mode, source='joy_btn')

    # ══════════════════════════════════════════════
    #  Voice command callback
    # ══════════════════════════════════════════════

    def _voice_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        cmd = data.get('cmd', '')

        if cmd == 'set_mode':
            new_mode = data.get('mode', '').upper()
            if new_mode in ALL_MODES:
                self._set_mode(new_mode, source='voice')

        elif cmd == 'stop':
            self._set_mode(MODE_IDLE, source='voice')
            self._publish_stop()

        elif cmd == 'move':
            # En modo VOICE, la velocidad ya viene de /voice/cmd_vel
            if self._mode != MODE_VOICE:
                self._set_mode(MODE_VOICE, source='voice_move')

        elif cmd in CMD_RESPONSES:
            self._say(CMD_RESPONSES[cmd])

        elif cmd == 'speed':
            # Ajuste de velocidad: no cambia modo
            delta = data.get('delta', 0.0)
            self.get_logger().info(f'🔊 Speed delta: {delta:+.2f}')
            # TODO: podría reenviar a los nodos relevantes

    # ══════════════════════════════════════════════
    #  Gesture command callback
    # ══════════════════════════════════════════════

    def _gesture_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        cmd = data.get('cmd', '')

        if cmd == 'set_mode':
            new_mode = data.get('mode', '').upper()
            source   = data.get('source', 'gesture')
            if new_mode in ALL_MODES:
                self._set_mode(new_mode, source=source)

        elif cmd == 'move':
            # Solo actuar si estamos en modo GESTURE
            if self._mode != MODE_GESTURE:
                self.get_logger().debug(
                    f'Gesto ignorado (modo actual: {self._mode})')

    # ══════════════════════════════════════════════
    #  Face detection callback
    # ══════════════════════════════════════════════

    def _faces_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        faces = data.get('faces', [])
        named = [f for f in faces if f.get('name')]
        if named:
            self._last_face_t = time.time()
            self._authorized_user = named[0]['name']

    # ══════════════════════════════════════════════
    #  Main arbitration timer
    # ══════════════════════════════════════════════

    def _arbitration_cb(self):
        """Publica la velocidad correcta según el modo actual."""
        now = time.time()
        twist = Twist()

        if self._mode == MODE_IDLE:
            pass  # twist = zero

        elif self._mode == MODE_TELEOP:
            age = now - self._vel_t['joy']
            if age < self._joy_timeout:
                twist = self._vel['joy']
            # Timeout de joystick → IDLE
            elif age > self._joy_timeout and self._joy_active:
                self.get_logger().info(
                    f'⏰ Joystick inactivo ({age:.1f}s) — volviendo a IDLE')
                self._joy_active = False
                self._set_mode(MODE_IDLE, source='timeout')

        elif self._mode == MODE_GESTURE:
            age = now - self._vel_t['gesture']
            if age < self._cmd_to_gesture:
                twist = self._vel['gesture']

        elif self._mode == MODE_VOICE:
            age = now - self._vel_t['voice']
            if age < self._cmd_to_voice:
                twist = self._vel['voice']

        elif self._mode == MODE_FOLLOW:
            age = now - self._vel_t['follow']
            if age < self._cmd_to_follow:
                twist = self._vel['follow']

        self._cmd_pub.publish(twist)

    # ══════════════════════════════════════════════
    #  Mode transitions
    # ══════════════════════════════════════════════

    def _set_mode(self, new_mode: str, source: str = 'unknown'):
        if new_mode not in self._allowed_modes:
            self.get_logger().warn(f'Modo {new_mode} no permitido')
            return
        if new_mode == self._mode:
            return

        old_mode = self._mode
        self._mode = new_mode

        self.get_logger().info(
            f'🔄 Modo: {old_mode} → {new_mode}  [via {source}]')

        # Parar robot al cambiar de modo
        self._publish_stop()

        # Publicar nuevo modo
        self._publish_mode()

        # TTS
        tts_msg = MODE_MESSAGES.get(new_mode, f'Modo {new_mode} activado.')
        self._say(tts_msg)

    def _publish_mode(self):
        msg = String()
        msg.data = self._mode
        self._mode_pub.publish(msg)

    def _publish_stop(self):
        self._cmd_pub.publish(Twist())

    # ══════════════════════════════════════════════
    #  Status timer
    # ══════════════════════════════════════════════

    def _status_cb(self):
        msg = String()
        msg.data = json.dumps({
            'mode':           self._mode,
            'authorized_user': self._authorized_user,
            'face_age':       round(time.time() - self._last_face_t, 1),
            'vel_ages': {
                k: round(time.time() - v, 2)
                for k, v in self._vel_t.items()
            },
            'joy_active': self._joy_active,
        })
        self._status_pub.publish(msg)

    # ══════════════════════════════════════════════
    #  TTS (espeak)
    # ══════════════════════════════════════════════

    def _say(self, text: str):
        """Reproduce texto con espeak en hilo separado."""
        # Publicar en topic (para sistemas con audio dedicado)
        tmsg = String()
        tmsg.data = text
        self._tts_pub.publish(tmsg)

        if not self._tts_enabled:
            return

        def _run():
            with self._tts_lock:
                try:
                    subprocess.run(
                        ['espeak', '-v', f'{self._tts_voice}+f3',
                         '-s', '140', '-a', '180', text],
                        timeout=10,
                        capture_output=True,
                    )
                except FileNotFoundError:
                    # espeak no instalado — silencio
                    pass
                except subprocess.TimeoutExpired:
                    pass
                except Exception as e:
                    self.get_logger().debug(f'TTS error: {e}')

        # Cancelar TTS anterior y lanzar nuevo
        if self._tts_thread and self._tts_thread.is_alive():
            # No podemos cancelar fácilmente espeak, pero al menos no bloqueamos
            pass
        self._tts_thread = threading.Thread(target=_run, daemon=True)
        self._tts_thread.start()


# ─── main ─────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RobotModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_stop()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
