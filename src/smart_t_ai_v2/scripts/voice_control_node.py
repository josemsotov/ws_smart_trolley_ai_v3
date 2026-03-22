#!/usr/bin/env python3
"""
ROS2 Node: Voice Control — Reconocimiento de voz offline (Vosk) para el robot
==============================================================================

Captura audio del micrófono en tiempo real y reconoce comandos en español
usando el motor Vosk (100% offline, no necesita internet).

Cuando se detecta un comando, publica en /voice/command un JSON con:
  {"cmd": "set_mode", "mode": "GESTURE", "text": "modo gesto", "confidence": 0.95}
  {"cmd": "move",     "direction": "forward", "text": "adelante"}
  {"cmd": "stop",     "text": "para"}
  {"cmd": "greet",    "text": "hola"}
  {"cmd": "status",   "text": "estado"}

Comandos reconocidos (español):
  ┌─────────────────────────────────────────────────────────────┐
  │  MODOS                                                       │
  │  "modo gesto" / "gestos"          → set_mode GESTURE         │
  │  "modo voz" / "control voz"       → set_mode VOICE           │
  │  "modo manual" / "joystick"       → set_mode TELEOP          │
  │  "sígueme" / "sígame" / "follow"  → set_mode FOLLOW          │
  │  "modo auto" / "autónomo"         → set_mode FOLLOW          │
  │                                                               │
  │  MOVIMIENTO (en modo VOICE)                                   │
  │  "adelante" / "avanza"            → move forward             │
  │  "atrás" / "retrocede"            → move backward            │
  │  "izquierda" / "gira izquierda"   → move left                │
  │  "derecha" / "gira derecha"       → move right               │
  │  "gira" / "rotar"                 → move spin                │
  │                                                               │
  │  CONTROL                                                      │
  │  "para" / "stop" / "detente"      → stop                     │
  │  "más rápido" / "acelera"         → speed_up                 │
  │  "más lento" / "reduce"           → speed_down               │
  │                                                               │
  │  INTERACCIÓN                                                  │
  │  "hola" / "hey"                   → greet                    │
  │  "estado" / "cómo estás"          → status                   │
  │  "quién eres" / "nombre"          → identify                 │
  └─────────────────────────────────────────────────────────────┘

Topics publicados:
  /voice/command    (std_msgs/String) - JSON con comando reconocido
  /voice/transcript (std_msgs/String) - transcripción cruda
  /voice/status     (std_msgs/String) - estado del nodo

Topics suscritos:
  /robot/mode       (std_msgs/String) - modo actual del robot

Requires:
  - vosk==0.3.45    (pip install vosk)
  - pyaudio         (apt install python3-pyaudio)
  - Modelo Vosk ES: ~/.vosk/models/vosk-model-small-es  (descargado automáticamente)

Author: Smart Trolley V5 Project
"""

import os
import json
import time
import wave
import queue
import struct
import threading
import urllib.request
import zipfile

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# ─── Imports opcionales ───────────────────────────────────────────────────────
try:
    import pyaudio
    PYAUDIO_OK = True
except ImportError:
    PYAUDIO_OK = False

try:
    from vosk import Model, KaldiRecognizer, SetLogLevel
    VOSK_OK = True
except ImportError:
    VOSK_OK = False

# ─── Constantes ───────────────────────────────────────────────────────────────
VOSK_MODELS_DIR  = os.path.expanduser('~/.vosk/models')
VOSK_MODEL_ES    = os.path.join(VOSK_MODELS_DIR, 'vosk-model-small-es')
VOSK_MODEL_URL   = (
    'https://alphacephei.com/vosk/models/vosk-model-small-es-0.42.zip'
)

SAMPLE_RATE  = 16000
CHUNK_FRAMES = 8000   # 0.5s de audio por chunk

# ─── Mapa de comandos de voz ──────────────────────────────────────────────────
# Formato: 'frase_clave': {'cmd': ..., 'params': {...}}
# Las frases se detectan si ESTÁN CONTENIDAS en la transcripción.

VOICE_COMMANDS = [
    # ── Modos ──
    {'keywords': ['modo gesto', 'gestos', 'control gesto', 'modo gestos'],
     'action': {'cmd': 'set_mode', 'mode': 'GESTURE'}},
    {'keywords': ['modo voz', 'control voz', 'voz'],
     'action': {'cmd': 'set_mode', 'mode': 'VOICE'}},
    {'keywords': ['modo manual', 'joystick', 'control manual', 'manual'],
     'action': {'cmd': 'set_mode', 'mode': 'TELEOP'}},
    {'keywords': ['sígueme', 'sígame', 'seguirme', 'follow', 'modo auto',
                  'modo autónomo', 'autónomo'],
     'action': {'cmd': 'set_mode', 'mode': 'FOLLOW'}},
    {'keywords': ['modo idle', 'desactivar', 'apagar'],
     'action': {'cmd': 'set_mode', 'mode': 'IDLE'}},

    # ── Movimiento (activo en modo VOICE) ──
    {'keywords': ['adelante', 'avanza', 'avanzar', 'hacia adelante'],
     'action': {'cmd': 'move', 'direction': 'forward'}},
    {'keywords': ['atrás', 'retrocede', 'retroceder', 'hacia atrás', 'marcha atrás'],
     'action': {'cmd': 'move', 'direction': 'backward'}},
    {'keywords': ['izquierda', 'gira izquierda', 'a la izquierda', 'girar izquierda'],
     'action': {'cmd': 'move', 'direction': 'left'}},
    {'keywords': ['derecha', 'gira derecha', 'a la derecha', 'girar derecha'],
     'action': {'cmd': 'move', 'direction': 'right'}},
    {'keywords': ['gira', 'rotar', 'girar', 'vuelta'],
     'action': {'cmd': 'move', 'direction': 'spin'}},

    # ── Control ──
    {'keywords': ['para', 'stop', 'detente', 'detener', 'frenar', 'frena', 'quieto'],
     'action': {'cmd': 'stop'}},
    {'keywords': ['más rápido', 'acelera', 'acelerear', 'rápido', 'rapido'],
     'action': {'cmd': 'speed', 'delta': 0.05}},
    {'keywords': ['más lento', 'reduce', 'lento', 'despacio'],
     'action': {'cmd': 'speed', 'delta': -0.05}},

    # ── Interacción ──
    {'keywords': ['hola', 'hey', 'buenos días', 'buenas'],
     'action': {'cmd': 'greet'}},
    {'keywords': ['estado', 'cómo estás', 'como estas', 'informe'],
     'action': {'cmd': 'status'}},
    {'keywords': ['quién eres', 'quien eres', 'tu nombre', 'cómo te llamas',
                  'como te llamas', 'nombre'],
     'action': {'cmd': 'identify'}},
    {'keywords': ['ayuda', 'qué puedes hacer', 'que puedes hacer', 'comandos'],
     'action': {'cmd': 'help'}},
]


class VoiceControlNode(Node):
    """Nodo de reconocimiento de voz offline para control del robot."""

    def __init__(self):
        super().__init__('voice_control')

        # ─── Parámetros ───
        self.declare_parameter('model_path',   VOSK_MODEL_ES)
        self.declare_parameter('language',     'es')
        self.declare_parameter('sample_rate',  SAMPLE_RATE)
        self.declare_parameter('device_index', -1)
        self.declare_parameter('publish_transcript', True)
        self.declare_parameter('min_confidence', 0.55)
        self.declare_parameter('echo_commands', True)
        self.declare_parameter('auto_download_model', True)
        self.declare_parameter('linear_speed',  0.20)
        self.declare_parameter('angular_speed', 0.45)
        self.declare_parameter('voice_move_duration', 1.5)   # segundos por comando

        p = self.get_parameter
        self._model_path         = p('model_path').value
        self._sample_rate        = p('sample_rate').value
        self._device_index       = p('device_index').value
        self._pub_transcript     = p('publish_transcript').value
        self._min_conf           = p('min_confidence').value
        self._echo               = p('echo_commands').value
        self._auto_dl            = p('auto_download_model').value
        self._lin_speed          = p('linear_speed').value
        self._ang_speed          = p('angular_speed').value
        self._move_duration      = p('voice_move_duration').value

        # ─── Estado ───
        self._robot_mode   = 'IDLE'
        self._running      = False
        self._audio_queue  = queue.Queue(maxsize=20)
        self._last_cmd     = ''
        self._last_cmd_t   = 0.0
        self._cmd_cooldown = 1.2   # segundos entre comandos iguales
        self._move_thread  = None
        self._move_stop    = threading.Event()

        # ─── Publishers ───
        self._cmd_pub   = self.create_publisher(String, '/voice/command',    10)
        self._trans_pub = self.create_publisher(String, '/voice/transcript', 10)
        self._stat_pub  = self.create_publisher(String, '/voice/status',     10)
        self._cmdvel_pub = self.create_publisher(Twist, '/voice/cmd_vel',    10)

        # ─── Subscribers ───
        self.create_subscription(String, '/robot/mode', self._mode_cb, 10)

        # ─── Verificar dependencias ───
        if not VOSK_OK:
            self.get_logger().error('❌ vosk no instalado. Ejecuta: pip3 install vosk')
            self._publish_status('ERROR_NO_VOSK')
            return

        if not PYAUDIO_OK:
            self.get_logger().error(
                '❌ pyaudio no instalado. Ejecuta: apt install python3-pyaudio')
            self._publish_status('ERROR_NO_PYAUDIO')
            return

        # ─── Descargar modelo si no existe ───
        if not os.path.exists(self._model_path):
            if self._auto_dl:
                self.get_logger().info(
                    f'📥 Descargando modelo Vosk ES → {VOSK_MODELS_DIR}...')
                ok = self._download_model()
                if not ok:
                    self.get_logger().error('❌ No se pudo descargar el modelo Vosk')
                    self._publish_status('ERROR_NO_MODEL')
                    return
            else:
                self.get_logger().error(
                    f'❌ Modelo Vosk no encontrado en {self._model_path}\n'
                    '   Descárgalo con: download_hailo_models.sh o '
                    '   ros2 param set /voice_control auto_download_model true')
                self._publish_status('ERROR_NO_MODEL')
                return

        # ─── Cargar modelo Vosk ───
        SetLogLevel(-1)
        self.get_logger().info(f'📚 Cargando modelo Vosk desde {self._model_path}...')
        try:
            self._model = Model(self._model_path)
            self._recognizer = KaldiRecognizer(self._model, float(self._sample_rate))
            self._recognizer.SetWords(True)
        except Exception as e:
            self.get_logger().error(f'❌ Error cargando modelo Vosk: {e}')
            self._publish_status('ERROR_MODEL_LOAD')
            return

        # ─── Iniciar audio ───
        ok = self._init_audio()
        if not ok:
            return

        # ─── Iniciar hilo de reconocimiento ───
        self._running = True
        self._asr_thread = threading.Thread(target=self._asr_loop, daemon=True)
        self._asr_thread.start()

        self.get_logger().info(
            '🎤 Voice Control activo | '
            f'Vosk ES | Sample rate: {self._sample_rate}Hz | '
            f'Dispositivo: {self._device_index if self._device_index >= 0 else "auto"}')
        self._publish_status('LISTENING')

    # ══════════════════════════════════════════════
    #  Descarga de modelo
    # ══════════════════════════════════════════════

    def _download_model(self) -> bool:
        """Descarga el modelo Vosk español pequeño."""
        try:
            os.makedirs(VOSK_MODELS_DIR, exist_ok=True)
            zip_path = os.path.join(VOSK_MODELS_DIR, 'vosk-model-small-es.zip')
            self.get_logger().info(f'  Descargando desde: {VOSK_MODEL_URL}')

            def _progress(count, block, total):
                pct = int(count * block * 100 / max(total, 1))
                if pct % 10 == 0:
                    self.get_logger().info(f'  ... {pct}%')

            urllib.request.urlretrieve(VOSK_MODEL_URL, zip_path, reporthook=_progress)
            self.get_logger().info('  Descomprimiendo...')

            with zipfile.ZipFile(zip_path, 'r') as zf:
                zf.extractall(VOSK_MODELS_DIR)

            # El zip suele extraer a vosk-model-small-es-0.42, renombrar
            extracted = None
            for d in os.listdir(VOSK_MODELS_DIR):
                if d.startswith('vosk-model-small-es') and d != 'vosk-model-small-es':
                    extracted = os.path.join(VOSK_MODELS_DIR, d)
                    break
            if extracted and os.path.isdir(extracted):
                os.rename(extracted, VOSK_MODEL_ES)

            os.remove(zip_path)
            self.get_logger().info(f'✅ Modelo Vosk instalado en {VOSK_MODEL_ES}')
            return True
        except Exception as e:
            self.get_logger().error(f'Error descargando modelo: {e}')
            return False

    # ══════════════════════════════════════════════
    #  Audio initialization
    # ══════════════════════════════════════════════

    def _init_audio(self) -> bool:
        """Inicializa PyAudio y selecciona dispositivo de entrada."""
        try:
            self._pa = pyaudio.PyAudio()

            # Listar dispositivos disponibles
            n_devices = self._pa.get_device_count()
            self.get_logger().info(f'🎧 Dispositivos de audio ({n_devices} total):')
            default_input = self._pa.get_default_input_device_info()['index']

            for i in range(n_devices):
                info = self._pa.get_device_info_by_index(i)
                if info['maxInputChannels'] > 0:
                    marker = '← DEFAULT' if i == default_input else ''
                    self.get_logger().info(
                        f'  [{i}] {info["name"][:50]} {marker}')

            # Seleccionar dispositivo
            if self._device_index < 0:
                self._device_index = default_input
                self.get_logger().info(
                    f'  Usando dispositivo por defecto: [{default_input}]')

            # Abrir stream
            self._stream = self._pa.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self._sample_rate,
                input=True,
                input_device_index=self._device_index,
                frames_per_buffer=CHUNK_FRAMES,
                stream_callback=self._audio_callback,
            )
            self._stream.start_stream()
            return True

        except Exception as e:
            self.get_logger().error(f'❌ Error abriendo micrófono: {e}')
            self.get_logger().error(
                '   Verifica que hay un micrófono conectado: arecord -l')
            self._publish_status('ERROR_NO_AUDIO')
            return False

    def _audio_callback(self, in_data, frame_count, time_info, status):
        """Callback de PyAudio — pone audio en la cola."""
        try:
            self._audio_queue.put_nowait(in_data)
        except queue.Full:
            pass  # Drop frame
        return (None, pyaudio.paContinue)

    # ══════════════════════════════════════════════
    #  ASR loop (hilo separado)
    # ══════════════════════════════════════════════

    def _asr_loop(self):
        """Hilo principal de reconocimiento de voz."""
        self.get_logger().info('🎤 ASR loop iniciado')

        while self._running:
            try:
                audio_data = self._audio_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if self._recognizer.AcceptWaveform(audio_data):
                result = json.loads(self._recognizer.Result())
                text = result.get('text', '').strip().lower()
                conf = result.get('confidence', 0.0)
            else:
                # Resultado parcial — solo loguear, no actuar
                partial = json.loads(self._recognizer.PartialResult())
                partial_text = partial.get('partial', '')
                if partial_text:
                    pass  # Podría publicar transcript parcial aquí
                continue

            if not text:
                continue

            self.get_logger().info(f'🗣️  Transcripción: "{text}" (conf={conf:.2f})')

            # Publicar transcripción cruda
            if self._pub_transcript:
                tmsg = String()
                tmsg.data = text
                self._trans_pub.publish(tmsg)

            # Filtrar por confianza (si disponible)
            if conf > 0 and conf < self._min_conf:
                self.get_logger().debug(
                    f'  Ignorado (confianza {conf:.2f} < {self._min_conf})')
                continue

            # Detectar comando
            self._process_transcript(text, conf)

    def _process_transcript(self, text: str, conf: float):
        """Busca el comando en la transcripción y lo publica."""
        text_norm = self._normalize(text)
        now = time.time()

        matched_action = None
        for entry in VOICE_COMMANDS:
            for kw in entry['keywords']:
                if self._normalize(kw) in text_norm:
                    matched_action = entry['action'].copy()
                    matched_action['text'] = text
                    matched_action['confidence'] = conf
                    matched_action['keyword'] = kw
                    break
            if matched_action:
                break

        if matched_action is None:
            self.get_logger().debug(f'  Sin coincidencia para: "{text}"')
            return

        # Cooldown para evitar repeticiones rápidas del mismo comando
        cmd_key = json.dumps(matched_action.get('cmd', '') + matched_action.get('mode', '') +
                             matched_action.get('direction', ''))
        if cmd_key == self._last_cmd and (now - self._last_cmd_t) < self._cmd_cooldown:
            return

        self._last_cmd   = cmd_key
        self._last_cmd_t = now

        self.get_logger().info(
            f'✅ Comando: {matched_action["cmd"]} | '
            f'keyword="{matched_action.get("keyword", "")}"')

        # Publicar comando
        msg = String()
        msg.data = json.dumps(matched_action)
        self._cmd_pub.publish(msg)

        # Si estamos en modo VOICE y es un movimiento, ejecutar directo
        if (self._robot_mode == 'VOICE' and
                matched_action.get('cmd') == 'move'):
            self._execute_voice_move(matched_action.get('direction', ''))

    def _normalize(self, text: str) -> str:
        """Normaliza texto: minúsculas, quita tildes extra."""
        replacements = {
            'á': 'a', 'é': 'e', 'í': 'i', 'ó': 'o', 'ú': 'u',
            'ü': 'u', 'ñ': 'n',
        }
        t = text.lower()
        for src, dst in replacements.items():
            t = t.replace(src, dst)
        return t

    # ══════════════════════════════════════════════
    #  Voice movement execution
    # ══════════════════════════════════════════════

    def _execute_voice_move(self, direction: str):
        """Mueve el robot durante voice_move_duration segundos."""
        # Cancelar movimiento anterior
        self._move_stop.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=0.5)
        self._move_stop.clear()

        self._move_thread = threading.Thread(
            target=self._move_worker,
            args=(direction,),
            daemon=True,
        )
        self._move_thread.start()

    def _move_worker(self, direction: str):
        """Publica cmd_vel por voice_move_duration segundos."""
        twist = Twist()
        lin, ang = self._lin_speed, self._ang_speed

        if   direction == 'forward':   twist.linear.x =  lin
        elif direction == 'backward':  twist.linear.x = -lin
        elif direction == 'left':      twist.angular.z =  ang
        elif direction == 'right':     twist.angular.z = -ang
        elif direction == 'spin':
            twist.angular.z = ang * 1.5

        rate_hz  = 20.0
        interval = 1.0 / rate_hz
        n_iters  = int(self._move_duration * rate_hz)

        for _ in range(n_iters):
            if self._move_stop.is_set():
                break
            self._cmdvel_pub.publish(twist)
            time.sleep(interval)

        # Parar al terminar
        self._cmdvel_pub.publish(Twist())

    # ══════════════════════════════════════════════
    #  Callbacks
    # ══════════════════════════════════════════════

    def _mode_cb(self, msg: String):
        prev = self._robot_mode
        self._robot_mode = msg.data
        if prev != self._robot_mode:
            self.get_logger().info(f'🔄 Modo cambiado a: {self._robot_mode}')
            # Parar movimiento si salimos de modo VOICE
            if prev == 'VOICE':
                self._move_stop.set()
                self._cmdvel_pub.publish(Twist())

    # ══════════════════════════════════════════════
    #  Status publisher
    # ══════════════════════════════════════════════

    def _publish_status(self, status: str):
        msg = String()
        msg.data = json.dumps({
            'status': status,
            'robot_mode': self._robot_mode,
            'vosk': VOSK_OK,
            'pyaudio': PYAUDIO_OK,
        })
        self._stat_pub.publish(msg)

    # ══════════════════════════════════════════════
    #  Cleanup
    # ══════════════════════════════════════════════

    def destroy_node(self):
        self._running = False
        self._move_stop.set()
        try:
            self._stream.stop_stream()
            self._stream.close()
            self._pa.terminate()
        except Exception:
            pass
        super().destroy_node()


# ─── main ─────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
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
