#!/usr/bin/env python3
"""
ROS2 Node: Hailo AI Face Detection + MediaPipe Hand Gesture Recognition
========================================================================

Usa el Hailo AI HAT2+ (HAILO10H) para detección de caras acelerada por hardware
y MediaPipe para reconocimiento de gestos con la mano.

Pipeline:
  1. Recibe imagen RGB del Kinect
  2. Detecta caras con Hailo SCRFD 10G (o scrfd_2.5g / Haar cascade como fallback)
  3. Reconoce usuario con LBPH (modelo entrenado con register_face.py)
  4. Detecta landmarks de manos con MediaPipe Hands
  5. Clasifica gesto por conteo de dedos
  6. Publica comandos y cambios de modo del robot

Mapeo de gestos → comandos:
  ✊ 0 dedos (puño)      → STOP / desactivar modo
  ☝️ 1 dedo              → ADELANTE
  ✌️ 2 dedos             → ATRÁS
  🤟 3 dedos             → IZQUIERDA
  🖖 4 dedos             → DERECHA
  🖐 5 dedos (palma)     → SIGUEME / activar FOLLOW mode (mantener 2s)

Cambio de modo por gestos especiales (mantener 2s):
  Pulgar arriba (thumb_up)  → activar GESTURE mode
  Palma abierta 2s          → cambiar a FOLLOW mode
  Puño 2s                   → volver a IDLE

Topics publicados:
  /hailo/faces          (std_msgs/String)    - JSON con caras detectadas
  /hailo/gesture_cmd    (std_msgs/String)    - comando de gesto actual
  /hailo/cmd_vel        (geometry_msgs/Twist) - velocidad del robot
  /hailo/annotated      (sensor_msgs/Image)  - frame anotado
  /hailo/status         (std_msgs/String)    - estado JSON del nodo

Topics suscritos:
  /kinect/rgb/image_raw (sensor_msgs/Image) - RGB del Kinect
  /robot/mode           (std_msgs/String)   - modo actual del robot

Requires:
  - hailort 5.2.0 + hailo_platform
  - mediapipe 0.10+
  - opencv-contrib (LBPH)
  - HEF model en models/scrfd_10g.hef (recomendado para HAILO10H)
    o models/scrfd_2.5g.hef (alternativa ligera)
    Descarga con: tools/download_hailo_models.sh

Author: Smart Trolley V5 Project
"""

import os
import json
import time
import math
import threading

import cv2
import numpy as np
import mediapipe as mp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ─── Hailo Platform ───────────────────────────────────────────────────────────
try:
    from hailo_platform import (
        VDevice, HEF, FormatType, HailoSchedulingAlgorithm,
    )
    HAILO_AVAILABLE = True
except ImportError:
    HAILO_AVAILABLE = False

# ─── Constantes ───────────────────────────────────────────────────────────────
HAAR_CASCADE = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
FACES_DIR = os.path.expanduser('~/.smart_trolley/faces')
LBPH_MODEL = os.path.join(FACES_DIR, 'lbph_model.yml')
LABEL_MAP  = os.path.join(FACES_DIR, 'label_map.json')

GESTURE_NAMES = {
    -1: 'NONE',
     0: 'STOP',
     1: 'ADELANTE',
     2: 'ATRAS',
     3: 'IZQUIERDA',
     4: 'DERECHA',
     5: 'SIGUEME',
     6: 'THUMB_UP',
     7: 'THUMB_DOWN',
}

GESTURE_CMDS = {
    0: 'STOP',
    1: 'FORWARD',
    2: 'BACKWARD',
    3: 'LEFT',
    4: 'RIGHT',
    5: 'FOLLOW',
    6: 'MODE_GESTURE',
    7: 'MODE_IDLE',
}

# Estados del nodo
STATE_DISABLED    = 'DISABLED'
STATE_WAITING     = 'WAITING'       # Esperando cara autorizada
STATE_AUTHORIZED  = 'AUTHORIZED'    # Cara reconocida, esperando gesto
STATE_EXECUTING   = 'EXECUTING'     # Ejecutando comando
STATE_MODE_CHANGE = 'MODE_CHANGE'   # Gesto de cambio de modo en progreso

# Colores BGR
C_RED    = (0, 0, 255)
C_GREEN  = (0, 255, 0)
C_BLUE   = (255, 0, 0)
C_YELLOW = (0, 255, 255)
C_CYAN   = (255, 255, 0)
C_WHITE  = (255, 255, 255)
C_ORANGE = (0, 165, 255)
C_PURPLE = (255, 0, 255)


# ─── Hailo Inference Wrapper ─────────────────────────────────────────────────
# SCRFD output tensor names (por modelo)
_SCRFD_TENSORS = {
    'scrfd_10g': {
        'cls':  ['scrfd_10g/conv41', 'scrfd_10g/conv49', 'scrfd_10g/conv56'],
        'bbox': ['scrfd_10g/conv42', 'scrfd_10g/conv50', 'scrfd_10g/conv57'],
        'lm':   ['scrfd_10g/conv43', 'scrfd_10g/conv51', 'scrfd_10g/conv58'],
    },
    'scrfd_2_5g': {
        'cls':  ['scrfd_2_5g/conv42', 'scrfd_2_5g/conv49', 'scrfd_2_5g/conv55'],
        'bbox': ['scrfd_2_5g/conv43', 'scrfd_2_5g/conv50', 'scrfd_2_5g/conv56'],
        'lm':   ['scrfd_2_5g/conv44', 'scrfd_2_5g/conv51', 'scrfd_2_5g/conv57'],
    },
}
_SCRFD_STRIDES    = [8, 16, 32]   # feature map strides
_SCRFD_NUM_ANCHORS = 2            # 2 anchors por celda
_SCRFD_INPUT_SIZE  = 640          # modelo 640×640


class HailoFaceDetector:
    """
    Detección de caras con Hailo SCRFD usando la API nueva de HAILO10H.

    API: VDevice.create_params() + create_infer_model() + run_async()
    Decode: SCRFD anchor-based ltrb con NMS por OpenCV.
    """

    def __init__(self, hef_path: str, score_thr: float = 0.45,
                 iou_thr: float = 0.4, logger=None):
        self._log       = logger
        self._ready     = False
        self._vdevice   = None
        self._infer_model = None
        self._config_ctx  = None
        self._configured  = None
        self._score_thr   = score_thr
        self._iou_thr     = iou_thr
        self._input_shape = (640, 640, 3)  # H, W, C
        self._anchors     = {}            # precomputados por stride
        self._tensor_cfg  = None

        if not HAILO_AVAILABLE:
            if self._log:
                self._log.warn('hailo_platform no disponible — usando fallback OpenCV')
            return
        if not os.path.exists(hef_path):
            if self._log:
                self._log.warn(f'HEF no encontrado: {hef_path} — usando fallback OpenCV')
            return

        try:
            from hailo_platform import HailoSchedulingAlgorithm
            params = VDevice.create_params()
            params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
            params.group_id = 'SHARED'
            self._vdevice = VDevice(params)

            self._infer_model = self._vdevice.create_infer_model(hef_path)
            self._infer_model.set_batch_size(1)
            self._infer_model.input().set_format_type(FormatType.UINT8)
            for out in self._infer_model.outputs:
                self._infer_model.output(out.name).set_format_type(FormatType.FLOAT32)

            self._config_ctx = self._infer_model.configure()
            self._configured = self._config_ctx.__enter__()

            # Detectar qué variante de SCRFD es por los nombres de output
            out_names = set(out.name for out in self._infer_model.outputs)
            for variant, cfg in _SCRFD_TENSORS.items():
                if cfg['cls'][0] in out_names:
                    self._tensor_cfg = cfg
                    break
            if self._tensor_cfg is None:
                raise RuntimeError(f'Modelo SCRFD desconocido. Outputs: {out_names}')

            # Precomputar anchors para cada escala
            self._anchors = self._build_anchors()

            # Input shape
            shape = self._infer_model.input().shape
            self._input_shape = tuple(shape)  # (H, W, C)

            self._ready = True
            if self._log:
                self._log.info(
                    f'✅ Hailo SCRFD cargado: {os.path.basename(hef_path)} | '
                    f'input={self._input_shape} | score_thr={score_thr}')
        except Exception as e:
            if self._log:
                self._log.error(f'Error inicializando Hailo: {e}')
            self._cleanup()

    # ── Anchor precomputation ────────────────────────────────────────────────

    @property
    def ready(self):
        return self._ready

    @property
    def input_shape(self):
        return self._input_shape  # (H, W, C)

    def _build_anchors(self) -> dict:
        """
        Genera anchors por stride.
        Formato: [cy_px, cx_px] — centros de cada anchor en píxeles del input 640×640.
        Los offsets bbox/landmark del SCRFD son en unidades de stride píxeles.
        Decode:
          x1 = (cx_px - l_raw * stride) / INPUT_SIZE * orig_w
          x2 = (cx_px + r_raw * stride) / INPUT_SIZE * orig_w
        """
        anchors = {}
        H = W = _SCRFD_INPUT_SIZE
        for stride in _SCRFD_STRIDES:
            feat_h = H // stride
            feat_w = W // stride
            rows = np.arange(feat_h, dtype=np.float32)
            cols = np.arange(feat_w, dtype=np.float32)
            grid_y, grid_x = np.meshgrid(rows, cols, indexing='ij')  # [feat_h, feat_w]
            # Centro del anchor en píxeles del input 640×640
            cy_px = (grid_y.reshape(-1) + 0.5) * stride  # [N_cells]
            cx_px = (grid_x.reshape(-1) + 0.5) * stride
            # 2 anchors por celda (mismo centro, mismo decode — solo 2x más detecciones)
            cy_px = np.repeat(cy_px, _SCRFD_NUM_ANCHORS)
            cx_px = np.repeat(cx_px, _SCRFD_NUM_ANCHORS)
            anchors[stride] = np.stack([cy_px, cx_px], axis=1)  # [N, 2]
        return anchors

    # ── Inference ────────────────────────────────────────────────────────────

    def detect(self, bgr_image: np.ndarray) -> list:
        """
        Corre inferencia SCRFD con Hailo HAILO10H.
        Retorna lista de dicts:
          [{'bbox': [x1,y1,x2,y2], 'score': float, 'landmarks': [(x,y)×5]}]
        bbox en píxeles del frame original.
        """
        if not self._ready:
            return []

        orig_h, orig_w = bgr_image.shape[:2]
        tgt_h, tgt_w = self._input_shape[0], self._input_shape[1]

        # Preprocesar: resize + BGR→RGB
        resized = cv2.resize(bgr_image, (tgt_w, tgt_h))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        try:
            raw = self._run_sync(rgb)
        except Exception as e:
            if self._log:
                self._log.warn(f'Hailo infer error: {e}', throttle_duration_sec=5.0)
            return []

        return self._decode(raw, orig_w, orig_h)

    def _run_sync(self, rgb_frame: np.ndarray) -> dict:
        """Inferencia síncrona con el nuevo API de HAILO10H."""
        import threading

        # Crear buffers de salida
        out_buffers = {
            out.name: np.empty(out.shape, dtype=np.float32)
            for out in self._infer_model.outputs
        }
        binding = self._configured.create_bindings(output_buffers=out_buffers)
        binding.input().set_buffer(rgb_frame.astype(np.uint8))

        # Callback para capturar excepción
        exc = [None]
        done = threading.Event()

        def _cb(completion_info=None, **kwargs):
            if completion_info and completion_info.exception:
                exc[0] = completion_info.exception
            done.set()

        self._configured.wait_for_async_ready(timeout_ms=5000)
        job = self._configured.run_async([binding], _cb)
        job.wait(timeout_ms=5000)

        if exc[0]:
            raise exc[0]

        return {name: binding.output(name).get_buffer() for name in out_buffers}

    # ── SCRFD decode ─────────────────────────────────────────────────────────

    def _decode(self, raw: dict, orig_w: int, orig_h: int) -> list:
        """
        Decode raw SCRFD outputs → lista de detecciones en píxeles originales.

        Formato de anchors: [cy_px, cx_px] — centros en píxeles del input 640×640.
        Decode SCRFD (offsets en unidades de stride):
          x1_640 = cx_px - l_raw * stride
          y1_640 = cy_px - t_raw * stride
          x2_640 = cx_px + r_raw * stride
          y2_640 = cy_px + b_raw * stride
        Luego escalar al frame original:
          x1 = x1_640 / 640 * orig_w
        """
        all_boxes  = []
        all_scores = []
        all_lms    = []

        cfg = self._tensor_cfg
        for i, stride in enumerate(_SCRFD_STRIDES):
            cls_key  = cfg['cls'][i]
            bbox_key = cfg['bbox'][i]
            lm_key   = cfg['lm'][i]

            if cls_key not in raw or bbox_key not in raw or lm_key not in raw:
                continue

            # Tensores raw: float32, shape [H, W, C]
            cls_raw  = raw[cls_key]    # [feat_h, feat_w, 2]   — 1 score × 2 anchors
            bbox_raw = raw[bbox_key]   # [feat_h, feat_w, 8]   — 4 offsets × 2 anchors
            lm_raw   = raw[lm_key]     # [feat_h, feat_w, 20]  — 10 coords × 2 anchors

            # Sigmoid en cls si los valores están fuera de [0,1]
            if cls_raw.max() > 1.0 or cls_raw.min() < 0.0:
                cls_raw = 1.0 / (1.0 + np.exp(-cls_raw))

            # Reshape a [N_anchors, values_per_anchor]
            # Layout: para cada celda, los 2 anchors son consecutivos en el eje C
            # cls: [H,W,2]  → reshape(-1) da [anchor0_cell0, anchor1_cell0, anchor0_cell1, ...]
            n = cls_raw.shape[0] * cls_raw.shape[1] * _SCRFD_NUM_ANCHORS
            scores = cls_raw.reshape(n)           # [N]
            boxes  = bbox_raw.reshape(n, 4)       # [N, 4] — ltrb en unidades de stride
            lms    = lm_raw.reshape(n, 10)        # [N, 10] — 5 pts × (dy,dx) en stride units

            anchors = self._anchors[stride]       # [N, 2] — [cy_px, cx_px]

            # Filtrar por threshold
            mask = scores > self._score_thr
            if not np.any(mask):
                continue

            scores = scores[mask]
            boxes  = boxes[mask]
            lms    = lms[mask]
            anc    = anchors[mask]  # [M, 2]

            cy_px = anc[:, 0]  # centro y en píxeles del input 640×640
            cx_px = anc[:, 1]  # centro x en píxeles del input 640×640

            # Decode boxes en espacio 640×640 (offsets en unidades de stride)
            # boxes[:, 0] = l (left),  boxes[:, 1] = t (top)
            # boxes[:, 2] = r (right), boxes[:, 3] = b (bottom)
            x1_640 = cx_px - boxes[:, 0] * stride
            y1_640 = cy_px - boxes[:, 1] * stride
            x2_640 = cx_px + boxes[:, 2] * stride
            y2_640 = cy_px + boxes[:, 3] * stride
            boxes_dec = np.stack([x1_640, y1_640, x2_640, y2_640], axis=1)  # [x1,y1,x2,y2] en 640px

            # Decode landmarks en espacio 640×640
            # lms layout: [dy0,dx0, dy1,dx1, ..., dy4,dx4]
            cy_t = np.tile(cy_px[:, None], (1, 5))
            cx_t = np.tile(cx_px[:, None], (1, 5))
            lm_dy = lms[:, 0::2]  # [N, 5] — offsets y en stride units
            lm_dx = lms[:, 1::2]  # [N, 5] — offsets x en stride units
            lm_y_640 = cy_t + lm_dy * stride  # en píxeles 640px
            lm_x_640 = cx_t + lm_dx * stride
            lms_dec = np.zeros_like(lms)
            lms_dec[:, 0::2] = lm_y_640
            lms_dec[:, 1::2] = lm_x_640

            all_boxes.append(boxes_dec)
            all_scores.append(scores)
            all_lms.append(lms_dec)

        if not all_boxes:
            return []

        all_boxes  = np.concatenate(all_boxes,  axis=0)  # [N, 4] en 640px space [x1,y1,x2,y2]
        all_scores = np.concatenate(all_scores, axis=0)  # [N]
        all_lms    = np.concatenate(all_lms,    axis=0)  # [N, 10] en 640px space

        # Escalar al frame original
        sx = orig_w / _SCRFD_INPUT_SIZE
        sy = orig_h / _SCRFD_INPUT_SIZE
        x1_px = (all_boxes[:, 0] * sx).astype(np.float32)
        y1_px = (all_boxes[:, 1] * sy).astype(np.float32)
        x2_px = (all_boxes[:, 2] * sx).astype(np.float32)
        y2_px = (all_boxes[:, 3] * sy).astype(np.float32)

        # NMS con OpenCV (necesita [x, y, w, h] en píxeles)
        cv_boxes = np.stack([x1_px, y1_px, x2_px - x1_px, y2_px - y1_px], axis=1)

        indices = cv2.dnn.NMSBoxes(
            cv_boxes.tolist(), all_scores.tolist(),
            self._score_thr, self._iou_thr)

        detections = []
        if len(indices) == 0:
            return detections
        for idx in np.array(indices).flatten():
            x1 = int(np.clip(x1_px[idx], 0, orig_w - 1))
            y1 = int(np.clip(y1_px[idx], 0, orig_h - 1))
            x2 = int(np.clip(x2_px[idx], 0, orig_w - 1))
            y2 = int(np.clip(y2_px[idx], 0, orig_h - 1))
            if (x2 - x1) < 15 or (y2 - y1) < 15:
                continue
            # Landmarks en píxeles del frame original: [y0,x0, y1,x1, ...] → [(x,y)×5]
            lm_row = all_lms[idx]
            lms_px = []
            for j in range(5):
                ly = int(np.clip(lm_row[j * 2]     * sy, 0, orig_h - 1))
                lx = int(np.clip(lm_row[j * 2 + 1] * sx, 0, orig_w - 1))
                lms_px.append((lx, ly))
            detections.append({
                'bbox': [x1, y1, x2, y2],
                'score': float(all_scores[idx]),
                'landmarks': lms_px,
            })
        return detections

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def _cleanup(self):
        try:
            if self._config_ctx is not None:
                self._config_ctx.__exit__(None, None, None)
        except Exception:
            pass
        self._ready = False

    def release(self):
        self._cleanup()
        self._vdevice = None


# ─── Nodo ROS2 ───────────────────────────────────────────────────────────────

class HailoFaceGestureNode(Node):
    """Face detection (Hailo SCRFD) + Hand gesture (MediaPipe) → Robot control."""

    def __init__(self):
        super().__init__('hailo_face_gesture')

        # ─── Parámetros ───
        self.declare_parameter('image_topic', '/kinect/rgb/image_raw')
        self.declare_parameter('face_model', 'scrfd_10g.hef')
        self.declare_parameter('face_score_threshold', 0.55)
        self.declare_parameter('lbph_confidence', 130.0)  # medido en vivo: jose ~102-110
        self.declare_parameter('require_auth', True)
        self.declare_parameter('auth_timeout', 30.0)
        self.declare_parameter('gesture_hold_time', 0.7)
        self.declare_parameter('mode_change_hold_time', 2.0)
        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 0.50)
        self.declare_parameter('publish_image', True)
        self.declare_parameter('process_rate', 10.0)
        self.declare_parameter('max_num_hands', 1)
        self.declare_parameter('hand_confidence', 0.65)
        self.declare_parameter('enabled', True)

        p = self.get_parameter
        self._image_topic     = p('image_topic').value
        self._face_model_name = p('face_model').value
        self._face_score_thr  = p('face_score_threshold').value
        self._lbph_conf       = p('lbph_confidence').value
        self._require_auth    = p('require_auth').value
        self._auth_timeout    = p('auth_timeout').value
        self._gesture_hold    = p('gesture_hold_time').value
        self._mode_hold       = p('mode_change_hold_time').value
        self._lin_speed       = p('linear_speed').value
        self._ang_speed       = p('angular_speed').value
        self._pub_image       = p('publish_image').value
        self._process_rate    = p('process_rate').value
        self._max_hands       = p('max_num_hands').value
        self._hand_conf       = p('hand_confidence').value
        self._enabled         = p('enabled').value

        # ─── Resolver ruta del modelo HEF ───
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory('smart_t_ai_v2')
            hef_path = os.path.join(pkg_dir, 'models', self._face_model_name)
        except Exception:
            hef_path = os.path.join(
                os.path.dirname(__file__), '..', 'models', self._face_model_name)
        hef_path = os.path.normpath(hef_path)

        # ─── Inicializar Hailo Face Detector ───
        self._hailo = HailoFaceDetector(
            hef_path,
            score_thr=self._face_score_thr,
            logger=self.get_logger()
        )
        if not self._hailo.ready:
            self.get_logger().warn('⚠️  Usando Haar cascade como fallback de detección de caras')
            self._face_cascade = cv2.CascadeClassifier(HAAR_CASCADE)
        else:
            self._face_cascade = None

        # ─── LBPH Face Recognizer ───
        self._recognizer = None
        self._label_map = {}
        self._load_lbph_model()

        # ─── MediaPipe Hands ───
        self._mp_hands = mp.solutions.hands
        self._hands = self._mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=self._max_hands,
            min_detection_confidence=self._hand_conf,
            min_tracking_confidence=0.5,
        )
        self._mp_draw = mp.solutions.drawing_utils
        self._mp_styles = mp.solutions.drawing_styles

        # ─── Estado ───
        self.state         = STATE_DISABLED if not self._enabled else STATE_WAITING
        self._auth_user    = ''
        self._auth_time    = 0.0
        self._robot_mode   = 'IDLE'
        # Gesto actual y confirmado
        self._cur_gesture  = -1
        self._gest_start   = 0.0
        self._conf_gesture = -1
        self._last_cmd_t   = 0.0
        # Modo-change tracking
        self._mode_cand    = None
        self._mode_cand_t  = 0.0

        # ─── Buffer de imagen ───
        self._lock = threading.Lock()
        self._latest_img: Image = None

        # ─── Stats ───
        self._frames = 0
        self._stats_t = time.time()

        # ─── Publishers ───
        self._faces_pub  = self.create_publisher(String, '/hailo/faces',       10)
        self._gest_pub   = self.create_publisher(String, '/hailo/gesture_cmd', 10)
        self._cmdvel_pub = self.create_publisher(Twist,  '/hailo/cmd_vel',     10)
        self._status_pub = self.create_publisher(String, '/hailo/status',      10)
        if self._pub_image:
            self._img_pub = self.create_publisher(Image, '/hailo/annotated', 5)

        # ─── Subscribers ───
        self.create_subscription(Image,  self._image_topic,  self._img_cb,    1)
        self.create_subscription(String, '/robot/mode',      self._mode_cb,   10)

        # ─── Timers ───
        period = 1.0 / self._process_rate
        self.create_timer(period, self._process_cb)
        self.create_timer(0.4,    self._safety_cb)
        self.create_timer(15.0,   self._stats_cb)

        self.get_logger().info(
            f'🤖 Hailo Face+Gesture Node arriba | '
            f'Hailo={"✅" if self._hailo.ready else "❌(Haar)"} | '
            f'MediaPipe=✅ | '
            f'state={self.state}')

    # ══════════════════════════════════════════════
    #  Model loading
    # ══════════════════════════════════════════════

    def _load_lbph_model(self):
        if os.path.exists(LBPH_MODEL) and os.path.exists(LABEL_MAP):
            try:
                r = cv2.face.LBPHFaceRecognizer_create()
                r.read(LBPH_MODEL)
                self._recognizer = r
                with open(LABEL_MAP) as f:
                    raw = json.load(f)
                    self._label_map = {int(k): v for k, v in raw.items()}
                self.get_logger().info(
                    f'✅ LBPH cargado: {list(self._label_map.values())}')
            except Exception as e:
                self.get_logger().error(f'Error LBPH: {e}')
        else:
            self.get_logger().warn(
                f'Modelo LBPH no encontrado ({LBPH_MODEL}). '
                'Usa register_face.py para entrenar.')

    # ══════════════════════════════════════════════
    #  Callbacks
    # ══════════════════════════════════════════════

    def _img_cb(self, msg: Image):
        with self._lock:
            self._latest_img = msg

    def _mode_cb(self, msg: String):
        self._robot_mode = msg.data

    # ══════════════════════════════════════════════
    #  Main processing pipeline
    # ══════════════════════════════════════════════

    def _process_cb(self):
        if self.state == STATE_DISABLED:
            self._publish_status(-1, 'NONE')
            return

        with self._lock:
            if self._latest_img is None:
                return
            img_msg = self._latest_img
            self._latest_img = None

        self._frames += 1

        try:
            # ─── Decodificar imagen ───
            h, w = img_msg.height, img_msg.width
            enc = img_msg.encoding.lower()
            if enc in ('rgb8',):
                img_rgb = np.frombuffer(img_msg.data, np.uint8).reshape(h, w, 3)
                img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            elif enc in ('bgr8',):
                img_bgr = np.frombuffer(img_msg.data, np.uint8).reshape(h, w, 3)
                img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            else:
                return

            display = img_rgb.copy()  # Para publicar (RGB)
            now = time.time()

            # ─── Paso 1: Detección + reconocimiento de cara ───
            face_name, face_bbox = self._detect_and_recognize(img_bgr, img_rgb, display)

            # ─── Paso 2: Máquina de estados de autorización ───
            self._update_auth_state(face_name, now)

            # ─── Paso 3: Detección de gestos (MediaPipe) ───
            gesture = -1
            if self.state in (STATE_AUTHORIZED, STATE_EXECUTING, STATE_MODE_CHANGE):
                gesture = self._detect_gesture_mediapipe(img_rgb, display, now)
                self._update_gesture_state(gesture, now)

            # ─── Paso 4: Ejecutar comando si gesto confirmado ───
            if self.state == STATE_EXECUTING and self._conf_gesture >= 0:
                self._execute_gesture(self._conf_gesture)
                self._last_cmd_t = now

            # ─── HUD ───
            self._draw_hud(display, gesture, now)

            # ─── Publicar imagen anotada ───
            if self._pub_image:
                out = Image()
                out.header = img_msg.header
                out.height = h
                out.width  = w
                out.encoding = 'rgb8'
                out.step = w * 3
                out.data = display.tobytes()
                self._img_pub.publish(out)

            # ─── Publicar estado ───
            self._publish_status(gesture, GESTURE_NAMES.get(self._conf_gesture, 'NONE'))

        except Exception as e:
            self.get_logger().error(f'Pipeline error: {e}', throttle_duration_sec=5.0)

    # ══════════════════════════════════════════════
    #  Face detection
    # ══════════════════════════════════════════════

    def _detect_and_recognize(self, bgr, rgb, display):
        """Detecta cara con Hailo (o Haar) y reconoce con LBPH."""
        h, w = bgr.shape[:2]

        if self._hailo.ready:
            dets = self._hailo.detect(bgr)
            faces = [(d['bbox'][0], d['bbox'][1],
                      d['bbox'][2] - d['bbox'][0],
                      d['bbox'][3] - d['bbox'][1]) for d in dets]
        else:
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            gray_eq = cv2.equalizeHist(gray)
            min_dim = max(60, int(w * 0.1))
            rects = self._face_cascade.detectMultiScale(
                gray_eq, scaleFactor=1.15, minNeighbors=5,
                minSize=(min_dim, min_dim))
            faces = list(rects) if len(rects) > 0 else []

        best_name = None
        best_rect = None
        best_conf = 9999

        gray_full = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        face_list_json = []
        for (fx, fy, fw, fh) in faces:
            x1, y1, x2, y2 = fx, fy, fx + fw, fy + fh
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            fw, fh = x2 - x1, y2 - y1
            if fw < 20 or fh < 20:
                continue

            name = None
            conf = 9999

            if self._recognizer is not None:
                roi = gray_full[y1:y2, x1:x2]
                roi_resized = cv2.resize(roi, (200, 200))
                label, conf = self._recognizer.predict(roi_resized)
                if conf < self._lbph_conf:
                    name = self._label_map.get(label, f'id_{label}')

            color = C_GREEN if name else C_RED
            cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
            if name:
                cv2.putText(display, f'{name} ({conf:.0f})',
                            (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
                if conf < best_conf:
                    best_name = name
                    best_conf = conf
                    best_rect = (x1, y1, x2, y2)
            else:
                cv2.putText(display, '?', (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_YELLOW, 2)

            face_list_json.append({'bbox': [x1, y1, x2, y2], 'name': name or ''})

        # Publicar detecciones
        fmsg = String()
        fmsg.data = json.dumps({'faces': face_list_json})
        self._faces_pub.publish(fmsg)

        return best_name, best_rect

    # ══════════════════════════════════════════════
    #  Authorization state machine
    # ══════════════════════════════════════════════

    def _update_auth_state(self, face_name, now):
        if self.state == STATE_WAITING:
            if face_name:
                self.state = STATE_AUTHORIZED
                self._auth_user = face_name
                self._auth_time = now
                self.get_logger().info(f'👤 Autorizado: {face_name}')
            elif not self._require_auth:
                self.state = STATE_AUTHORIZED
                self._auth_user = 'anónimo'
                self._auth_time = now

        elif self.state in (STATE_AUTHORIZED, STATE_EXECUTING, STATE_MODE_CHANGE):
            if self._require_auth:
                if (now - self._auth_time) > self._auth_timeout:
                    if not face_name:
                        self.get_logger().info('⏰ Timeout auth — volviendo a WAITING')
                        self._reset_state()
                        self._stop_robot()
                        return
            if face_name:
                self._auth_time = now

    # ══════════════════════════════════════════════
    #  Gesture detection (MediaPipe)
    # ══════════════════════════════════════════════

    def _detect_gesture_mediapipe(self, rgb, display, now) -> int:
        """Detecta gesto con MediaPipe Hands. Retorna número de dedos (0-7) o -1."""
        results = self._hands.process(rgb)
        if not results.multi_hand_landmarks:
            return -1

        h, w = rgb.shape[:2]
        gesture = -1

        for hand_lms, hand_info in zip(
                results.multi_hand_landmarks,
                results.multi_handedness):

            # Dibujar landmarks
            self._mp_draw.draw_landmarks(
                display, hand_lms,
                self._mp_hands.HAND_CONNECTIONS,
                self._mp_styles.get_default_hand_landmarks_style(),
                self._mp_styles.get_default_hand_connections_style(),
            )

            handedness = hand_info.classification[0].label  # 'Left' o 'Right'
            finger_count = self._count_fingers(hand_lms, handedness)

            # Detectar gestos especiales
            special = self._detect_special_gesture(hand_lms, handedness)
            if special >= 0:
                gesture = special
            else:
                gesture = finger_count

            # Anotar
            lms = hand_lms.landmark
            cx = int(lms[9].x * w)
            cy = int(lms[9].y * h) - 30
            gname = GESTURE_NAMES.get(gesture, str(gesture))
            cv2.putText(display, f'{gesture} - {gname}',
                        (max(5, cx - 50), max(20, cy)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, C_YELLOW, 2)

        return gesture

    def _count_fingers(self, hand_lms, handedness: str) -> int:
        """Cuenta dedos extendidos usando landmarks de MediaPipe."""
        lms = hand_lms.landmark
        count = 0

        # Puntas y articulaciones PIP de los 4 dedos (index, middle, ring, pinky)
        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        for tip, pip in zip(tips, pips):
            if lms[tip].y < lms[pip].y:   # punta más arriba = extendido
                count += 1

        # Pulgar: comparar x según mano
        # Para mano derecha (imagen espejada de cámara): pulgar extendido si tip.x < ip.x
        # Para mano izquierda: pulgar extendido si tip.x > ip.x
        thumb_tip_x = lms[4].x
        thumb_ip_x  = lms[3].x
        if handedness == 'Right':
            if thumb_tip_x < thumb_ip_x:
                count += 1
        else:
            if thumb_tip_x > thumb_ip_x:
                count += 1

        return min(count, 5)

    def _detect_special_gesture(self, hand_lms, handedness: str) -> int:
        """
        Detecta gestos especiales que no son conteo de dedos.
        Retorna:
          6 = THUMB_UP (solo pulgar arriba, otros cerrados)
          7 = THUMB_DOWN (solo pulgar abajo, otros cerrados)
         -1 = no especial
        """
        lms = hand_lms.landmark

        # Verificar que index,middle,ring,pinky están cerrados
        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        fingers_closed = all(lms[t].y > lms[p].y for t, p in zip(tips, pips))

        if not fingers_closed:
            return -1

        # Pulgar arriba: tip.y significativamente menor que mcp.y
        thumb_tip_y = lms[4].y
        thumb_mcp_y = lms[2].y
        threshold = 0.08

        if thumb_tip_y < (thumb_mcp_y - threshold):
            return 6  # THUMB_UP
        if thumb_tip_y > (thumb_mcp_y + threshold):
            return 7  # THUMB_DOWN

        return -1

    # ══════════════════════════════════════════════
    #  Gesture state machine
    # ══════════════════════════════════════════════

    def _update_gesture_state(self, gesture: int, now: float):
        """Confirma gestos con debounce temporal y maneja cambios de modo."""
        if gesture < 0:
            self._cur_gesture = -1
            self._gest_start  = now
            self._mode_cand   = None
            return

        # ── Gestos especiales → cambio de modo ──
        if gesture == 6:   # THUMB_UP → activar GESTURE mode (si no lo está)
            if self._mode_cand != 'MODE_GESTURE':
                self._mode_cand   = 'MODE_GESTURE'
                self._mode_cand_t = now
            elif (now - self._mode_cand_t) >= self._mode_hold:
                self._request_mode_change('GESTURE')
                self._mode_cand = None
            return

        if gesture == 7:   # THUMB_DOWN → volver a IDLE
            if self._mode_cand != 'MODE_IDLE':
                self._mode_cand   = 'MODE_IDLE'
                self._mode_cand_t = now
            elif (now - self._mode_cand_t) >= self._mode_hold:
                self._request_mode_change('IDLE')
                self._mode_cand = None
            return

        # ── Palma abierta (5) mantenida → FOLLOW mode ──
        if gesture == 5 and self._robot_mode != 'FOLLOW':
            if self._mode_cand != 'MODE_FOLLOW':
                self._mode_cand   = 'MODE_FOLLOW'
                self._mode_cand_t = now
            elif (now - self._mode_cand_t) >= self._mode_hold:
                self._request_mode_change('FOLLOW')
                self._mode_cand = None
                return
        else:
            if self._mode_cand in ('MODE_FOLLOW',):
                self._mode_cand = None

        # ── Gesto normal → confirmación temporal ──
        if gesture == self._cur_gesture:
            elapsed = now - self._gest_start
            if elapsed >= self._gesture_hold:
                if gesture != self._conf_gesture:
                    self._conf_gesture = gesture
                    self.state = STATE_EXECUTING
                    self.get_logger().info(
                        f'🤚 Gesto confirmado: {GESTURE_NAMES.get(gesture)} '
                        f'({gesture} dedos)')
        else:
            self._cur_gesture = gesture
            self._gest_start  = now

    def _request_mode_change(self, new_mode: str):
        """Publica solicitud de cambio de modo al mode_manager."""
        self.get_logger().info(f'✋ Gesto → solicitud modo: {new_mode}')
        msg = String()
        msg.data = json.dumps({'cmd': 'set_mode', 'mode': new_mode, 'source': 'gesture'})
        self._gest_pub.publish(msg)

    # ══════════════════════════════════════════════
    #  Command execution
    # ══════════════════════════════════════════════

    def _execute_gesture(self, gesture: int):
        """Traduce gesto confirmado a cmd_vel y al topic de comando."""
        twist = Twist()
        lin, ang = self._lin_speed, self._ang_speed

        if   gesture == 0:  pass                              # STOP
        elif gesture == 1:  twist.linear.x = lin              # ADELANTE
        elif gesture == 2:  twist.linear.x = -lin             # ATRAS
        elif gesture == 3:  twist.angular.z = ang             # IZQUIERDA
        elif gesture == 4:  twist.angular.z = -ang            # DERECHA
        elif gesture == 5:  twist.linear.x = lin * 0.6       # SIGUEME (lento)

        self._cmdvel_pub.publish(twist)

        cmd = GESTURE_CMDS.get(gesture, 'UNKNOWN')
        gmsg = String()
        gmsg.data = json.dumps({
            'cmd': 'move',
            'gesture': gesture,
            'gesture_name': GESTURE_NAMES.get(gesture, ''),
            'direction': cmd,
        })
        self._gest_pub.publish(gmsg)

    def _stop_robot(self):
        self._cmdvel_pub.publish(Twist())

    def _reset_state(self):
        self.state          = STATE_WAITING
        self._auth_user     = ''
        self._auth_time     = 0.0
        self._cur_gesture   = -1
        self._gest_start    = 0.0
        self._conf_gesture  = -1
        self._mode_cand     = None

    # ══════════════════════════════════════════════
    #  Safety & stats timers
    # ══════════════════════════════════════════════

    def _safety_cb(self):
        """Para el robot si no hay gestos recientes en EXECUTING."""
        if self.state == STATE_EXECUTING:
            if time.time() - self._last_cmd_t > 1.2:
                self._stop_robot()
                self._conf_gesture = -1
                self.state = STATE_AUTHORIZED

    def _stats_cb(self):
        now = time.time()
        fps = self._frames / max(now - self._stats_t, 1e-3)
        self.get_logger().info(
            f'👁️ Face+Gesture: {fps:.1f} FPS | state={self.state} | '
            f'user={self._auth_user} | mode={self._robot_mode}')
        self._frames  = 0
        self._stats_t = now

    # ══════════════════════════════════════════════
    #  HUD
    # ══════════════════════════════════════════════

    def _draw_hud(self, display, raw_gesture, now):
        h, w = display.shape[:2]

        # Barra superior semitransparente
        overlay = display.copy()
        cv2.rectangle(overlay, (0, 0), (w, 75), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, display, 0.45, 0, display)

        state_colors = {
            STATE_DISABLED:   (100, 100, 100),
            STATE_WAITING:    C_RED,
            STATE_AUTHORIZED: C_GREEN,
            STATE_EXECUTING:  C_CYAN,
            STATE_MODE_CHANGE: C_PURPLE,
        }
        sc = state_colors.get(self.state, C_WHITE)
        cv2.putText(display, f'Estado: {self.state}',
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, sc, 2)
        cv2.putText(display, f'Modo: {self._robot_mode}',
                    (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_YELLOW, 1)
        if self._auth_user:
            remaining = max(0.0, self._auth_timeout - (now - self._auth_time))
            cv2.putText(display, f'Usuario: {self._auth_user} ({remaining:.0f}s)',
                        (10, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_GREEN, 1)

        if self._conf_gesture >= 0:
            gn = GESTURE_NAMES.get(self._conf_gesture, '')
            cv2.putText(display, f'CMD: {gn}',
                        (w - 210, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, C_CYAN, 2)

        # Barra inferior
        if self.state == STATE_WAITING:
            txt = 'Muestra tu rostro para autorizar'
        elif self.state == STATE_AUTHORIZED:
            txt = 'Muestra gesto con la mano'
        elif self.state == STATE_EXECUTING:
            txt = 'Ejecutando gesto...'
        elif self.state == STATE_MODE_CHANGE:
            txt = f'Manteniendo... ({self._mode_cand})'
        else:
            txt = 'Nodo desactivado'

        cv2.putText(display, txt, (w // 2 - 160, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_WHITE, 1)

        # Leyenda gestos
        legend = ['0:STOP 1:FWD 2:BACK', '3:IZQ 4:DER 5:FOLLOW',
                  '👍:GESTURE MODE  👎:IDLE']
        for i, line in enumerate(legend):
            cv2.putText(display, line, (w - 230, h - 60 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, C_WHITE, 1)

    # ══════════════════════════════════════════════
    #  Status publisher
    # ══════════════════════════════════════════════

    def _publish_status(self, gesture, gesture_name):
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'user': self._auth_user,
            'gesture': gesture,
            'gesture_name': gesture_name,
            'robot_mode': self._robot_mode,
            'hailo': self._hailo.ready,
            'authorized': self.state in (STATE_AUTHORIZED, STATE_EXECUTING),
        })
        self._status_pub.publish(msg)

    def destroy_node(self):
        try:
            self._stop_robot()
            self._hailo.release()
            self._hands.close()
        except Exception:
            pass
        super().destroy_node()


# ─── main ─────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = HailoFaceGestureNode()
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
