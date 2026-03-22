#!/usr/bin/env python3
"""
Kinect Auto-Tracking: ajusta el motor tilt para mantener
la mano o cara centrada en el cuadro.

Modo:
  --target face   → sigue tu cara (Haar cascade)
  --target hand   → sigue tu mano (skin detection)
  --target both   → cara primero, si no mano

Uso:
  python3 kinect_track.py --target face
  python3 kinect_track.py --target hand
  python3 kinect_track.py --target both

El Kinect inclina arriba/abajo automáticamente.
Ctrl+C para salir.

Requiere: libfreenect, opencv
"""

import os
import sys
import time
import argparse
import ctypes
import cv2
import numpy as np

# ═══════════════════════════════════════════════════════════
#  libfreenect bindings
# ═══════════════════════════════════════════════════════════

# --- Sync API (video capture) ---
_sync = ctypes.CDLL('libfreenect_sync.so')
_sync.freenect_sync_get_video.argtypes = [
    ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_uint32),
    ctypes.c_int, ctypes.c_int,
]
_sync.freenect_sync_get_video.restype = ctypes.c_int
_sync.freenect_sync_stop.argtypes = []
_sync.freenect_sync_stop.restype = None

FREENECT_VIDEO_RGB = 0
WIDTH, HEIGHT = 640, 480

# --- Direct API (motor control) ---
_lib = ctypes.CDLL('libfreenect.so')


class _freenect_context(ctypes.Structure):
    pass


class _freenect_device(ctypes.Structure):
    pass


_ctx_p = ctypes.POINTER(_freenect_context)
_dev_p = ctypes.POINTER(_freenect_device)

_lib.freenect_init.argtypes = [ctypes.POINTER(_ctx_p), ctypes.c_void_p]
_lib.freenect_init.restype = ctypes.c_int
_lib.freenect_select_subdevices.argtypes = [_ctx_p, ctypes.c_int]
_lib.freenect_select_subdevices.restype = None
_lib.freenect_open_device.argtypes = [_ctx_p, ctypes.POINTER(_dev_p), ctypes.c_int]
_lib.freenect_open_device.restype = ctypes.c_int
_lib.freenect_set_tilt_degs.argtypes = [_dev_p, ctypes.c_double]
_lib.freenect_set_tilt_degs.restype = ctypes.c_int
_lib.freenect_update_tilt_state.argtypes = [_dev_p]
_lib.freenect_update_tilt_state.restype = ctypes.c_int
_lib.freenect_get_tilt_state.argtypes = [_dev_p]
_lib.freenect_get_tilt_state.restype = ctypes.c_void_p
_lib.freenect_get_tilt_degs.argtypes = [ctypes.c_void_p]
_lib.freenect_get_tilt_degs.restype = ctypes.c_double
_lib.freenect_set_led.argtypes = [_dev_p, ctypes.c_int]
_lib.freenect_set_led.restype = ctypes.c_int
_lib.freenect_close_device.argtypes = [_dev_p]
_lib.freenect_close_device.restype = ctypes.c_int
_lib.freenect_shutdown.argtypes = [_ctx_p]
_lib.freenect_shutdown.restype = ctypes.c_int

# LED values
LED_OFF = 0
LED_GREEN = 1
LED_RED = 2
LED_YELLOW = 3
LED_BLINK_GREEN = 4
LED_BLINK_RED_YELLOW = 6

# Tilt limits (Kinect v1 hardware)
TILT_MIN = -27.0
TILT_MAX = 27.0

# Haar cascade
HAAR_CASCADE = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'


def grab_frame(device_index=0):
    """Captura un frame RGB via sync API."""
    vp = ctypes.c_void_p()
    ts = ctypes.c_uint32()
    ret = _sync.freenect_sync_get_video(
        ctypes.byref(vp), ctypes.byref(ts), device_index, FREENECT_VIDEO_RGB)
    if ret != 0 or not vp.value:
        return None
    buf = (ctypes.c_uint8 * (WIDTH * HEIGHT * 3)).from_address(vp.value)
    return np.frombuffer(buf, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3).copy()


class KinectMotor:
    """Control del motor tilt del Kinect via libfreenect directo."""

    def __init__(self, device_index=0):
        self.ctx = _ctx_p()
        self.dev = _dev_p()
        self.current_tilt = 0.0
        self._target_tilt = 0.0
        self._last_move_time = 0.0

        ret = _lib.freenect_init(ctypes.byref(self.ctx), None)
        if ret != 0:
            raise RuntimeError(f'freenect_init failed: {ret}')

        # Solo subdevice MOTOR (0x01)
        _lib.freenect_select_subdevices(self.ctx, 0x01)

        ret = _lib.freenect_open_device(
            self.ctx, ctypes.byref(self.dev), device_index)
        if ret != 0:
            _lib.freenect_shutdown(self.ctx)
            raise RuntimeError(f'freenect_open_device failed: {ret}')

        # Leer ángulo actual
        self._update_tilt()
        print(f'   Motor inicializado, tilt actual: {self.current_tilt:.1f}°')

    def _update_tilt(self):
        """Lee el ángulo actual del motor."""
        _lib.freenect_update_tilt_state(self.dev)
        ts = _lib.freenect_get_tilt_state(self.dev)
        if ts:
            self.current_tilt = _lib.freenect_get_tilt_degs(ts)

    def set_tilt(self, angle_deg):
        """Mueve el motor al ángulo dado (clamped a [-27, 27])."""
        angle_deg = max(TILT_MIN, min(TILT_MAX, angle_deg))

        # No mover si la diferencia es menor a 1° (evita vibración)
        if abs(angle_deg - self._target_tilt) < 1.0:
            return

        # No mover más de 1 vez por 0.3s (el motor es lento)
        now = time.time()
        if (now - self._last_move_time) < 0.3:
            return

        self._target_tilt = angle_deg
        _lib.freenect_set_tilt_degs(self.dev, angle_deg)
        self._last_move_time = now

    def set_led(self, led):
        _lib.freenect_set_led(self.dev, led)

    def close(self):
        try:
            _lib.freenect_set_tilt_degs(self.dev, 0.0)
            time.sleep(0.5)
            _lib.freenect_set_led(self.dev, LED_OFF)
            _lib.freenect_close_device(self.dev)
            _lib.freenect_shutdown(self.ctx)
        except Exception:
            pass


def detect_face(gray_eq, face_cascade):
    """Detecta la cara más grande. Retorna (cx, cy, w, h) o None."""
    faces = face_cascade.detectMultiScale(
        gray_eq, scaleFactor=1.15, minNeighbors=3, minSize=(60, 60))
    if len(faces) == 0:
        return None
    # La más grande
    biggest = max(faces, key=lambda f: f[2] * f[3])
    x, y, w, h = biggest
    cx = x + w // 2
    cy = y + h // 2
    return cx, cy, w, h


def detect_hand(img_bgr):
    """Detecta la mano (skin blob más grande). Retorna (cx, cy, w, h) o None."""
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # Rango de piel en HSV
    lower1 = np.array([0, 30, 60], dtype=np.uint8)
    upper1 = np.array([20, 150, 255], dtype=np.uint8)
    mask1 = cv2.inRange(hsv, lower1, upper1)

    lower2 = np.array([170, 30, 60], dtype=np.uint8)
    upper2 = np.array([180, 150, 255], dtype=np.uint8)
    mask2 = cv2.inRange(hsv, lower2, upper2)

    skin = mask1 | mask2

    # Limpiar
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    skin = cv2.morphologyEx(skin, cv2.MORPH_CLOSE, kernel, iterations=2)
    skin = cv2.morphologyEx(skin, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(skin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # Mayor contorno
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)

    # Filtrar: mano debe ser entre 2% y 40% de la imagen
    img_area = HEIGHT * WIDTH
    if area < img_area * 0.02 or area > img_area * 0.40:
        return None

    x, y, w, h = cv2.boundingRect(c)
    cx = x + w // 2
    cy = y + h // 2
    return cx, cy, w, h


def main():
    parser = argparse.ArgumentParser(description='Kinect Auto-Tracking')
    parser.add_argument('--target', '-t', choices=['face', 'hand', 'both'],
                        default='face', help='Qué seguir (default: face)')
    parser.add_argument('--speed', '-s', type=float, default=0.15,
                        help='Velocidad de seguimiento 0.05-0.5 (default: 0.15)')
    parser.add_argument('--deadzone', '-d', type=float, default=0.15,
                        help='Zona muerta centro 0.0-0.4 (default: 0.15)')
    args = parser.parse_args()

    speed = max(0.05, min(0.5, args.speed))
    deadzone = max(0.0, min(0.4, args.deadzone))

    print(f'╔══════════════════════════════════════════╗')
    print(f'║   Kinect Auto-Tracking                   ║')
    print(f'║   Target: {args.target:<8s}                      ║')
    print(f'║   Speed:  {speed:.2f}   Deadzone: {deadzone:.2f}       ║')
    print(f'║   Ctrl+C para salir                      ║')
    print(f'╚══════════════════════════════════════════╝')

    # Inicializar Haar cascade
    face_cascade = None
    if args.target in ('face', 'both'):
        face_cascade = cv2.CascadeClassifier(HAAR_CASCADE)
        if face_cascade.empty():
            print(f'ERROR: No se pudo cargar {HAAR_CASCADE}')
            return

    # IMPORTANTE: inicializar video sync PRIMERO, luego motor directo
    # La sync API reclama camera, la direct API reclama motor por separado
    print('\n📷 Inicializando Kinect...')

    # 1) Video primero via sync API
    test = grab_frame()
    if test is None:
        print('ERROR: No se pudo capturar frame del Kinect')
        return
    print(f'   Cámara OK: {test.shape}')

    # 2) Motor después via direct API
    motor = KinectMotor()
    motor.set_led(LED_GREEN)

    # Leer tilt actual
    motor._update_tilt()
    current_tilt = motor.current_tilt
    print(f'   Tilt inicial: {current_tilt:.1f}°')
    print(f'\n🎯 Tracking activo — mueve tu {"cara" if args.target == "face" else "mano"}\n')

    frame_count = 0
    track_count = 0
    no_target_count = 0
    start_time = time.time()
    last_print = 0

    try:
        while True:
            frame_rgb = grab_frame()
            if frame_rgb is None:
                time.sleep(0.05)
                continue

            frame_count += 1
            img_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
            gray_eq = cv2.equalizeHist(gray)

            target = None
            target_type = ''

            # Detección según modo
            if args.target == 'face' and face_cascade is not None:
                target = detect_face(gray_eq, face_cascade)
                target_type = 'CARA'

            elif args.target == 'hand':
                target = detect_hand(img_bgr)
                target_type = 'MANO'

            elif args.target == 'both':
                # Primero cara, luego mano
                target = detect_face(gray_eq, face_cascade)
                target_type = 'CARA'
                if target is None:
                    target = detect_hand(img_bgr)
                    target_type = 'MANO'

            now = time.time()

            if target is not None:
                cx, cy, tw, th = target
                no_target_count = 0
                track_count += 1

                # Calcular offset vertical normalizado [-1, +1]
                # cy=0 → arriba, cy=HEIGHT → abajo
                # offset positivo = target está ABAJO del centro
                offset_y = (cy - HEIGHT / 2) / (HEIGHT / 2)

                # LED según posición
                if abs(offset_y) < deadzone:
                    motor.set_led(LED_GREEN)  # Centrado
                else:
                    motor.set_led(LED_YELLOW)  # Ajustando

                # Solo ajustar si está fuera de la zona muerta
                if abs(offset_y) > deadzone:
                    # Target abajo del centro → bajar tilt (restar)
                    # Target arriba del centro → subir tilt (sumar)
                    adjustment = -offset_y * speed * 5.0  # grados por ciclo

                    current_tilt += adjustment
                    current_tilt = max(TILT_MIN, min(TILT_MAX, current_tilt))
                    motor.set_tilt(current_tilt)

                # Log cada 2 segundos
                if (now - last_print) > 2.0:
                    motor._update_tilt()
                    real_tilt = motor.current_tilt
                    elapsed = now - start_time
                    fps = frame_count / elapsed if elapsed > 0 else 0
                    direction = '↑' if offset_y < -deadzone else ('↓' if offset_y > deadzone else '●')
                    print(f'  {target_type} {direction} '
                          f'pos=({cx},{cy}) offset={offset_y:+.2f} '
                          f'tilt={current_tilt:.1f}°(real={real_tilt:.1f}°) '
                          f'{fps:.0f}fps')
                    last_print = now

            else:
                no_target_count += 1

                # Si no detecta por mucho tiempo, parpadear LED
                if no_target_count > 30:
                    motor.set_led(LED_BLINK_GREEN)

                if (now - last_print) > 3.0:
                    elapsed = now - start_time
                    fps = frame_count / elapsed if elapsed > 0 else 0
                    print(f'  ... buscando {target_type or args.target} '
                          f'(tilt={current_tilt:.1f}°, {fps:.0f}fps)')
                    last_print = now

            time.sleep(0.03)  # ~30 FPS máx

    except KeyboardInterrupt:
        print('\n\n⏹  Detenido')

    finally:
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0
        print(f'   Frames: {frame_count}, Tracked: {track_count}, '
              f'FPS: {fps:.1f}, Tiempo: {elapsed:.0f}s')

        print('   Regresando tilt a 0°...')
        motor.set_tilt(0.0)
        # Force move since set_tilt has deadzone
        _lib.freenect_set_tilt_degs(motor.dev, 0.0)
        time.sleep(1.5)
        motor.close()
        _sync.freenect_sync_stop()
        print('   ✅ Kinect liberado')


if __name__ == '__main__':
    main()
