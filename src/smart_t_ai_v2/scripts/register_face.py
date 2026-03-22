#!/usr/bin/env python3
"""
Registro de rostros para Smart Trolley V5.

Captura imágenes DIRECTAMENTE del Kinect via libfreenect (sin ROS2).
Guarda muestras del rostro y entrena reconocedor LBPH con augmentación.

Uso:
  python3 register_face.py --name jose            # captura 30 muestras
  python3 register_face.py --name jose --reset    # borra datos viejos y recaptura
  python3 register_face.py --name jose --samples 50
  python3 register_face.py --list                 # listar usuarios + threshold sugerido
  python3 register_face.py --train                # re-entrenar con datos existentes
  python3 register_face.py --delete jose          # eliminar usuario
  python3 register_face.py --no-preview           # sin ventana gráfica (headless)

Datos en: ~/.smart_trolley/faces/<nombre>/
Modelo:   ~/.smart_trolley/faces/lbph_model.yml
"""

import os
import sys
import time
import json
import argparse
import shutil
from datetime import datetime

import cv2
import numpy as np
import ctypes

# --- Paths ---
HAAR_CASCADE = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
FACES_DIR = os.path.expanduser('~/.smart_trolley/faces')

# --- libfreenect ctypes (acceso directo al Kinect, sin ROS) ---
_sync = None
FREENECT_VIDEO_RGB = 0  # 640x480 RGB


def _init_freenect():
    """Initialize libfreenect_sync bindings."""
    global _sync
    if _sync is not None:
        return True
    try:
        _sync = ctypes.CDLL('libfreenect_sync.so')
        _sync.freenect_sync_get_video.argtypes = [
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.c_int, ctypes.c_int,
        ]
        _sync.freenect_sync_get_video.restype = ctypes.c_int
        _sync.freenect_sync_stop.argtypes = []
        _sync.freenect_sync_stop.restype = None
        return True
    except OSError:
        print('Error: libfreenect_sync.so no disponible.')
        print('   Instala: sudo apt install freenect')
        return False


def _blur_score(gray_roi) -> float:
    """Laplacian variance — valores bajos = imagen borrosa."""
    return cv2.Laplacian(gray_roi, cv2.CV_64F).var()


def _try_usb_reset():
    """Intenta resetear el USB del Kinect via ioctl si está BUSY."""
    import fcntl
    import glob
    USBDEVFS_RESET = 0x5514
    for path in glob.glob('/dev/bus/usb/00[0-9]/0[0-9][0-9]'):
        try:
            with open(path, 'wb') as f:
                # Leer descriptor para identificar el Kinect camera 045e:02ae
                pass
        except Exception:
            pass
    # Buscar por ID de producto conocido
    for bus_path in glob.glob('/sys/bus/usb/devices/*/idProduct'):
        try:
            with open(bus_path) as f:
                prod = f.read().strip()
            if prod == '02ae':  # Kinect camera
                dev_dir = os.path.dirname(bus_path)
                bus_num_path = os.path.join(dev_dir, 'busnum')
                dev_num_path = os.path.join(dev_dir, 'devnum')
                if os.path.exists(bus_num_path) and os.path.exists(dev_num_path):
                    with open(bus_num_path) as f:
                        bus = f.read().strip().zfill(3)
                    with open(dev_num_path) as f:
                        dev = f.read().strip().zfill(3)
                    dev_path = f'/dev/bus/usb/{bus}/{dev}'
                    if os.path.exists(dev_path):
                        with open(dev_path, 'wb') as f:
                            fcntl.ioctl(f, USBDEVFS_RESET, 0)
                        print(f'   USB reset: {dev_path}')
                        time.sleep(1.5)
                        return True
        except Exception:
            pass
    return False


def grab_kinect_frame(device_index=0):
    """Captura un frame RGB del Kinect via libfreenect sync."""
    video_ptr = ctypes.c_void_p()
    timestamp = ctypes.c_uint32()
    ret = _sync.freenect_sync_get_video(
        ctypes.byref(video_ptr), ctypes.byref(timestamp),
        device_index, FREENECT_VIDEO_RGB
    )
    if ret != 0:
        return None
    buf = (ctypes.c_uint8 * (640 * 480 * 3)).from_address(video_ptr.value)
    frame = np.frombuffer(buf, dtype=np.uint8).reshape(480, 640, 3).copy()
    return frame  # RGB


def capture_faces(user_name, num_samples=30, show_preview=True, reset=False):
    """Captura rostros directamente del Kinect via libfreenect.

    Args:
        user_name:    Nombre del usuario.
        num_samples:  Número de muestras a capturar.
        show_preview: Mostrar ventana con preview en tiempo real.
        reset:        Si True, borra fotos previas del usuario antes de capturar.
    """
    if not _init_freenect():
        return False

    face_cascade = cv2.CascadeClassifier(HAAR_CASCADE)
    if face_cascade.empty():
        print(f'Error: No se pudo cargar cascade: {HAAR_CASCADE}')
        return False

    user_dir = os.path.join(FACES_DIR, user_name)

    # ── Borrar datos viejos si --reset ────────────────────────────────────────
    if reset and os.path.isdir(user_dir):
        old_count = len([f for f in os.listdir(user_dir) if f.endswith('.jpg')])
        shutil.rmtree(user_dir)
        print(f'  RESET: {old_count} fotos viejas de "{user_name}" eliminadas.')

    os.makedirs(user_dir, exist_ok=True)
    existing = len([f for f in os.listdir(user_dir) if f.endswith('.jpg')])

    print(f'\n╔══════════════════════════════════════════════════╗')
    print(f'║  CAPTURA DE ROSTRO — {user_name:<27}║')
    print(f'╠══════════════════════════════════════════════════╣')
    print(f'║  Muestras existentes : {existing:<26}║')
    print(f'║  Muestras a capturar : {num_samples:<26}║')
    print(f'║  Preview             : {"sí" if show_preview else "no (headless)":<26}║')
    print(f'╠══════════════════════════════════════════════════╣')
    print(f'║  • Ponte frente al Kinect (~1 metro)             ║')
    print(f'║  • Mueve ligeramente la cabeza entre capturas    ║')
    print(f'║  • Buena iluminación (sin contraluz)             ║')
    print(f'║  • Auto-captura cada 0.6s cuando detecta 1 cara  ║')
    print(f'║  • Ctrl+C para interrumpir                       ║')
    print(f'╚══════════════════════════════════════════════════╝\n')

    # ── Conectar Kinect ───────────────────────────────────────────────────────
    print('   Conectando al Kinect...', end='', flush=True)
    test_frame = None
    for attempt in range(12):
        test_frame = grab_kinect_frame()
        if test_frame is not None:
            break
        if attempt == 3:
            print(' intentando reset USB...', end='', flush=True)
            _try_usb_reset()
        time.sleep(0.5)

    if test_frame is None:
        print('\n  ERROR: No se pudo obtener imagen del Kinect.')
        print('  → Desenchufa y vuelve a enchufar el Kinect, luego reintenta.')
        _sync.freenect_sync_stop()
        return False
    print(f' ✓ Kinect OK ({test_frame.shape[1]}×{test_frame.shape[0]})')

    # ── Ventana de preview ────────────────────────────────────────────────────
    WIN_NAME = f'Captura: {user_name}  [q=salir]'
    has_display = show_preview
    if has_display:
        try:
            cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WIN_NAME, 800, 500)
        except Exception:
            has_display = False
            print('   AVISO: No se pudo abrir ventana (modo headless)')

    count      = 0
    last_cap   = 0.0
    AUTO_DELAY = 0.6      # segundos entre capturas
    BLUR_THR   = 80.0     # Laplacian variance mínima para aceptar la foto
    blink_on   = False

    try:
        while count < num_samples:
            frame_rgb = grab_kinect_frame()
            if frame_rgb is None:
                time.sleep(0.1)
                continue

            # BGR para OpenCV
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            gray      = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            gray_eq   = cv2.equalizeHist(gray)

            faces = face_cascade.detectMultiScale(
                gray_eq, scaleFactor=1.12, minNeighbors=5, minSize=(70, 70)
            )

            now = time.time()
            captured_this_frame = False

            if len(faces) == 1:
                x, y, w, h = faces[0]
                face_roi    = gray[y:y + h, x:x + w]
                blur        = _blur_score(face_roi)
                ready       = (now - last_cap) > AUTO_DELAY
                blur_ok     = blur > BLUR_THR

                if ready and blur_ok:
                    face_resized = cv2.resize(face_roi, (200, 200))
                    idx      = existing + count
                    filepath = os.path.join(user_dir, f'face_{idx:04d}.jpg')
                    cv2.imwrite(filepath, face_resized)
                    count        += 1
                    last_cap      = now
                    blink_on      = True
                    captured_this_frame = True
                    bar = '█' * count + '░' * (num_samples - count)
                    print(f'  ✓ [{bar}] {count}/{num_samples}  '
                          f'(blur={blur:.0f})', flush=True)

                # Dibujar en preview
                if has_display:
                    color = (0, 255, 0) if blur_ok else (0, 165, 255)
                    cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), color, 2)
                    label = f'blur={blur:.0f}'
                    if not blur_ok:
                        label += ' MOVETE MENOS'
                    cv2.putText(frame_bgr, label, (x, y - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

            elif len(faces) == 0 and has_display:
                cv2.putText(frame_bgr, 'Sin cara detectada',
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 0, 255), 2)
            elif len(faces) > 1 and has_display:
                cv2.putText(frame_bgr,
                            f'{len(faces)} caras — debe haber solo 1',
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 165, 255), 2)

            if has_display:
                # Barra de progreso y parpadeo en captura
                blink_on = False
                pct = count / num_samples
                bar_w = int(760 * pct)
                cv2.rectangle(frame_bgr, (20, frame_bgr.shape[0] - 30),
                              (20 + bar_w, frame_bgr.shape[0] - 10),
                              (0, 255, 0), -1)
                cv2.rectangle(frame_bgr, (20, frame_bgr.shape[0] - 30),
                              (780, frame_bgr.shape[0] - 10),
                              (200, 200, 200), 2)
                cv2.putText(frame_bgr,
                            f'{count}/{num_samples}  |  {user_name}',
                            (20, frame_bgr.shape[0] - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                if captured_this_frame:
                    overlay = frame_bgr.copy()
                    cv2.rectangle(overlay, (0, 0),
                                  (frame_bgr.shape[1], frame_bgr.shape[0]),
                                  (0, 255, 0), -1)
                    frame_bgr = cv2.addWeighted(overlay, 0.25, frame_bgr, 0.75, 0)

                cv2.imshow(WIN_NAME, frame_bgr)
                key = cv2.waitKey(30) & 0xFF
                if key in (ord('q'), ord('Q'), 27):
                    print('\n  Captura interrumpida por el usuario.')
                    break

            time.sleep(0.03)  # ~30 FPS polling

    except KeyboardInterrupt:
        print('\n  Captura interrumpida (Ctrl+C)')
    finally:
        _sync.freenect_sync_stop()
        if has_display:
            cv2.destroyAllWindows()

    total = existing + count
    print(f'\n  Total acumulado: {total} muestras para "{user_name}"')

    if total >= 10:
        train_model()
    elif total >= 5:
        print(f'  AVISO: {total} muestras — recomendado ≥30 para buena precisión.')
        train_model()
    else:
        print(f'  ERROR: Solo {total} muestras — necesitas al menos 5.')
        print('  Ejecuta de nuevo para agregar más.')

    return count > 0


def _augment_face(img):
    """Genera variaciones augmentadas de una imagen de rostro 200x200.

    Retorna lista de imágenes adicionales (sin incluir la original).
    Augmentaciones:
      - Flip horizontal (espejo)
      - Brillo +20 / -20
      - Rotación ±5° y ±10°
      - Ligero zoom (1.1x recorte central)
    """
    augmented = []
    h, w = img.shape[:2]

    # 1. Flip horizontal
    augmented.append(cv2.flip(img, 1))

    # 2. Brillo +20 y -20
    for delta in [20, -20]:
        adj = np.clip(img.astype(np.int16) + delta, 0, 255).astype(np.uint8)
        augmented.append(adj)

    # 3. Rotaciones ±5° y ±10°
    center = (w // 2, h // 2)
    for angle in [5, -5, 10, -10]:
        M = cv2.getRotationMatrix2D(center, angle, 1.0)
        rot = cv2.warpAffine(img, M, (w, h), borderMode=cv2.BORDER_REFLECT)
        augmented.append(rot)

    # 4. Zoom 1.1x (recorte central)
    crop = int(w * 0.05)
    zoomed = img[crop:h - crop, crop:w - crop]
    augmented.append(cv2.resize(zoomed, (w, h)))

    # 5. Flip + brillo combinado
    flipped = cv2.flip(img, 1)
    adj = np.clip(flipped.astype(np.int16) + 15, 0, 255).astype(np.uint8)
    augmented.append(adj)

    return augmented  # 9 variaciones extra


def train_model():
    """Entrena el modelo LBPH con todas las caras registradas.

    Incluye augmentación de datos para mayor robustez:
    cada muestra genera ~9 variaciones adicionales.
    """
    print('\nEntrenando modelo LBPH (con augmentación de datos)...')

    faces = []
    labels = []
    label_map = {}
    label_id = 0

    if not os.path.isdir(FACES_DIR):
        print('Error: No hay directorio de rostros')
        return False

    for user_name in sorted(os.listdir(FACES_DIR)):
        user_dir = os.path.join(FACES_DIR, user_name)
        if not os.path.isdir(user_dir):
            continue

        label_map[str(label_id)] = user_name
        originals = 0

        for filename in sorted(os.listdir(user_dir)):
            if not filename.endswith('.jpg'):
                continue
            filepath = os.path.join(user_dir, filename)
            img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                img_200 = cv2.resize(img, (200, 200))
                # Original
                faces.append(img_200)
                labels.append(label_id)
                # Augmentaciones
                for aug_img in _augment_face(img_200):
                    faces.append(aug_img)
                    labels.append(label_id)
                originals += 1

        total = originals * 10  # original + 9 augmentaciones
        print(f'  {user_name}: {originals} originales '
              f'-> {total} con augmentación (label={label_id})')
        label_id += 1

    if len(faces) < 5:
        print(f'Error: Insuficientes muestras ({len(faces)}). Minimo 5.')
        return False

    # LBPH con parámetros mejorados: radius=2 captura patrones más grandes
    recognizer = cv2.face.LBPHFaceRecognizer_create(
        radius=2, neighbors=8, grid_x=8, grid_y=8
    )
    recognizer.train(faces, np.array(labels))

    model_path = os.path.join(FACES_DIR, 'lbph_model.yml')
    recognizer.save(model_path)

    map_path = os.path.join(FACES_DIR, 'label_map.json')
    with open(map_path, 'w') as f:
        json.dump(label_map, f, indent=2)

    print(f'\nModelo guardado: {model_path}')
    print(f'   Label map: {map_path}')
    print(f'   Usuarios: {len(label_map)}, Muestras totales: {len(faces)}')

    # ── Test de threshold sugerido ────────────────────────────────────────────
    # Tomar una muestra de originales (sin augmentar) y predecir sobre sí mismos
    print('\nCalculando threshold sugerido...')
    confs = []
    for user_name in sorted(os.listdir(FACES_DIR)):
        user_dir = os.path.join(FACES_DIR, user_name)
        if not os.path.isdir(user_dir):
            continue
        uid = next((k for k, v in label_map.items() if v == user_name), None)
        if uid is None:
            continue
        imgs = sorted([f for f in os.listdir(user_dir) if f.endswith('.jpg')])
        # Tomar máx 10 imágenes originales para el test
        for fn in imgs[:10]:
            img = cv2.imread(os.path.join(user_dir, fn), cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            img_200 = cv2.resize(img, (200, 200))
            _, conf = recognizer.predict(img_200)
            confs.append(conf)

    if confs:
        avg_conf = sum(confs) / len(confs)
        max_conf = max(confs)
        suggested = round(max_conf * 1.4)  # margen del 40% sobre el peor caso propio
        suggested = min(max(suggested, 60), 120)  # clamp [60, 120]
        print(f'\n╔══════════════════════════════════════════════════╗')
        print(f'║  RESULTADO DEL ENTRENAMIENTO                     ║')
        print(f'╠══════════════════════════════════════════════════╣')
        print(f'║  Confianza promedio (sobre sí mismo): {avg_conf:>6.1f}    ║')
        print(f'║  Confianza máxima  (peor caso propio): {max_conf:>5.1f}    ║')
        print(f'║  Threshold sugerido:                   {suggested:>5.0f}    ║')
        print(f'╠══════════════════════════════════════════════════╣')
        print(f'║  Para ajustar en el nodo:                        ║')
        print(f'║    ros2 run smart_t_ai_v2 hailo_face_gesture_node║')
        print(f'║      --ros-args -p lbph_confidence:={suggested:<14.0f}║')
        print(f'╚══════════════════════════════════════════════════╝')

    return True


def delete_user(user_name):
    """Elimina un usuario y re-entrena."""
    user_dir = os.path.join(FACES_DIR, user_name)
    if not os.path.isdir(user_dir):
        print(f'Error: Usuario "{user_name}" no encontrado')
        return
    shutil.rmtree(user_dir)
    print(f'Usuario "{user_name}" eliminado')

    remaining = [d for d in os.listdir(FACES_DIR)
                 if os.path.isdir(os.path.join(FACES_DIR, d))]
    if remaining:
        train_model()
    else:
        for f in ['lbph_model.yml', 'label_map.json']:
            p = os.path.join(FACES_DIR, f)
            if os.path.exists(p):
                os.remove(p)
        print('   No quedan usuarios. Modelo eliminado.')


def list_users():
    """Lista usuarios registrados."""
    if not os.path.isdir(FACES_DIR):
        print('No hay usuarios registrados.')
        return

    print('\nUsuarios registrados:')
    total_samples = 0
    for user_name in sorted(os.listdir(FACES_DIR)):
        user_dir = os.path.join(FACES_DIR, user_name)
        if not os.path.isdir(user_dir):
            continue
        n = len([f for f in os.listdir(user_dir) if f.endswith('.jpg')])
        total_samples += n
        augmented = n * 10
        print(f'  {user_name:20s}: {n:3d} fotos originales  (~{augmented} con augmentación)')

    model_path = os.path.join(FACES_DIR, 'lbph_model.yml')
    if os.path.exists(model_path):
        mtime = os.path.getmtime(model_path)
        dt = datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M')
        size_mb = os.path.getsize(model_path) / 1024 / 1024
        print(f'\nModelo LBPH: {model_path}')
        print(f'   Entrenado: {dt}  ({size_mb:.1f} MB)')
        print(f'   Total fotos originales: {total_samples}')
    else:
        print('\nAVISO: Modelo LBPH no entrenado. Ejecuta --train')
        return

    # Sugerir qué threshold usar
    label_map_path = os.path.join(FACES_DIR, 'label_map.json')
    if os.path.exists(label_map_path):
        print('\nPara lanzar el nodo con el threshold recomendado:')
        print('  Re-entrena con: python3 register_face.py --train')
        print('  (el entrenamiento imprime el threshold sugerido automáticamente)')


def main():
    parser = argparse.ArgumentParser(
        description='Registro de Rostros - Smart Trolley V5',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Ejemplos:
  python3 register_face.py --name jose              # captura 30 fotos
  python3 register_face.py --name jose --reset      # borra fotos viejas y recaptura
  python3 register_face.py --name jose --samples 50 # captura 50 fotos
  python3 register_face.py --name jose --no-preview # sin ventana (servidor headless)
  python3 register_face.py --list                   # listar usuarios
  python3 register_face.py --train                  # re-entrenar modelo
  python3 register_face.py --delete jose            # eliminar usuario
''')
    parser.add_argument('--name', '-n', type=str,
                        help='Nombre del usuario a registrar')
    parser.add_argument('--samples', '-s', type=int, default=30,
                        help='Número de muestras a capturar (default: 30)')
    parser.add_argument('--reset', '-r', action='store_true',
                        help='Borrar fotos existentes del usuario antes de capturar')
    parser.add_argument('--no-preview', action='store_true',
                        help='No mostrar ventana de preview (modo headless)')
    parser.add_argument('--list', '-l', action='store_true',
                        help='Listar usuarios registrados')
    parser.add_argument('--train', '-t', action='store_true',
                        help='Re-entrenar modelo LBPH con datos existentes')
    parser.add_argument('--delete', '-d', type=str,
                        help='Eliminar un usuario y re-entrenar')

    args = parser.parse_args()

    if args.list:
        list_users()
    elif args.train:
        train_model()
    elif args.delete:
        delete_user(args.delete)
    elif args.name:
        capture_faces(
            user_name=args.name,
            num_samples=args.samples,
            show_preview=not args.no_preview,
            reset=args.reset,
        )
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
