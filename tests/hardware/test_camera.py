#!/usr/bin/env python3
"""
Test de cámaras disponibles en el robot.
Prueba la cámara integrada y el Kinect.
"""
import cv2
import time
import sys
import os

def test_v4l2_camera(device, name, width=640, height=480):
    """Intenta capturar una imagen de un dispositivo V4L2."""
    print(f"\n--- Probando {name} ({device}) ---")
    
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"  ❌ No se pudo abrir {device}")
        return False
    
    # Configurar MJPG primero (menor ancho de banda)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    print(f"  Resolución configurada: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    
    time.sleep(0.5)
    
    # Intentar capturar
    for attempt in range(5):
        ret, frame = cap.read()
        if ret:
            filename = f"camera_test_{name.replace(' ', '_').lower()}.jpg"
            filepath = os.path.join(os.path.dirname(__file__), filename)
            cv2.imwrite(filepath, frame)
            print(f"  ✅ Imagen capturada: {frame.shape[1]}x{frame.shape[0]}")
            print(f"  📁 Guardada en: {filepath}")
            cap.release()
            return True
        time.sleep(0.2)
    
    print(f"  ❌ No se pudo capturar frame después de 5 intentos")
    cap.release()
    return False


def test_kinect_freenect():
    """Intenta capturar imagen usando libfreenect (Kinect)."""
    print("\n--- Probando Kinect via freenect ---")
    try:
        import freenect
        import numpy as np
        
        # Capturar RGB
        frame, _ = freenect.sync_get_video()
        if frame is not None:
            # freenect devuelve RGB, OpenCV usa BGR
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            filepath = os.path.join(os.path.dirname(__file__), "camera_test_kinect_rgb.jpg")
            cv2.imwrite(filepath, frame_bgr)
            print(f"  ✅ Kinect RGB capturado: {frame_bgr.shape[1]}x{frame_bgr.shape[0]}")
            print(f"  📁 Guardada en: {filepath}")
            
            # Capturar profundidad
            depth, _ = freenect.sync_get_depth()
            if depth is not None:
                # Normalizar profundidad a 8 bits para visualización
                depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
                depth_8bit = depth_normalized.astype('uint8')
                depth_color = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)
                filepath_depth = os.path.join(os.path.dirname(__file__), "camera_test_kinect_depth.jpg")
                cv2.imwrite(filepath_depth, depth_color)
                print(f"  ✅ Kinect Depth capturado: {depth.shape[1]}x{depth.shape[0]}")
                print(f"  📁 Guardada en: {filepath_depth}")
            
            freenect.sync_stop()
            return True
        else:
            print("  ❌ freenect no devolvió frame")
            freenect.sync_stop()
            return False
            
    except ImportError:
        print("  ⚠️  Módulo freenect no disponible (pip install freenect)")
        return False
    except Exception as e:
        print(f"  ❌ Error: {e}")
        try:
            import freenect
            freenect.sync_stop()
        except:
            pass
        return False


def main():
    print("=" * 50)
    print("  TEST DE CÁMARAS - Smart Trolley Robot")
    print("=" * 50)
    
    # Listar dispositivos de video
    print("\nDispositivos de video detectados:")
    for i in range(20):
        dev = f"/dev/video{i}"
        if os.path.exists(dev):
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            if cap.isOpened():
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                print(f"  {dev}: {w}x{h}")
                cap.release()
    
    results = {}
    
    # Test 1: Cámara integrada
    results['Integrada'] = test_v4l2_camera('/dev/video0', 'Integrada', 640, 480)
    
    # Test 2: Kinect via freenect
    results['Kinect'] = test_kinect_freenect()
    
    # Resumen
    print("\n" + "=" * 50)
    print("  RESUMEN")
    print("=" * 50)
    for name, ok in results.items():
        status = "✅ OK" if ok else "❌ FALLO"
        print(f"  {name}: {status}")
    
    if not any(results.values()):
        print("\n⚠️  NOTA: La cámara integrada y el Kinect comparten el")
        print("   Bus USB 2 (480Mbps). El Kinect consume mucho ancho de")
        print("   banda, lo que impide que la cámara integrada funcione.")
        print("\n   SOLUCIONES:")
        print("   1. Mover la cámara integrada a otro puerto USB (Bus 4)")
        print("   2. Usar solo la cámara del Kinect via freenect/OpenNI")
        print("   3. Desconectar el Kinect para usar la cámara integrada")

    print()


if __name__ == '__main__':
    main()
