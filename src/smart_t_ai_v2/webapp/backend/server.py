#!/usr/bin/env python3
"""
SmartTrolley WebApp Backend
FastAPI + WebSocket — Bridge entre la PWA (iPhone/iPad) y ROS2

Arquitectura:
  PWA iPhone  ──WebSocket──►  server.py  ──rclpy──►  ROS2 Topics
                                         ◄──────────  /arduino_status
                                                       /odom
                                                       /gesture/status
  MJPEG Stream ──HTTP──►  /video_feed    ◄──────────  /kinect/rgb/image_raw

Correr:  python3 server.py
Puerto:  8765 (WebSocket) + 8000 (HTTP/REST/MJPEG)
"""

import asyncio
import json
import threading
import time
import logging
from typing import Set, Optional

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

# cv_bridge puede no estar disponible en entornos sin ROS instalado
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_OK = True
except ImportError:
    CV_BRIDGE_OK = False

logging.basicConfig(level=logging.INFO)
log = logging.getLogger("smarttrolley")

# ─────────────────────────────────────────────────────────────────────────────
# Estado global compartido entre ROS2 y FastAPI
# ─────────────────────────────────────────────────────────────────────────────
app_state = {
    # Modos activos
    "follower_enabled": False,
    "face_recognition_enabled": False,
    "gesture_control_enabled": False,
    "teleop_enabled": False,

    # Scorecard golf
    "current_hole": 1,
    "scores": {str(i): 0 for i in range(1, 19)},   # Hoyos 1-18
    "par": {str(i): 4 for i in range(1, 19)},       # Par por defecto 4

    # Telemetría
    "arduino_status": "DESCONECTADO",
    "odom_x": 0.0,
    "odom_y": 0.0,
    "odom_theta": 0.0,
    "linear_vel": 0.0,
    "angular_vel": 0.0,
    "gesture_status": "idle",

    # Cámara
    "latest_frame": None,        # numpy array BGR, thread-safe via lock
    "frame_lock": threading.Lock(),
}

# Clientes WebSocket conectados
ws_clients: Set[WebSocket] = set()
ws_lock = asyncio.Lock()

# ─────────────────────────────────────────────────────────────────────────────
# Nodo ROS2
# ─────────────────────────────────────────────────────────────────────────────
class TrolleyWebBridge(Node):
    """
    Nodo ROS2 que:
    - Suscribe a topics del trolley y actualiza app_state
    - Expone métodos para publicar comandos desde la API
    """

    def __init__(self):
        super().__init__('trolley_web_bridge')
        self.bridge = CvBridge() if CV_BRIDGE_OK else None

        # ── Publicadores ──────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.follower_enable_pub = self.create_publisher(Bool, '/person_follower/enable', 10)
        self.face_recog_pub = self.create_publisher(Bool, '/face_recognition/enable', 10)
        self.gesture_pub = self.create_publisher(Bool, '/gesture_control/enable', 10)

        # ── Suscriptores ──────────────────────────────────────────
        self.create_subscription(String, '/arduino_status',
                                 self._cb_arduino_status, 10)
        self.create_subscription(Odometry, '/odom',
                                 self._cb_odom, 10)
        self.create_subscription(String, '/gesture/status',
                                 self._cb_gesture_status, 10)
        self.create_subscription(Image, '/kinect/rgb/image_raw',
                                 self._cb_camera, 5)

        self.get_logger().info('TrolleyWebBridge iniciado')

    # ── Callbacks de suscriptores ─────────────────────────────────
    def _cb_arduino_status(self, msg: String):
        app_state["arduino_status"] = msg.data
        asyncio.run_coroutine_threadsafe(
            broadcast({"type": "arduino_status", "data": msg.data}),
            event_loop
        )

    def _cb_odom(self, msg: Odometry):
        app_state["odom_x"] = round(msg.pose.pose.position.x, 3)
        app_state["odom_y"] = round(msg.pose.pose.position.y, 3)
        app_state["linear_vel"] = round(msg.twist.twist.linear.x, 3)
        app_state["angular_vel"] = round(msg.twist.twist.angular.z, 3)
        # Enviar telemetría cada 10 callbacks (~1 Hz si odom va a 10 Hz)
        if not hasattr(self, '_odom_counter'):
            self._odom_counter = 0
        self._odom_counter += 1
        if self._odom_counter % 10 == 0:
            asyncio.run_coroutine_threadsafe(
                broadcast({"type": "telemetry", "data": {
                    "x": app_state["odom_x"],
                    "y": app_state["odom_y"],
                    "linear_vel": app_state["linear_vel"],
                    "angular_vel": app_state["angular_vel"],
                }}),
                event_loop
            )

    def _cb_gesture_status(self, msg: String):
        app_state["gesture_status"] = msg.data
        asyncio.run_coroutine_threadsafe(
            broadcast({"type": "gesture_status", "data": msg.data}),
            event_loop
        )

    def _cb_camera(self, msg: Image):
        if self.bridge is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Escalar para no saturar el stream (720p → 480p)
            h, w = frame.shape[:2]
            if w > 640:
                scale = 640 / w
                frame = cv2.resize(frame, (640, int(h * scale)))
            with app_state["frame_lock"]:
                app_state["latest_frame"] = frame
        except Exception as e:
            self.get_logger().warn(f'Camera frame error: {e}')

    # ── Métodos de publicación (llamados desde FastAPI) ───────────
    def send_cmd_vel(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        self.send_cmd_vel(0.0, 0.0)

    def set_follower(self, enabled: bool):
        app_state["follower_enabled"] = enabled
        msg = Bool()
        msg.data = enabled
        self.follower_enable_pub.publish(msg)

    def set_face_recognition(self, enabled: bool):
        app_state["face_recognition_enabled"] = enabled
        msg = Bool()
        msg.data = enabled
        self.face_recog_pub.publish(msg)

    def set_gesture_control(self, enabled: bool):
        app_state["gesture_control_enabled"] = enabled
        msg = Bool()
        msg.data = enabled
        self.gesture_pub.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 spin en hilo separado
# ─────────────────────────────────────────────────────────────────────────────
ros_node: Optional[TrolleyWebBridge] = None
event_loop: Optional[asyncio.AbstractEventLoop] = None


def ros_spin_thread():
    global ros_node
    rclpy.init()
    ros_node = TrolleyWebBridge()
    try:
        rclpy.spin(ros_node)
    except Exception as e:
        log.error(f'ROS2 spin error: {e}')
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


# ─────────────────────────────────────────────────────────────────────────────
# FastAPI app
# ─────────────────────────────────────────────────────────────────────────────
app = FastAPI(title="SmartTrolley WebApp", version="1.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Servir el frontend React compilado (build/)
import os
frontend_build = os.path.join(os.path.dirname(__file__), "../frontend/build")

# Montar solo los assets estáticos (JS, CSS, imágenes) bajo /assets/
# NO montar en "/" raíz — eso interceptaría las rutas API
_assets_dir = os.path.join(frontend_build, "assets")
if os.path.isdir(_assets_dir):
    app.mount("/assets", StaticFiles(directory=_assets_dir), name="assets")


# ─────────────────────────────────────────────────────────────────────────────
# WebSocket — canal principal bidireccional
# ─────────────────────────────────────────────────────────────────────────────
async def broadcast(message: dict):
    """Envía un mensaje JSON a todos los clientes WebSocket conectados."""
    if not ws_clients:
        return
    payload = json.dumps(message)
    dead = set()
    async with ws_lock:
        for ws in ws_clients:
            try:
                await ws.send_text(payload)
            except Exception:
                dead.add(ws)
        ws_clients.difference_update(dead)


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    async with ws_lock:
        ws_clients.add(ws)
    log.info(f"WebSocket conectado. Clientes: {len(ws_clients)}")

    # Enviar estado completo al conectar
    await ws.send_text(json.dumps({"type": "full_state", "data": {
        "follower_enabled":       app_state["follower_enabled"],
        "face_recognition_enabled": app_state["face_recognition_enabled"],
        "gesture_control_enabled":  app_state["gesture_control_enabled"],
        "arduino_status":           app_state["arduino_status"],
        "current_hole":             app_state["current_hole"],
        "scores":                   app_state["scores"],
        "par":                      app_state["par"],
    }}))

    try:
        while True:
            raw = await ws.receive_text()
            await handle_ws_message(raw)
    except WebSocketDisconnect:
        pass
    finally:
        async with ws_lock:
            ws_clients.discard(ws)
        log.info(f"WebSocket desconectado. Clientes: {len(ws_clients)}")


async def handle_ws_message(raw: str):
    """Procesa mensajes entrantes del cliente (iPhone/iPad)."""
    try:
        msg = json.loads(raw)
    except json.JSONDecodeError:
        return

    action = msg.get("action")
    data = msg.get("data", {})

    if action == "set_mode":
        mode = data.get("mode")
        enabled = data.get("enabled", False)
        if ros_node is None:
            return
        if mode == "follower":
            ros_node.set_follower(enabled)
        elif mode == "face_recognition":
            ros_node.set_face_recognition(enabled)
        elif mode == "gesture_control":
            ros_node.set_gesture_control(enabled)
        await broadcast({"type": "mode_update", "data": {
            "mode": mode, "enabled": enabled
        }})

    elif action == "cmd_vel":
        if ros_node:
            ros_node.send_cmd_vel(
                data.get("linear", 0.0),
                data.get("angular", 0.0)
            )

    elif action == "stop":
        if ros_node:
            ros_node.stop()

    elif action == "score_update":
        hole = str(data.get("hole", app_state["current_hole"]))
        delta = data.get("delta", 0)     # +1 o -1
        app_state["scores"][hole] = max(0, app_state["scores"][hole] + delta)
        await broadcast({"type": "score_update", "data": {
            "hole": hole,
            "score": app_state["scores"][hole],
            "scores": app_state["scores"],
        }})

    elif action == "set_hole":
        app_state["current_hole"] = data.get("hole", 1)
        await broadcast({"type": "hole_update", "data": {
            "current_hole": app_state["current_hole"]
        }})

    elif action == "reset_scores":
        app_state["scores"] = {str(i): 0 for i in range(1, 19)}
        await broadcast({"type": "scores_reset", "data": app_state["scores"]})

    elif action == "ping":
        await broadcast({"type": "pong", "ts": time.time()})


# ─────────────────────────────────────────────────────────────────────────────
# MJPEG Stream — cámara en vivo
# ─────────────────────────────────────────────────────────────────────────────
def _open_usb_camera():
    """Busca la primera cámara USB funcional como fallback."""
    for idx in [0, 1, 2]:
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            continue
        # Usar resolución nativa MJPEG de la cámara (evitar resize forzado)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)
        # Drenar buffer y esperar frame real
        for _ in range(20):
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                log.info(f"Cámara USB fallback: índice {idx} {frame.shape}")
                return cap
            time.sleep(0.05)
        cap.release()
    return None

_usb_cap = None          # cámara USB fallback (abre bajo demanda)
_usb_cap_lock = threading.Lock()


def gen_mjpeg():
    """
    Generador de frames MJPEG.
    Prioridad: 1) Frames de ROS2 (Kinect)  2) Cámara USB  3) Placeholder
    """
    global _usb_cap
    placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.putText(placeholder, "Sin camara", (60, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)

    while True:
        # ── 1. Frame de ROS2 (Kinect) ─────────────────────────────────────
        with app_state["frame_lock"]:
            frame = app_state["latest_frame"]

        if frame is not None:
            # Hay Kinect activo — cerrar USB si estaba abierta
            with _usb_cap_lock:
                if _usb_cap is not None:
                    _usb_cap.release()
                    _usb_cap = None
        else:
            # ── 2. Fallback cámara USB ─────────────────────────────────────
            with _usb_cap_lock:
                if _usb_cap is None:
                    _usb_cap = _open_usb_camera()
                if _usb_cap is not None:
                    ret, usb_frame = _usb_cap.read()
                    if ret and usb_frame is not None and usb_frame.size > 0:
                        frame = usb_frame
                    else:
                        # Cámara perdida — reabrir en próxima iteración
                        _usb_cap.release()
                        _usb_cap = None

            # ── 3. Placeholder si no hay nada ─────────────────────────────
            if frame is None:
                frame = placeholder

        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
        if ok:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' +
                   buf.tobytes() + b'\r\n')
        time.sleep(1 / 15)   # 15 fps máx


@app.get("/video_feed")
def video_feed():
    return StreamingResponse(
        gen_mjpeg(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


# ─────────────────────────────────────────────────────────────────────────────
# REST endpoints de respaldo (compatibilidad)
# ─────────────────────────────────────────────────────────────────────────────
@app.get("/api/state")
def get_state():
    return JSONResponse({
        "follower_enabled":          app_state["follower_enabled"],
        "face_recognition_enabled":  app_state["face_recognition_enabled"],
        "gesture_control_enabled":   app_state["gesture_control_enabled"],
        "arduino_status":            app_state["arduino_status"],
        "current_hole":              app_state["current_hole"],
        "scores":                    app_state["scores"],
        "par":                       app_state["par"],
    })


# ─────────────────────────────────────────────────────────────────────────────
# Catch-all SPA — sirve index.html para todas las rutas no-API (React Router)
# DEBE ser el último endpoint registrado
# ─────────────────────────────────────────────────────────────────────────────
from fastapi.responses import FileResponse as _FileResponse

@app.get("/api/health")
def health():
    return {"status": "ok", "ros2": ros_node is not None}


@app.get("/{full_path:path}")
async def serve_spa(full_path: str):
    """Sirve archivos estáticos del build React o index.html (SPA routing)."""
    if not os.path.isdir(frontend_build):
        return JSONResponse({"error": "frontend no compilado"}, status_code=503)
    # Intentar servir el archivo exacto (favicons, manifest, sw.js…)
    candidate = os.path.join(frontend_build, full_path)
    if os.path.isfile(candidate):
        return _FileResponse(candidate)
    # Para cualquier otra ruta (React Router), devolver index.html
    index = os.path.join(frontend_build, "index.html")
    if os.path.isfile(index):
        return _FileResponse(index)
    return JSONResponse({"error": "index.html no encontrado"}, status_code=503)


# ─────────────────────────────────────────────────────────────────────────────
# Punto de entrada
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys

    # Arrancar ROS2 en hilo background
    ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    ros_thread.start()
    log.info("ROS2 hilo iniciado")

    # Obtener el event loop de asyncio para poder enviar desde ROS2
    async def _set_loop():
        global event_loop
        event_loop = asyncio.get_running_loop()

    async def _run_server():
        global event_loop
        event_loop = asyncio.get_running_loop()
        config = uvicorn.Config(app, host="0.0.0.0", port=8000,
                                log_level="info", ws_ping_interval=20)
        server = uvicorn.Server(config)
        await server.serve()

    asyncio.run(_run_server())
