#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# start_webapp.sh — Arrancar el backend SmartTrolley PWA
#
# Uso:
#   ./start_webapp.sh            # arranca en primer plano
#   ./start_webapp.sh --bg       # arranca en background (log en /tmp/trolley_webapp.log)
# ─────────────────────────────────────────────────────────────────────────────
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND="$SCRIPT_DIR/src/smart_t_ai_v2/webapp/backend/server.py"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$SCRIPT_DIR/install/setup.bash"
LOG="/tmp/trolley_webapp.log"

# Cargar ROS2
if [[ -f "$ROS_SETUP" ]]; then
  source "$ROS_SETUP"
else
  echo "[ERROR] ROS2 setup no encontrado: $ROS_SETUP"
  exit 1
fi

# Cargar workspace local (si existe build)
if [[ -f "$WS_SETUP" ]]; then
  source "$WS_SETUP"
fi

KINECT_SCRIPT="$SCRIPT_DIR/src/smart_t_ai_v2/scripts/kinect_node.py"
KINECT_LOG="/tmp/trolley_kinect.log"

PI_IP=$(hostname -I | awk '{print $1}')
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " SmartTrolley WebApp"
echo " iPhone → http://$PI_IP:8000"
echo " WiFi hotspot: SmartTrolley / trolley123"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# ── Hotspot WiFi (si wlan0 está disponible) ───────────────────────────────
if nmcli device status | grep -q "wlan0.*disconnected"; then
  echo " [WiFi] Activando hotspot SmartTrolley..."
  nmcli radio wifi on 2>/dev/null
  sudo nmcli device wifi hotspot ssid "SmartTrolley" password "trolley123" ifname wlan0 2>/dev/null \
    && echo " [WiFi] Hotspot activo → http://10.42.0.1:8000" \
    || echo " [WiFi] Hotspot no disponible (continuando sin él)"
fi

# ── Kinect node ───────────────────────────────────────────────────────────
if [[ -f "$KINECT_SCRIPT" ]]; then
  pkill -f "kinect_node.py" 2>/dev/null; sleep 1
  echo " [Kinect] Arrancando kinect_node..."
  nohup python3 "$KINECT_SCRIPT" > "$KINECT_LOG" 2>&1 &
  KINECT_PID=$!
  echo " [Kinect] PID=$KINECT_PID  log: $KINECT_LOG"
fi

echo ""

# ── Backend FastAPI ───────────────────────────────────────────────────────
if [[ "$1" == "--bg" ]]; then
  echo " [Server] Ejecutando en BACKGROUND → log: $LOG"
  nohup python3 "$BACKEND" > "$LOG" 2>&1 &
  echo " [Server] PID=$!  log: $LOG"
  echo ""
  echo " Ver logs:"
  echo "   tail -f $LOG"
  echo "   tail -f $KINECT_LOG"
  echo " Parar todo: pkill -f 'server.py'; pkill -f 'kinect_node.py'"
else
  echo " [Server] Ejecutando en primer plano (Ctrl+C para parar)"
  python3 "$BACKEND"
  # Al parar el servidor, parar también el Kinect
  pkill -f "kinect_node.py" 2>/dev/null
fi
