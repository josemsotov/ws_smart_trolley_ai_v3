#!/bin/bash
# Script para iniciar el robot con control por teclado
# Conexión: USB primario (/dev/ttyACM0 @ 115200) + BT fallback (/dev/rfcomm0 @ 38400)

echo "==================================="
echo " INICIANDO ROBOT CON TECLADO"
echo "==================================="
echo ""

# Source ROS2
source /home/josemsotov/WS/ws_smart_trolley_ai_v2/install/setup.bash

# ── Preparar BT fallback (no bloquea si HC-05 no está disponible) ──────────────
BT_MAC="98:D3:32:30:8C:62"
if ! ls /dev/rfcomm0 &>/dev/null; then
    echo "📡 Enlazando HC-05 BT como fallback (/dev/rfcomm0)..."
    sudo rfcomm bind rfcomm0 "$BT_MAC" 1 2>/dev/null && \
        echo "   ✅ rfcomm0 enlazado (BT fallback disponible)" || \
        echo "   ⚠️  rfcomm0 no disponible — solo USB activo"
else
    echo "   ✅ rfcomm0 ya enlazado (BT fallback disponible)"
fi

# Limpiar procesos previos
echo "1. Limpiando procesos anteriores..."
pkill -9 -f "arduino_bridge|test_movement" 2>/dev/null
sleep 2

# Iniciar arduino_bridge
echo "2. Iniciando arduino_bridge..."
python3 /home/josemsotov/WS/ws_smart_trolley_ai_v2/src/smart_t_ai_v2/scripts/arduino_bridge.py &
ARDUINO_PID=$!
sleep 2

echo ""
echo "✓ Arduino Bridge iniciado (PID: $ARDUINO_PID)"
echo ""
echo "====================================="
echo " CONTROL POR TECLADO"
echo "====================================="
echo ""
echo "Para mover el robot, ejecuta en otra terminal:"
echo ""
echo "  cd ~/WS/ws_smart_trolley_ai_v2"
echo "  source install/setup.bash"
echo "  python3 test_movement.py"
echo ""
echo "Controles:"
echo "  W = Avanzar"
echo "  S = Retroceder"
echo "  A = Girar izquierda"
echo "  D = Girar derecha"
echo "  SPACE = Detener"
echo "  Q = Salir"
echo ""
echo "Para detener arduino_bridge: pkill -9 -f arduino_bridge"
echo ""

# Mantener script corriendo
wait
