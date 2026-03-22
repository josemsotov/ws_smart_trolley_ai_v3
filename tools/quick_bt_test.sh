#!/bin/bash
# Script rápido para probar conexión Bluetooth con Arduino
# Uso: ./quick_bt_test.sh [puerto] [baud_rate]

PORT=${1:-/dev/rfcomm0}
BAUD=${2:-115200}

echo "═══════════════════════════════════════════════════════════"
echo "  QUICK BLUETOOTH TEST - Arduino Communication"
echo "═══════════════════════════════════════════════════════════"
echo "  Puerto: $PORT"
echo "  Baud:   $BAUD"
echo "═══════════════════════════════════════════════════════════"
echo ""

# Verificar que el puerto existe
if [ ! -e "$PORT" ]; then
    echo "❌ Error: El puerto $PORT no existe"
    echo ""
    echo "Crea el puerto primero:"
    echo "  python3 src/smart_t_ai_v2/scripts/bluetooth_helper.py connect HC-05"
    echo "  o"
    echo "  sudo rfcomm bind /dev/rfcomm0 <MAC_ADDRESS> 1"
    exit 1
fi

echo "✅ Puerto $PORT existe"
echo ""

# Test 1: Verificar permisos
echo "──────────────────────────────────────────────────────────"
echo "TEST 1: Verificando permisos..."
echo "──────────────────────────────────────────────────────────"
if [ -r "$PORT" ] && [ -w "$PORT" ]; then
    echo "✅ Permisos OK (lectura/escritura)"
else
    echo "⚠️  Permisos insuficientes"
    echo "Ejecuta: sudo chmod 666 $PORT"
fi
echo ""

# Test 2: Test con cat (lectura simple)
echo "──────────────────────────────────────────────────────────"
echo "TEST 2: Lectura básica (5 segundos)..."
echo "──────────────────────────────────────────────────────────"
echo "Esperando datos del Arduino..."

timeout 5s cat "$PORT" 2>/dev/null || {
    echo "⚠️  No se recibieron datos"
    echo "   (Puede ser normal si el Arduino no envía datos continuamente)"
}
echo ""

# Test 3: Test con stty (configuración serial)
echo "──────────────────────────────────────────────────────────"
echo "TEST 3: Configuración del puerto..."
echo "──────────────────────────────────────────────────────────"
stty -F "$PORT" "$BAUD" raw -echo 2>/dev/null && {
    echo "✅ Puerto configurado a $BAUD baud"
} || {
    echo "❌ Error configurando puerto"
}
echo ""

# Test 4: Enviar comando simple
echo "──────────────────────────────────────────────────────────"
echo "TEST 4: Enviar comando y leer respuesta..."
echo "──────────────────────────────────────────────────────────"

# Configurar puerto
stty -F "$PORT" "$BAUD" raw -echo 2>/dev/null

# Enviar comando y leer respuesta
(
    echo "?" > "$PORT"  # Enviar comando de status/info
    sleep 1
    timeout 3s cat "$PORT"
) 2>/dev/null && {
    echo "✅ Comunicación bidireccional exitosa"
} || {
    echo "⚠️  Sin respuesta a comando"
}
echo ""

# Test 5: Probar con Python
echo "──────────────────────────────────────────────────────────"
echo "TEST 5: Test con Python (pyserial)..."
echo "──────────────────────────────────────────────────────────"

python3 - <<EOF
import serial
import time
import sys

try:
    ser = serial.Serial('$PORT', $BAUD, timeout=2)
    print(f"✅ Puerto abierto con pyserial")
    time.sleep(2)
    
    # Limpiar buffer
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    # Enviar comando
    ser.write(b"?\n")
    ser.flush()
    print("📤 Comando enviado: ?")
    
    # Leer respuesta
    time.sleep(0.5)
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        print(f"📥 Respuesta recibida ({len(data)} bytes):")
        print(f"   {data}")
    else:
        print("⚠️  Sin respuesta inmediata")
    
    ser.close()
    print("✅ Test Python exitoso")
    
except serial.SerialException as e:
    print(f"❌ Error: {e}")
    sys.exit(1)
except Exception as e:
    print(f"❌ Error inesperado: {e}")
    sys.exit(1)
EOF

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  TESTS COMPLETADOS"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "Próximos pasos:"
echo "  1. Si todo funciona, usa: python3 test_bluetooth_arduino.py $PORT"
echo "  2. Para modo interactivo: python3 test_bluetooth_arduino.py $PORT -i"
echo "  3. Para ROS: ros2 launch smart_t_ai_v2 launch_robot.launch.py \\"
echo "               connection_type:=bluetooth serial_port:=$PORT"
echo ""
