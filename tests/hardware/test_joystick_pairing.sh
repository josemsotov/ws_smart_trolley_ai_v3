#!/bin/bash
# Script de diagnóstico y test continuo del joystick

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🎮 MONITOR DE JOYSTICK EN TIEMPO REAL"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Verificar devices
echo "📋 Dispositivos detectados:"
for i in /dev/input/event{0..3}; do
    if [ -e "$i" ]; then
        name=$(cat /sys/class/input/$(basename $i)/device/name 2>/dev/null || echo "Unknown")
        echo "  $i → $name"
    fi
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔍 PROCESO DE EMPAREJAMIENTO"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Prueba estos pasos EN ORDEN:"
echo ""
echo "1️⃣  APAGAR EL CONTROL:"
echo "   → Mantén MODE presionado 10+ segundos"
echo "   → El LED debería apagarse"
echo ""
echo "2️⃣  ENCENDER EL CONTROL:"
echo "   → Presiona START o MODE brevemente"
echo "   → Observa el LED:"
echo "      • Parpadeo LENTO = buscando dongle"
echo "      • Parpadeo RÁPIDO = modo pairing"
echo "      • FIJO = conectado"
echo ""
echo "3️⃣  SI PARPADEA (no conecta):"
echo "   → Método A: MODE + START (simultáneos 3 seg)"
echo "   → Método B: START + SELECT (simultáneos 3 seg)"
echo "   → Método C: HOME/PS (si tiene, 5 seg)"
echo ""
echo "4️⃣  VERIFICAR DONGLE USB:"
echo "   → Busca un botón pequeño en el dongle"
echo "   → Si lo tiene, presiónalo antes de encender el control"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "⏱️  MONITOREANDO EVENTOS (60 segundos)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Mientras pruebas los pasos, detectaré automáticamente"
echo "cuando el control transmita datos."
echo ""
echo "Presiona CTRL+C cuando logres conectar o para salir."
echo ""
echo "Iniciando monitoreo..."
echo ""

# Monitor en paralelo de ambos event devices
timeout 60 bash -c '
    echo "[MONITOR ACTIVO] Esperando eventos del joystick..."
    echo ""
    
    while true; do
        # Check event0
        if timeout 0.1 evtest --grab /dev/input/event0 2>&1 | grep -q "Event:"; then
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "✅ CONTROL DETECTADO EN /dev/input/event0"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo ""
            echo "🎉 ¡ÉXITO! El control está transmitiendo."
            echo ""
            echo "Eventos recibidos:"
            timeout 3 evtest /dev/input/event0 2>&1 | grep "Event:" | head -5
            exit 0
        fi
        
        # Check event1
        if timeout 0.1 evtest --grab /dev/input/event1 2>&1 | grep -q "Event:"; then
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "✅ CONTROL DETECTADO EN /dev/input/event1"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo ""
            echo "🎉 ¡ÉXITO! El control está transmitiendo."
            echo ""
            echo "Eventos recibidos:"
            timeout 3 evtest /dev/input/event1 2>&1 | grep "Event:" | head -5
            exit 0
        fi
        
        sleep 0.5
    done
' && echo "" && echo "✅ Control conectado correctamente" || echo "" && echo "⏱️  Tiempo agotado. Si el control no responde, intenta:" && echo "   • Reconectar el dongle en Windows (usbipd detach + attach)" && echo "   • Usar el segundo control del set" && echo "   • Cambiar a control por teclado temporalmente"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
