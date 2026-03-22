#!/bin/bash
# Script simple para probar movimiento del robot

cd ~/WS/ws_smart_trolley_ai_v2
source install/setup.bash

echo "╔════════════════════════════════════════╗"
echo "║   PRUEBA SIMPLE DE MOVIMIENTO         ║"
echo "╚════════════════════════════════════════╝"
echo ""

while true; do
    echo "Elige una opción:"
    echo "  1 - Avanzar (2 seg)"
    echo "  2 - Retroceder (2 seg)"
    echo "  3 - Girar izquierda (2 seg)"
    echo "  4 - Girar derecha (2 seg)"
    echo "  5 - Detener"
    echo "  0 - Salir"
    echo ""
    read -p "Opción: " opcion
    
    case $opcion in
        1)
            echo "⬆️  Avanzando..."
            timeout 2 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" >/dev/null 2>&1 &
            sleep 2.2
            ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
            echo "✅ Detenido"
            ;;
        2)
            echo "⬇️  Retrocediendo..."
            timeout 2 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.3}, angular: {z: 0.0}}" >/dev/null 2>&1 &
            sleep 2.2
            ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
            echo "✅ Detenido"
            ;;
        3)
            echo "◀️  Girando izquierda..."
            timeout 2 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.8}}" >/dev/null 2>&1 &
            sleep 2.2
            ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
            echo "✅ Detenido"
            ;;
        4)
            echo "▶️  Girando derecha..."
            timeout 2 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.8}}" >/dev/null 2>&1 &
            sleep 2.2
            ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
            echo "✅ Detenido"
            ;;
        5)
            echo "🛑 Deteniendo..."
            ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
            echo "✅ Robot detenido"
            ;;
        0)
            echo "👋 Saliendo..."
            ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
            exit 0
            ;;
        *)
            echo "❌ Opción no válida"
            ;;
    esac
    echo ""
done
