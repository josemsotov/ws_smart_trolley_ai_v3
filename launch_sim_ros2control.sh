#!/bin/bash
# Script para lanzar simulación con ros2_control
# Espera 20 segundos y luego puedes controlar el robot

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  SMART TROLLEY V5 - Simulación con ros2_control          ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

cd ~/WS/ws_smart_trolley_ai_v2
source install/setup.bash

echo "🚀 Iniciando Gazebo y ros2_control..."
echo ""
echo "⏳ Esto tomará unos 20 segundos..."
echo ""
echo "Después podrás controlar el robot en otra terminal con:"
echo ""
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard \\"
echo "    --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped"
echo ""
echo "═══════════════════════════════════════════════════════════"
echo ""

ros2 launch smart_t_ai_v2 launch_sim_ros2_control.launch.py
