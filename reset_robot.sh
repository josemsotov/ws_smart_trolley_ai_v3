#!/bin/bash
# Comando para resetear el robot al centro de la simulación

# Detener el robot
ros2 topic pub --once /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" 2>/dev/null &

# Eliminar robot
gz service -s /world/empty/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "smart_trolley"' 2>/dev/null

# Esperar
sleep 1

# Respawnear en el centro
cd /home/josemsotov/WS/ws_smart_trolley_ai_v2
source install/setup.bash
ros2 run ros_gz_sim create -topic robot_description -name smart_trolley -z 0.1

echo "✅ Robot reseteado al centro (0, 0, 0.1)"
