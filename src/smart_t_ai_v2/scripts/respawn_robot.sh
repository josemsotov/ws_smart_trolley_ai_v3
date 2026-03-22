#!/bin/bash
# Script para eliminar y respawnear el robot en el centro

# Esperar a que Gazebo esté listo
sleep 2

# Eliminar robot si existe
gz service -s /world/empty/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "smart_trolley"' 2>/dev/null

# Esperar un poco
sleep 1

# Respawnear robot en el centro
ros2 run ros_gz_sim create -topic robot_description -name smart_trolley -z 0.1

echo "Robot spawneado en el centro (0, 0, 0.1)"
