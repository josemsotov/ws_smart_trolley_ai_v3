#!/bin/bash
# Script de Pruebas para Smart Trolley V5
# Verifica que todos los componentes del sistema estén funcionando correctamente

echo "╔════════════════════════════════════════════════════════════╗"
echo "║  Smart Trolley V5 - Sistema de Verificación               ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Contadores
PASSED=0
FAILED=0

# Función para verificar
check_test() {
    if [ $1 -eq 0 ]; then
        echo -e "${GREEN}✓${NC} $2"
        ((PASSED++))
    else
        echo -e "${RED}✗${NC} $2"
        ((FAILED++))
    fi
}

echo "═══════════════════════════════════════════════════════════"
echo "1. VERIFICACIÓN DEL ENTORNO ROS2"
echo "═══════════════════════════════════════════════════════════"

# ROS2 instalado
ros2 --version &>/dev/null
check_test $? "ROS2 está instalado"

# Workspace compilado
if [ -f "install/setup.bash" ]; then
    check_test 0 "Workspace compilado correctamente"
else
    check_test 1 "Workspace NO compilado"
fi

# Source del workspace
if [ -n "$AMENT_PREFIX_PATH" ]; then
    check_test 0 "Variables de entorno ROS2 configuradas"
else
    echo -e "${YELLOW}⚠${NC} Warning: Source el workspace: source install/setup.bash"
    ((FAILED++))
fi

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "2. VERIFICACIÓN DE ARCHIVOS DEL PROYECTO"
echo "═══════════════════════════════════════════════════════════"

# URDF
[ -f "src/smart_t_ai_v2/description/robot.urdf.xacro" ]
check_test $? "robot.urdf.xacro existe"

# ros2_control URDF
[ -f "src/smart_t_ai_v2/description/ros2_control.xacro" ]
check_test $? "ros2_control.xacro existe"

# Configuración de controladores
[ -f "src/smart_t_ai_v2/config/ros2_controllers.yaml" ]
check_test $? "ros2_controllers.yaml existe"

# Arduino bridge
[ -f "src/smart_t_ai_v2/scripts/arduino_bridge.py" ]
check_test $? "arduino_bridge.py existe"

# Launch files
[ -f "src/smart_t_ai_v2/launch/launch_robot.launch.py" ]
check_test $? "launch_robot.launch.py existe"

[ -f "src/smart_t_ai_v2/launch/launch_sim_ros2_control.launch.py" ]
check_test $? "launch_sim_ros2_control.launch.py existe"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "3. VERIFICACIÓN DEL URDF"
echo "═══════════════════════════════════════════════════════════"

# Verificar URDF sin ros2_control
xacro src/smart_t_ai_v2/description/robot.urdf.xacro use_ros2_control:=false 2>/dev/null | grep -q "robot"
check_test $? "URDF genera correctamente (sin ros2_control)"

# Verificar URDF con ros2_control
xacro src/smart_t_ai_v2/description/robot.urdf.xacro use_ros2_control:=true 2>/dev/null | grep -q "ros2_control"
check_test $? "URDF genera correctamente (con ros2_control)"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "4. VERIFICACIÓN DE DEPENDENCIAS"
echo "═══════════════════════════════════════════════════════════"

# Verificar paquetes ROS2
ros2 pkg list 2>/dev/null | grep -q "ros2_control"
check_test $? "ros2_control instalado"

ros2 pkg list 2>/dev/null | grep -q "controller_manager"
check_test $? "controller_manager instalado"

ros2 pkg list 2>/dev/null | grep -q "diff_drive_controller"
check_test $? "diff_drive_controller instalado"

ros2 pkg list 2>/dev/null | grep -q "gz_ros2_control"
check_test $? "gz_ros2_control instalado"

# Verificar Python serial
python3 -c "import serial" 2>/dev/null
check_test $? "Python serial instalado"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "5. VERIFICACIÓN DEL HARDWARE (si está conectado)"
echo "═══════════════════════════════════════════════════════════"

# Arduino
if [ -e "/dev/ttyACM0" ]; then
    check_test 0 "Arduino conectado en /dev/ttyACM0"
    
    # Permisos
    if [ -r "/dev/ttyACM0" ] && [ -w "/dev/ttyACM0" ]; then
        check_test 0 "Permisos correctos en /dev/ttyACM0"
    else
        echo -e "${YELLOW}⚠${NC} Warning: Sin permisos de lectura/escritura. Ejecuta: sudo chmod 666 /dev/ttyACM0"
        ((FAILED++))
    fi
else
    echo -e "${YELLOW}⚠${NC} Arduino NO conectado (opcional para simulación)"
fi

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "RESUMEN"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo -e "Tests Pasados: ${GREEN}$PASSED${NC}"
echo -e "Tests Fallados: ${RED}$FAILED${NC}"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}╔════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║  ✓ SISTEMA LISTO PARA USAR                ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════╝${NC}"
    echo ""
    echo "Puedes proceder a:"
    echo ""
    echo "  Hardware Real:"
    echo "    ./start_robot.sh"
    echo "    o"
    echo "    ros2 launch smart_t_ai_v2 launch_robot.launch.py"
    echo ""
    echo "  Simulación con ros2_control:"
    echo "    ros2 launch smart_t_ai_v2 launch_sim_ros2_control.launch.py"
    echo ""
    echo "  Simulación básica:"
    echo "    ros2 launch smart_t_ai_v2 launch_sim.launch.py"
    echo ""
else
    echo -e "${RED}╔════════════════════════════════════════════╗${NC}"
    echo -e "${RED}║  ✗ ALGUNOS COMPONENTES NECESITAN ATENCIÓN ║${NC}"
    echo -e "${RED}╚════════════════════════════════════════════╝${NC}"
    echo ""
    echo "Revisa los errores arriba y:"
    echo ""
    echo "  1. Asegúrate de haber compilado el workspace:"
    echo "     colcon build --symlink-install"
    echo ""
    echo "  2. Source el workspace:"
    echo "     source install/setup.bash"
    echo ""
    echo "  3. Instala dependencias faltantes:"
    echo "     rosdep install --from-paths src --ignore-src -r -y"
    echo ""
fi

exit $FAILED
