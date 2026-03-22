# Guía de Control del Smart Trolley V5

Este documento explica las diferentes opciones de control para el Smart Trolley V5.

## Dos Modos de Operación

El robot soporta dos modos principales:

### 1. **Hardware Real** - Arduino Bridge
Para controlar el robot físico con Arduino Mega 2560.

### 2. **Simulación** - ros2_control o Plugin Gazebo
Para simular el robot en Gazebo Simulator.

---

## HARDWARE REAL - Arduino Bridge

### Configuración
El arduino_bridge se comunica directamente con el Arduino Mega 2560 a través del puerto serie (/dev/ttyACM0).

**Características:**
- ✅ Control directo del hardware
- ✅ Lectura de encoders
- ✅ Publicación de odometría
- ✅ Publicación de joint_states
- ✅ Watchdog de seguridad

### Iniciar Hardware Real

```bash
cd ~/WS/ws_smart_trolley_ai_v2
source install/setup.bash

# Lanzar con ROS2
ros2 launch smart_t_ai_v2 launch_robot.launch.py

# O manualmente
python3 src/smart_t_ai_v2/scripts/arduino_bridge.py
```

### Controlar con Teclado

**Opción 1: Script de Menú (Más fácil)**
```bash
cd ~/WS/ws_smart_trolley_ai_v2
./test_simple.sh
```
- 1 = Avanzar
- 2 = Retroceder
- 3 = Girar izquierda
- 4 = Girar derecha
- 5 = Detener
- 0 = Salir

**Opción 2: Control Interactivo**
```bash
cd ~/WS/ws_smart_trolley_ai_v2
source install/setup.bash
python3 test_movement.py
```
- W = Avanzar
- S = Retroceder
- A = Girar izquierda
- D = Girar derecha
- SPACE = Detener
- Q = Salir

**Opción 3: Comandos Directos**
```bash
# Avanzar
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Girar
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Detener
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

### Verificar Estado

```bash
# Ver nodos activos
ros2 node list

# Ver topics
ros2 topic list

# Ver odometría
ros2 topic echo /odom

# Ver estados de las juntas
ros2 topic echo /joint_states

# Ver estado del Arduino
ros2 topic echo /arduino_status
```

### Parámetros Arduino Bridge

Archivo: `config/arduino_params.yaml`

```yaml
serial_port: /dev/ttyACM0    # Puerto serie del Arduino
baud_rate: 115200             # Velocidad de comunicación
wheel_separation: 0.48        # Separación entre ruedas (m)
wheel_radius: 0.10            # Radio de las ruedas (m)
encoder_ppr: 360              # Pulsos por revolución
max_linear_vel: 0.5           # Velocidad lineal máxima (m/s)
max_angular_vel: 1.0          # Velocidad angular máxima (rad/s)
cmd_vel_timeout: 1.0          # Timeout de comandos (s)
publish_rate: 50.0            # Frecuencia de publicación (Hz)
```

---

## SIMULACIÓN - ros2_control

### Configuración
Usa el framework ros2_control con diff_drive_controller para control más avanzado en simulación.

**Características:**
- ✅ Control PID de velocidad
- ✅ Límites de aceleración
- ✅ Odometría integrada
- ✅ Publicación automática de TF
- ✅ Compatible con MoveIt2 y Nav2

### Iniciar Simulación con ros2_control

```bash
cd ~/WS/ws_smart_trolley_ai_v2
source install/setup.bash

# Compilar (primera vez o después de cambios)
colcon build --symlink-install

# Lanzar simulación con ros2_control
ros2 launch smart_t_ai_v2 launch_sim_ros2_control.launch.py
```

### Controladores Disponibles

**1. Joint State Broadcaster**
- Publica estados de todas las juntas
- Topic: `/joint_states`

**2. Diff Drive Controller**
- Controla el movimiento diferencial
- Suscribe a: `/diff_drive_controller/cmd_vel_unstamped`
- Publica: `/diff_drive_controller/odom`, `/tf`

### Controlar en Simulación

**Con teleop_twist_keyboard:**
```bash
# En otra terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

**Con comandos directos:**
```bash
# Avanzar
ros2 topic pub --rate 10 /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Girar
ros2 topic pub --rate 10 /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
```

### Gestión de Controladores

```bash
# Listar controladores
ros2 control list_controllers

# Ver información de un controlador
ros2 control list_hardware_interfaces

# Activar/desactivar controlador
ros2 control set_controller_state diff_drive_controller start
ros2 control set_controller_state diff_drive_controller stop

# Recargar configuración
ros2 control load_controller --set-state start diff_drive_controller
```

### Parámetros ros2_control

Archivo: `config/ros2_controllers.yaml`

```yaml
diff_drive_controller:
  wheel_separation: 0.48
  wheel_radius: 0.10
  max_velocity: 0.5
  max_acceleration: 1.0
  publish_rate: 50.0
  odom_frame_id: odom
  base_frame_id: base_link
  enable_odom_tf: true
```

---

## SIMULACIÓN - Plugin Gazebo Nativo

Para simulaciones simples sin ros2_control.

### Iniciar Simulación Básica

```bash
cd ~/WS/ws_smart_trolley_ai_v2
source install/setup.bash

ros2 launch smart_t_ai_v2 launch_sim.launch.py
```

**Nota:** Este modo usa el plugin DiffDrive nativo de Gazebo, más simple pero menos configurable.

---

## Resolución de Problemas

### Hardware Real

**Arduino no conecta:**
```bash
# Verificar puerto
ls -l /dev/ttyACM*

# Verificar permisos
sudo usermod -a -G dialout $USER
# Logout/login para aplicar

# En WSL2, verificar USB passthrough
usbipd list
usbipd bind --busid 7-2
usbipd attach --wsl --busid 7-2
```

**Robot no se mueve:**
```bash
# Verificar que arduino_bridge está funcionando
ros2 node list | grep arduino

# Ver logs del bridge
ros2 topic echo /arduino_status

# Monitorear comandos recibidos
ros2 topic echo /cmd_vel
```

**Valores siempre 0.0:**
- Verificar que no hay múltiples suscriptores en /cmd_vel
- Reiniciar arduino_bridge
- Verificar QoS settings

### Simulación

**Controladores no cargan:**
```bash
# Verificar que el paquete está compilado
cd ~/WS/ws_smart_trolley_ai_v2
colcon build --packages-select smart_t_ai_v2

# Source el workspace
source install/setup.bash

# Verificar dependencias
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**Gazebo no encuentra el robot:**
```bash
# Verificar paths
echo $GZ_SIM_RESOURCE_PATH

# Verificar URDF
xacro src/smart_t_ai_v2/description/robot.urdf.xacro use_ros2_control:=true

# Check for errors
gz sim --verbose 4
```

---

## Arquitectura del Sistema

### Hardware Real
```
Usuario → /cmd_vel → arduino_bridge.py → Serial → Arduino Mega 2560 → Motores
                          ↓
                    /odom, /joint_states, /tf
```

### Simulación con ros2_control
```
Usuario → /cmd_vel → diff_drive_controller → ros2_control → Gazebo → Motores virtuales
                              ↓
                    /odom, /joint_states, /tf
```

### Simulación Básica
```
Usuario → /cmd_vel → gz_bridge → Gazebo DiffDrive Plugin → Motores virtuales
                                        ↓
                                  /odom (Gazebo)
```

---

## Referencias

- [ros2_control Documentation](https://control.ros.org/)
- [diff_drive_controller](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control)
- Arduino Firmware: MOTOR-INTERFACE-V-13

---

## Autores

- Jose Soto Villasmil
- Smart Trolley V5 - AI Version V2
- ROS2 Jazzy + Gazebo Harmonic
