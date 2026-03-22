# Smart Trolley V5 - ROS2 Control System

Sistema completo de control para el Smart Trolley V5 con soporte dual para **hardware real** (Arduino) y **simulación** (Gazebo).

![ROS2 Version](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![Python](https://img.shields.io/badge/Python-3.10+-green)

## 📋 Tabla de Contenidos

- [Características](#características)
- [Arquitectura del Sistema](#arquitectura-del-sistema)
- [Instalación](#instalación)
- [Uso Rápido](#uso-rápido)
- [Documentación Completa](#documentación-completa)
- [Hardware](#hardware)
- [Desarrollo](#desarrollo)

## ✨ Características

### 🤖 Dual Mode Operation

- **Hardware Real**: Control directo del robot físico con Arduino Mega 2560
- **Simulación**: Simulación completa en Gazebo con física realista

### 🎮 Múltiples Opciones de Control

- ✅ **ros2_control**: Framework avanzado con diff_drive_controller
- ✅ **Arduino Bridge**: Control directo del hardware real
- ✅ **Teleop Keyboard**: Control por teclado interactivo
- ✅ **Comandos ROS2**: Publicación directa a topics

### 📡 Características Técnicas

- Cinemática diferencial (2 ruedas motrices + 1 rueda loca)
- Encoders de 360 PPR para odometría precisa
- Publicación de TF (odom → base_link)
- Joint states en tiempo real
- Watchdog de seguridad para detención automática
- Límites configurables de velocidad y aceleración

## 🏗️ Arquitectura del Sistema

### Hardware Real (Arduino Bridge)
```
┌─────────────┐     /cmd_vel      ┌──────────────────┐     Serial     ┌─────────────┐
│   Usuario   │ ──────────────▶   │ arduino_bridge   │ ──────────▶   │  Arduino    │
│             │                    │   (Python)       │                 │  Mega 2560  │
└─────────────┘                    └──────────────────┘                 └─────────────┘
                                            │                                   │
                                            │                                   │
                                            ▼                                   ▼
                                   /odom, /joint_states               Motores + Encoders
                                          /tf
```

### Simulación (ros2_control)
```
┌─────────────┐    /cmd_vel    ┌──────────────────┐   velocity    ┌─────────────┐
│   Usuario   │ ─────────────▶ │ diff_drive_ctrl  │ ────────────▶ │  Gazebo Sim │
│             │                 │  (ros2_control)  │                │             │
└─────────────┘                 └──────────────────┘                └─────────────┘
                                         │                                  │
                                         │                                  │
                                         ▼                                  ▼
                                /odom, /joint_states              Physics Engine
                                       /tf
```

## 📦 Instalación

### Requisitos Previos

- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic
- Python 3.10+

### Dependencias ROS2

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-controller-manager \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard \
  python3-serial
```

### Clonar e Instalar

```bash
# Crear workspace
mkdir -p ~/WS/ws_smart_trolley_ai_v2/src
cd ~/WS/ws_smart_trolley_ai_v2/src

# Clonar el repositorio (ajustar según tu repo)
# git clone <tu-repo> smart_t_ai_v2

# Compilar
cd ~/WS/ws_smart_trolley_ai_v2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Source el workspace
source install/setup.bash
```

## 🚀 Uso Rápido

### Hardware Real (Robot Físico)

1. **Conectar Arduino**
   ```bash
   # En Windows (si usas WSL2)
   usbipd bind --busid 7-2
   usbipd attach --wsl --busid 7-2
   
   # Verificar conexión
   ls -l /dev/ttyACM0
   ```

2. **Iniciar Robot**
   ```bash
   cd ~/WS/ws_smart_trolley_ai_v2
   source install/setup.bash
   
   # Opción A: Lanzar con ROS2
   ros2 launch smart_t_ai_v2 launch_robot.launch.py
   
   # Opción B: Script simplificado
   ./start_robot.sh
   ```

3. **Controlar el Robot**
   
   **Método 1: Menú Interactivo (Más Fácil)**
   ```bash
   ./test_simple.sh
   ```
   
   **Método 2: Teclado Directo**
   ```bash
   python3 test_movement.py
   # W=adelante, S=atrás, A=izq, D=der, SPACE=stop
   ```
   
   **Método 3: Comandos ROS2**
   ```bash
   # Avanzar
   ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.3}}"
   
   # Detener
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
   ```

### Simulación (Gazebo)

#### Opción A: Con ros2_control (Recomendado)

```bash
cd ~/WS/ws_smart_trolley_ai_v2
source install/setup.bash

# Lanzar simulación con ros2_control
ros2 launch smart_t_ai_v2 launch_sim_ros2_control.launch.py
```

**Controlar en simulación:**
```bash
# En otra terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

#### Opción B: Simulación Básica

```bash
ros2 launch smart_t_ai_v2 launch_sim.launch.py
```

**Controlar:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 📚 Documentación Completa

Para información detallada sobre configuración, troubleshooting y parámetros avanzados, consulta:

- **[CONTROL_GUIDE.md](CONTROL_GUIDE.md)**: Guía completa de control y configuración
- **[HARDWARE_GUIDE.md](src/smart_t_ai_v2/HARDWARE_GUIDE.md)**: Especificaciones del hardware

## 🔧 Hardware

### Componentes Principales

- **Microcontrolador**: Arduino Mega 2560
- **Drivers de Motor**: ZS-X11H (Dual H-Bridge)
- **Motores**: DC con reductora, 30V @ 1A
- **Encoders**: Hall Effect, 360 PPR
- **Sensores**: 
  - LIDAR (opcional)
  - Cámara RGB (opcional)
- **Alimentación**: 30V para motores, 5V para sensores

### Dimensiones

- Separación entre ruedas: 0.48 m
- Radio de ruedas: 0.10 m
- Base: 0.5 m × 0.4 m
- Altura: 0.3 m

### Firmware Arduino

**Versión**: MOTOR-INTERFACE-V-13  
**Protocolo Serial**: 115200 baud  
**Puerto**: `/dev/ttyACM0` (Linux/WSL2)

**Comandos soportados:**
- `v <linear> <angular>`: Establecer velocidades (m/s, rad/s)
- `e`: Leer encoders
- `s`: Status del sistema

## 🛠️ Desarrollo

### Estructura del Proyecto

```
ws_smart_trolley_ai_v2/
├── src/
│   └── smart_t_ai_v2/
│       ├── config/                    # Archivos de configuración
│       │   ├── arduino_params.yaml    # Parámetros del Arduino bridge
│       │   └── ros2_controllers.yaml  # Configuración ros2_control
│       ├── description/               # URDF/Xacro
│       │   ├── robot.urdf.xacro       # Descripción principal
│       │   ├── ros2_control.xacro     # Configuración ros2_control
│       │   └── gazebo_control_trolley.xacro  # Plugin Gazebo
│       ├── launch/                    # Launch files
│       │   ├── launch_robot.launch.py # Hardware real
│       │   ├── launch_sim.launch.py   # Simulación básica
│       │   └── launch_sim_ros2_control.launch.py  # Simulación con ros2_control
│       ├── scripts/                   # Nodos Python
│       │   ├── arduino_bridge.py      # Bridge ROS2-Arduino
│       │   └── cmd_vel_inverter.py    # Inversor de velocidad
│       ├── meshes/                    # Modelos 3D
│       └── worlds/                    # Mundos de Gazebo
├── test_movement.py                   # Control por teclado
├── test_simple.sh                     # Script de menú
├── start_robot.sh                     # Inicio rápido
├── CONTROL_GUIDE.md                   # Guía completa
└── README.md                          # Este archivo
```

### Compilar Después de Cambios

```bash
cd ~/WS/ws_smart_trolley_ai_v2
colcon build --packages-select smart_t_ai_v2 --symlink-install
source install/setup.bash
```

### Verificar URDF

```bash
# Con ros2_control
xacro src/smart_t_ai_v2/description/robot.urdf.xacro use_ros2_control:=true

# Sin ros2_control
xacro src/smart_t_ai_v2/description/robot.urdf.xacro use_ros2_control:=false

# Verificar en RViz
ros2 launch smart_t_ai_v2 rsp_sim.launch.py
```

### Debugging

**Ver nodos activos:**
```bash
ros2 node list
```

**Monitorear topics:**
```bash
# Comandos de velocidad
ros2 topic echo /cmd_vel

# Odometría
ros2 topic echo /odom

# Estados de juntas
ros2 topic echo /joint_states
```

**Ver controladores (ros2_control):**
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

**Ver logs del Arduino Bridge:**
```bash
ros2 topic echo /arduino_status
```

## 🐛 Troubleshooting

### Arduino no conecta en WSL2

```bash
# Windows PowerShell (como administrador)
usbipd list
usbipd bind --busid 7-2
usbipd attach --wsl --busid 7-2

# En WSL2
ls -l /dev/ttyACM0
sudo chmod 666 /dev/ttyACM0
```

### Robot no se mueve

1. Verificar que arduino_bridge está corriendo:
   ```bash
   ros2 node list | grep arduino
   ```

2. Ver status del Arduino:
   ```bash
   ros2 topic echo /arduino_status
   ```

3. Monitorear comandos recibidos:
   ```bash
   ros2 topic echo /cmd_vel
   ```

4. Reiniciar el bridge:
   ```bash
   pkill -9 -f arduino_bridge
   python3 src/smart_t_ai_v2/scripts/arduino_bridge.py
   ```

### Errores de compilación

```bash
# Actualizar dependencias
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Limpiar build
rm -rf build install log
colcon build --symlink-install
```

## 📝 Licencia

Apache License 2.0

## 👥 Autores

- **Jose Soto Villasmil** - josemsotov@gmail.com
- Smart Trolley V5 - AI Version V2

## 🤝 Contribuciones

Las contribuciones son bienvenidas. Por favor:

1. Fork el proyecto
2. Crea una rama para tu feature (`git checkout -b feature/AmazingFeature`)
3. Commit tus cambios (`git commit -m 'Add some AmazingFeature'`)
4. Push a la rama (`git push origin feature/AmazingFeature`)
5. Abre un Pull Request

## 📞 Soporte

Para problemas, preguntas o sugerencias:

- 📧 Email: josemsotov@gmail.com
- 📖 Wiki: Ver [CONTROL_GUIDE.md](CONTROL_GUIDE.md)
- 🐛 Issues: Reportar en el repositorio

---

**Desarrollado con ROS2 Jazzy + Gazebo Harmonic** 🚀
