# Guía de Configuración Bluetooth - Smart Trolley

## Conexión Arduino vía Bluetooth (HC-05/HC-06)

Esta guía te ayuda a configurar la conexión inalámbrica entre tu PC y el Arduino usando un módulo Bluetooth.

---

## 📋 Requisitos

### Hardware
- ✅ Módulo Bluetooth HC-05, HC-06 u otro compatible
- ✅ Arduino con módulo Bluetooth conectado
- ✅ Adaptador Bluetooth en el PC (integrado o USB)

### Software (Linux)
```bash
sudo apt update
sudo apt install -y bluetooth bluez bluez-tools rfcomm
```

---

## 🔧 Configuración del Módulo Bluetooth

### Paso 1: Verificar Bluetooth del PC

```bash
# Verificar que el adaptador está activo
hciconfig

# Debería mostrar algo como:
# hci0:    Type: Primary  Bus: USB
#          BD Address: XX:XX:XX:XX:XX:XX  ACL MTU: 1021:8  SCO MTU: 64:1
#          UP RUNNING
```

Si no está activo:
```bash
sudo hciconfig hci0 up
sudo systemctl start bluetooth
sudo systemctl enable bluetooth
```

### Paso 2: Escanear Dispositivos Bluetooth

**Opción A: Usando el script helper**
```bash
cd /home/josemsotov/WS/WorkSpace/ws_smart_trolley_ai_v2
source install/setup.bash
python3 src/smart_t_ai_v2/scripts/bluetooth_helper.py scan
```

**Opción B: Manualmente con bluetoothctl**
```bash
bluetoothctl
> power on
> scan on
# Esperar 10 segundos
# Buscar tu dispositivo (HC-05, HC-06, etc.)
> scan off
> devices
# Anota la dirección MAC del módulo (ej: 00:14:03:06:7D:9E)
```

### Paso 3: Emparejar el Módulo

**Opción A: Usando el script helper**
```bash
python3 src/smart_t_ai_v2/scripts/bluetooth_helper.py connect HC-05
# O con dirección MAC directa:
python3 src/smart_t_ai_v2/scripts/bluetooth_helper.py connect 00:14:03:06:7D:9E
```

**Opción B: Manualmente con bluetoothctl**
```bash
bluetoothctl
> pair 00:14:03:06:7D:9E
# Si pide PIN, usar: 1234 o 0000 (default HC-05/HC-06)
> trust 00:14:03:06:7D:9E
> connect 00:14:03:06:7D:9E
> exit
```

### Paso 4: Crear Puerto Serial RFCOMM

```bash
# Opción A: Automático (con helper)
python3 src/smart_t_ai_v2/scripts/bluetooth_helper.py connect HC-05

# Opción B: Manual
sudo rfcomm bind /dev/rfcomm0 00:14:03:06:7D:9E 1
sudo chmod 666 /dev/rfcomm0

# Verificar
ls -l /dev/rfcomm0
# Debería existir el dispositivo
```

---

## ⚙️ Configurar ROS2 para Usar Bluetooth

### Opción 1: Archivo de Configuración (Recomendado)

Editar: `src/smart_t_ai_v2/config/arduino_bluetooth.yaml`

```yaml
arduino_bridge:
  ros__parameters:
    connection_type: 'bluetooth'      # Cambiar de 'usb' a 'bluetooth'
    
    # Bluetooth config
    bluetooth_device_name: 'HC-05'    # Tu nombre de módulo
    bluetooth_address: ''             # O dirección MAC directa
    bluetooth_port: '/dev/rfcomm0'
    bluetooth_channel: 1
    baud_rate: 38400                  # HC-05 default (puede ser 9600, 38400, 115200)
    
    auto_reconnect: true
    reconnect_interval: 5.0
```

### Opción 2: Parámetros en Launch

```bash
ros2 launch smart_t_ai_v2 launch_robot.launch.py \
    connection_type:=bluetooth \
    serial_port:=/dev/rfcomm0 \
    baud_rate:=38400
```

### Opción 3: Parámetros en Línea de Comando

```bash
ros2 run smart_t_ai_v2 arduino_bridge.py \
    --ros-args \
    -p connection_type:=bluetooth \
    -p serial_port:=/dev/rfcomm0 \
    -p baud_rate:=38400
```

---

## 🚀 Lanzar con Bluetooth

### Para Person Follower con Bluetooth:

```bash
cd /home/josemsotov/WS/WorkSpace/ws_smart_trolley_ai_v2
source install/setup.bash

# Lanzar con Arduino vía Bluetooth
ros2 launch smart_t_ai_v2 person_follower.launch.py \
    use_arduino:=true \
    connection_type:=bluetooth \
    serial_port:=/dev/rfcomm0
```

### Para Control Completo del Robot:

```bash
ros2 launch smart_t_ai_v2 launch_robot.launch.py \
    connection_type:=bluetooth \
    serial_port:=/dev/rfcomm0 \
    baud_rate:=38400
```

---

## 🔍 Verificación y Troubleshooting

### Verificar Conexión

```bash
# Ver dispositivos emparejados
bluetoothctl devices

# Ver estado de conexión
bluetoothctl info 00:14:03:06:7D:9E

# Ver puertos RFCOMM activos
sudo rfcomm -a

# Probar comunicación serial
sudo cat /dev/rfcomm0
# Debería mostrar datos del Arduino
```

### Problemas Comunes

#### ❌ "Device not found"
```bash
# Asegurarse de que el módulo está encendido
# Escanear de nuevo
bluetoothctl
> scan on
# Esperar 15 segundos
> scan off
```

#### ❌ "Connection refused"
```bash
# Verificar que el módulo acepta conexiones
# PIN incorrecto: probar 0000, 1234, 1111
bluetoothctl
> remove 00:14:03:06:7D:9E
> pair 00:14:03:06:7D:9E
# Ingresar PIN cuando se solicite
```

#### ❌ "/dev/rfcomm0 no existe"
```bash
# Crear manualmente
sudo rfcomm bind /dev/rfcomm0 00:14:03:06:7D:9E 1
sudo chmod 666 /dev/rfcomm0
```

#### ❌ "Permission denied"
```bash
# Dar permisos al usuario
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/rfcomm0
# Cerrar sesión y volver a entrar
```

#### ❌ "Baud rate mismatch"
```bash
# El módulo HC-05/HC-06 puede tener diferentes baud rates
# Probar: 9600, 38400, 57600, 115200
# Configurar en arduino_bluetooth.yaml o como parámetro
```

---

## 📊 Verificar Funcionamiento

Una vez conectado, verificar tópicos ROS:

```bash
# Ver si publica odometría
ros2 topic echo /odom

# Ver estado de Arduino
ros2 topic echo /arduino_status

# Enviar comando de prueba
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

---

## 🔄 Reconexión Automática

El arduino_bridge tiene reconexión automática si se configura:

```yaml
auto_reconnect: true
reconnect_interval: 5.0  # segundos entre intentos
```

Si se pierde la conexión Bluetooth, intentará reconectar cada 5 segundos.

---

## ⚡ Rango y Limitaciones

### HC-05 / HC-06 Bluetooth 2.0
- **Rango:** ~10 metros en espacio abierto
- **Latencia:** ~50-100ms
- **Velocidad:** Hasta 115200 baud (típico: 38400)

### Consejos:
- Mantener línea de visión entre módulos
- Evitar obstáculos metálicos
- Distancia óptima: 1-5 metros
- Usar baud rate estable (38400 recomendado)

---

## 🛠️ Configuración Avanzada del HC-05

Si necesitas cambiar configuración del HC-05 (baud rate, PIN, nombre):

```bash
# Entrar en modo AT del HC-05
# 1. Desconectar Arduino
# 2. Mantener presionado botón en HC-05
# 3. Conectar alimentación
# 4. Soltar botón (LED parpadeará lento)

# Conectar serial a 38400 baud
screen /dev/ttyUSB0 38400

# Comandos AT:
AT              # Debe responder OK
AT+NAME=MyRobot # Cambiar nombre
AT+PSWD=1234    # Cambiar PIN
AT+UART=115200,0,0  # Cambiar baud rate
AT+ROLE=0       # Modo esclavo
```

---

## 📚 Referencias

- [HC-05 Datasheet](https://components101.com/wireless/hc-05-bluetooth-module)
- [Linux Bluetooth Guide](https://wiki.archlinux.org/title/Bluetooth)
- [RFCOMM Manual](https://man.archlinux.org/man/rfcomm.1)

---

## ✅ Checklist Rápido

- [ ] Bluetooth del PC activo
- [ ] Módulo HC-05/HC-06 visible en scan
- [ ] Dispositivo emparejado (paired)
- [ ] Dispositivo confiado (trusted)
- [ ] Dispositivo conectado
- [ ] Puerto /dev/rfcomm0 creado
- [ ] Permisos correctos (666)
- [ ] Baud rate correcto configurado
- [ ] arduino_bridge lanzado con connection_type:=bluetooth

---

**¿Problemas?** Revisa los logs de ROS:
```bash
ros2 run smart_t_ai_v2 arduino_bridge.py --ros-args --log-level DEBUG
```
