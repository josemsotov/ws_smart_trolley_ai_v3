# Test de Comunicación Bluetooth - Guía Rápida

## 🎯 Objetivo
Probar la comunicación Bluetooth con el Arduino **SIN usar USB** y **SIN ROS**.

---

## 📋 Pre-requisitos

1. **Módulo Bluetooth emparejado y conectado:**
   ```bash
   python3 src/smart_t_ai_v2/scripts/bluetooth_helper.py scan
   python3 src/smart_t_ai_v2/scripts/bluetooth_helper.py connect HC-05
   ```

2. **Puerto RFCOMM creado:**
   ```bash
   ls -l /dev/rfcomm0
   # Debería existir
   ```

---

## 🚀 Métodos de Prueba

### Método 1: Test Rápido (Bash Script)

El más simple y rápido:

```bash
cd /home/josemsotov/WS/WorkSpace/ws_smart_trolley_ai_v2

# Con puerto por defecto (/dev/rfcomm0 @ 115200)
./quick_bt_test.sh

# Con puerto y baud rate específicos
./quick_bt_test.sh /dev/rfcomm0 38400
```

**Qué hace:**
- ✅ Verifica que el puerto existe
- ✅ Comprueba permisos
- ✅ Lee datos del Arduino
- ✅ Envía comandos y espera respuesta
- ✅ Test con Python (pyserial)

---

### Método 2: Test Completo (Python)

Más detallado con múltiples tests:

```bash
cd /home/josemsotov/WS/WorkSpace/ws_smart_trolley_ai_v2

# Tests automáticos completos
python3 test_bluetooth_arduino.py /dev/rfcomm0

# Con baud rate diferente
python3 test_bluetooth_arduino.py /dev/rfcomm0 --baud 38400

# Modo interactivo (enviar comandos manualmente)
python3 test_bluetooth_arduino.py /dev/rfcomm0 --interactive

# Solo lectura continua (30 segundos)
python3 test_bluetooth_arduino.py /dev/rfcomm0 --continuous 30
```

**Tests automáticos incluyen:**
1. **Test Echo/Ping** - Envía comandos básicos (?, STATUS, INFO)
2. **Test Encoders** - Lee valores de encoders
3. **Test Motores** - Envía comando de velocidad 0 (seguro)
4. **Test Lectura Continua** - Lee todos los datos por 5 segundos

---

### Método 3: Modo Interactivo

Para enviar comandos manualmente y ver respuestas:

```bash
python3 test_bluetooth_arduino.py /dev/rfcomm0 -i
```

**Comandos especiales en modo interactivo:**
- `read` - Leer datos durante 5 segundos
- `exit` - Salir

**Comandos típicos del Arduino MOTOR-INTERFACE-V-13:**
- `?` - Info/status
- `E` - Leer encoders
- `M 0 0` - Motores a velocidad 0
- `STATUS` - Estado del sistema
- `V` - Versión del firmware

---

## 🔍 Verificaciones Paso a Paso

### 1. Verificar Conexión Bluetooth

```bash
# Ver dispositivos Bluetooth emparejados
bluetoothctl devices

# Ver info de tu HC-05
bluetoothctl info <MAC_ADDRESS>

# Estado de la conexión
bluetoothctl info <MAC_ADDRESS> | grep Connected
# Debería mostrar: Connected: yes
```

### 2. Verificar Puerto RFCOMM

```bash
# Listar puertos RFCOMM
sudo rfcomm -a

# Ver detalles del puerto
ls -l /dev/rfcomm0

# Si no existe, crearlo:
sudo rfcomm bind /dev/rfcomm0 <MAC_ADDRESS> 1
sudo chmod 666 /dev/rfcomm0
```

### 3. Test Manual con Cat

```bash
# Leer datos del Arduino (Ctrl+C para salir)
cat /dev/rfcomm0

# Si ves basura/caracteres raros = baud rate incorrecto
# Prueba con stty:
stty -F /dev/rfcomm0 38400 raw -echo
cat /dev/rfcomm0
```

### 4. Test Manual con Echo

```bash
# Configurar puerto
stty -F /dev/rfcomm0 115200 raw -echo

# Enviar comando y leer respuesta
echo "?" > /dev/rfcomm0 & timeout 2s cat /dev/rfcomm0
```

---

## 🐛 Troubleshooting

### ❌ "Puerto no existe"
```bash
# Verificar conexión Bluetooth
bluetoothctl info <MAC_ADDRESS>

# Reconectar
bluetoothctl connect <MAC_ADDRESS>

# Recrear puerto RFCOMM
sudo rfcomm release /dev/rfcomm0
sudo rfcomm bind /dev/rfcomm0 <MAC_ADDRESS> 1
sudo chmod 666 /dev/rfcomm0
```

### ❌ "Permission denied"
```bash
# Dar permisos
sudo chmod 666 /dev/rfcomm0

# O añadir usuario al grupo dialout
sudo usermod -a -G dialout $USER
# Cerrar sesión y volver a entrar
```

### ❌ "Sin respuesta del Arduino"
```bash
# Verificar baud rate (probar común HC-05: 38400)
python3 test_bluetooth_arduino.py /dev/rfcomm0 --baud 38400

# Verificar que Arduino está encendido y módulo BT con LED activo

# Test de lectura continua (ver si llegan datos)
python3 test_bluetooth_arduino.py /dev/rfcomm0 --continuous 10
```

### ❌ "Caracteres basura/ilegibles"
**Problema:** Baud rate incorrecto

**Solución:** Probar diferentes baud rates:
```bash
# Común HC-05
python3 test_bluetooth_arduino.py /dev/rfcomm0 --baud 38400

# Común HC-06
python3 test_bluetooth_arduino.py /dev/rfcomm0 --baud 9600

# Si tienes Arduino configurado a 115200
python3 test_bluetooth_arduino.py /dev/rfcomm0 --baud 115200
```

---

## 📊 Interpretación de Resultados

### ✅ Comunicación Exitosa

```
✅ Conectado exitosamente a /dev/rfcomm0
📤 Enviado: ?
📥 Recibido: MOTOR-INTERFACE-V-13 OK
📥 Recibido: L:1234 R:5678
✅ Respuesta recibida (2 líneas)
```

### ⚠️ Conexión pero sin respuesta

```
✅ Conectado exitosamente a /dev/rfcomm0
📤 Enviado: ?
⚠️  Sin respuesta
```

**Causas posibles:**
- Arduino no envía respuesta a ese comando
- Baud rate incorrecto
- Firmware diferente

### ❌ Error de conexión

```
❌ Error al conectar: [Errno 2] No such file or directory: '/dev/rfcomm0'
```

**Solución:** Puerto no existe, crear con `bluetooth_helper.py`

---

## 🔄 Workflow Completo de Prueba

```bash
cd /home/josemsotov/WS/WorkSpace/ws_smart_trolley_ai_v2

# 1. Escanear y conectar Bluetooth
python3 src/smart_t_ai_v2/scripts/bluetooth_helper.py scan
python3 src/smart_t_ai_v2/scripts/bluetooth_helper.py connect HC-05

# 2. Test rápido
./quick_bt_test.sh

# 3. Si funciona, test completo
python3 test_bluetooth_arduino.py /dev/rfcomm0

# 4. Modo interactivo para explorar comandos
python3 test_bluetooth_arduino.py /dev/rfcomm0 -i
> ?
> STATUS
> E
> exit

# 5. Si todo OK, usar con ROS
source install/setup.bash
ros2 launch smart_t_ai_v2 launch_robot.launch.py \
    connection_type:=bluetooth \
    serial_port:=/dev/rfcomm0
```

---

## 📝 Comparación USB vs Bluetooth

| Aspecto | USB (/dev/ttyACM0) | Bluetooth (/dev/rfcomm0) |
|---------|-------------------|--------------------------|
| **Conexión** | Cable físico | Inalámbrico (10m) |
| **Latencia** | ~10ms | ~50-100ms |
| **Velocidad** | Hasta 12 Mbps | Hasta 3 Mbps |
| **Confiabilidad** | Muy alta | Buena |
| **Setup** | Plug & play | Requiere emparejamiento |
| **Alcance** | ~2m (cable) | ~10m (aire libre) |
| **Baud rate típico** | 115200 | 38400 - 115200 |

---

## ✅ Checklist de Verificación

- [ ] Módulo Bluetooth encendido (LED parpadeando/fijo)
- [ ] Dispositivo emparejado (`bluetoothctl devices`)
- [ ] Dispositivo conectado (`bluetoothctl info | grep Connected: yes`)
- [ ] Puerto RFCOMM creado (`ls /dev/rfcomm0`)
- [ ] Permisos correctos (`ls -l /dev/rfcomm0` → rw-rw-rw-)
- [ ] Test rápido exitoso (`./quick_bt_test.sh`)
- [ ] Test Python exitoso (`python3 test_bluetooth_arduino.py /dev/rfcomm0`)
- [ ] Comandos interactivos funcionan
- [ ] Baud rate correcto identificado

---

## 🎓 Tips Avanzados

### Monitoreo en Tiempo Real

```bash
# Terminal 1: Monitor de datos
watch -n 1 'echo "E" > /dev/rfcomm0; timeout 0.5s cat /dev/rfcomm0'

# Terminal 2: Test continuo
python3 test_bluetooth_arduino.py /dev/rfcomm0 --continuous 60
```

### Logging de Comunicación

```bash
# Guardar toda la comunicación en archivo
python3 test_bluetooth_arduino.py /dev/rfcomm0 --continuous 30 | tee bt_log.txt
```

### Comparar USB vs Bluetooth

```bash
# Terminal 1: Test USB
python3 test_bluetooth_arduino.py /dev/ttyACM0

# Terminal 2: Test Bluetooth
python3 test_bluetooth_arduino.py /dev/rfcomm0

# Comparar latencia y respuestas
```

---

**¿Todo funcionó?** 🎉

Ahora puedes usar el robot completamente inalámbrico con:

```bash
ros2 launch smart_t_ai_v2 person_follower.launch.py \
    use_arduino:=true \
    connection_type:=bluetooth \
    serial_port:=/dev/rfcomm0
```
