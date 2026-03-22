# Guía: Conectar Arduino a WSL2 usando usbipd-win

Esta guía te ayudará a conectar tu Arduino Mega 2560 desde Windows a WSL2 para usar ROS2 con el hardware real.

## 📋 Requisitos Previos

- Windows 10/11 con WSL2 instalado ✓ (ya lo tienes)
- Arduino Mega 2560
- Cable USB
- Permisos de Administrador en Windows

---

## 🔧 PASO 1: Instalar usbipd-win en Windows

### Opción A: Usando winget (Recomendado)

1. **Abre PowerShell como Administrador**:
   - Presiona `Windows + X`
   - Selecciona "Terminal (Admin)" o "Windows PowerShell (Admin)"
   - Si aparece UAC, da clic en "Sí"

2. **Instala usbipd-win**:
   ```powershell
   winget install --interactive --exact dorssel.usbipd-win
   ```

3. **Espera a que termine la instalación** (puede tardar 1-2 minutos)

4. **Cierra y vuelve a abrir PowerShell** (como Administrador) para cargar el comando

### Opción B: Instalador MSI (si winget no funciona)

1. Descarga desde: https://github.com/dorssel/usbipd-win/releases/latest
2. Busca el archivo `usbipd-win_X.X.X.msi` y descárgalo
3. Ejecuta el instalador (doble clic)
4. Sigue el asistente de instalación
5. Reinicia PowerShell

---

## 🔌 PASO 2: Conectar Arduino

### 2.1 Conectar físicamente

1. **Conecta tu Arduino Mega 2560** al puerto USB de tu PC
2. Espera a que Windows lo reconozca (puede instalar drivers automáticamente)

### 2.2 Verificar en Windows

En **PowerShell (como Administrador)**:

```powershell
usbipd list
```

**Ejemplo de salida**:
```
BUSID  VID:PID    DEVICE                                     STATE
1-1    2341:0042  Arduino Mega 2560 (COM3)                   Not attached
2-3    046d:c52b  Logitech USB Input Device                  Not attached
3-2    8087:0026  Intel(R) Wireless Bluetooth(R)             Not attached
```

**Busca tu Arduino**:
- VID:PID `2341:0042` o `2341:0001` = Arduino Mega 2560
- Anota el **BUSID** (ejemplo: `1-1`)
- Puede aparecer como "USB Serial Device" o "Arduino Mega 2560"

---

## 🔗 PASO 3: Vincular y Conectar a WSL2

### 3.1 Bind (solo la primera vez)

En **PowerShell (como Administrador)**, reemplaza `X-Y` con tu BUSID:

```powershell
usbipd bind --busid X-Y
```

**Ejemplo**:
```powershell
usbipd bind --busid 1-1
```

**Salida esperada**: "Device bound successfully" o similar

### 3.2 Attach (cada vez que conectes el Arduino)

```powershell
usbipd attach --wsl --busid X-Y
```

**Ejemplo**:
```powershell
usbipd attach --wsl 1-1
```

**Salida esperada**: "Device attached to WSL" o similar

⚠️ **IMPORTANTE**: Este comando `attach` debes ejecutarlo **cada vez que**:
- Conectes el Arduino
- Reinicies Windows
- Reinicies WSL2

---

## ✅ PASO 4: Verificar en WSL2

### 4.1 Verificar puerto serial

Abre tu terminal WSL2 y ejecuta:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

**Salida esperada**:
```
crw-rw---- 1 root dialout 166, 0 Feb 15 10:30 /dev/ttyACM0
```

### 4.2 Verificar permisos

```bash
groups | grep dialout
```

Si NO aparece "dialout", ejecuta:
```bash
sudo usermod -a -G dialout $USER
# Luego cierra TODAS las terminales WSL y abre una nueva
```

### 4.3 Probar conexión con Arduino

```bash
cd ~/WS/ws_smart_trolley_ai_v2
python3 src/smart_t_ai_v2/scripts/test_arduino_connection.py
```

**Si todo funciona verás**:
```
========================================
SMART TROLLEY - Arduino Connection Test
========================================

✓ Connection established!

Test 1: Sending status command 's'...
Test 2: Reading encoders 'e'...
  ✓ Left encoder: 0, Right encoder: 0
...

✓ All tests completed successfully!
```

---

## 🚀 PASO 5: Usar ROS2 con el Robot Real

### 5.1 Lanzar el robot

```bash
setup_smart  # tu alias personalizado
ros2 launch smart_t_ai_v2 launch_robot.launch.py
```

Si el puerto es diferente a `/dev/ttyACM0`:
```bash
ros2 launch smart_t_ai_v2 launch_robot.launch.py serial_port:=/dev/ttyUSB0
```

### 5.2 Controlar con teclado

En otra terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 5.3 Monitorear tópicos

```bash
# Ver odometría
ros2 topic echo /odom

# Ver comandos de velocidad
ros2 topic echo /cmd_vel

# Ver estado del Arduino
ros2 topic echo /arduino_status
```

---

## 🔄 Comandos de Rutina

### Cada vez que conectes el Arduino:

**En PowerShell (Windows, como Admin)**:
```powershell
usbipd attach --wsl --busid X-Y
```

### Para desconectar el Arduino de WSL2:

**En PowerShell (Windows)**:
```powershell
usbipd detach --busid X-Y
```

Ahora Windows podrá usarlo (Arduino IDE, etc.)

### Ver estado de dispositivos:

**En PowerShell**:
```powershell
usbipd list
```

Muestra qué dispositivos están:
- `Not attached` = En Windows
- `Attached` = En WSL2

---

## ❌ Solución de Problemas

### Problema: "usbipd: command not found"

**Causa**: usbipd-win no instalado o PowerShell no reiniciado

**Solución**:
1. Cierra PowerShell
2. Abre nueva PowerShell como Administrador
3. Si persiste, reinstala usbipd-win

### Problema: "Access denied" al hacer bind/attach

**Causa**: PowerShell sin permisos de Administrador

**Solución**: Abre PowerShell como Administrador (Windows + X → Terminal (Admin))

### Problema: Arduino no aparece en "usbipd list"

**Causa**: Arduino no conectado o drivers no instalados

**Solución**:
1. Desconecta y reconecta el Arduino
2. Verifica en "Administrador de dispositivos" (Windows)
3. Instala Arduino IDE en Windows (instala drivers automáticamente)

### Problema: "/dev/ttyACM0" no aparece en WSL2

**Causa**: No ejecutaste `attach` o Arduino desconectado

**Solución**:
1. Verifica en PowerShell: `usbipd list` (debe decir "Attached")
2. Si dice "Not attached", ejecuta: `usbipd attach --wsl --busid X-Y`
3. Verifica cable USB y conexión física

### Problema: "Permission denied" al abrir puerto serial

**Causa**: Usuario no en grupo dialout

**Solución**:
```bash
sudo usermod -a -G dialout $USER
# Cierra TODAS las terminales WSL (escribe exit)
# Abre nueva terminal WSL
groups | grep dialout  # verificar
```

### Problema: Arduino se desconecta de WSL2 aleatoriamente

**Causa**: Suspensión de energía USB o desconexión física

**Solución**:
1. Verifica cable USB (prueba otro cable)
2. Usa puerto USB 2.0 (más estable que USB 3.0)
3. En PowerShell: `usbipd attach --wsl --busid X-Y` de nuevo

### Problema: "Device busy" al hacer attach

**Causa**: Otro programa usa el Arduino (Arduino IDE, PuTTY, etc.)

**Solución**:
1. Cierra Arduino IDE, Serial Monitor, PuTTY, etc. en Windows
2. En WSL2, verifica: `lsof /dev/ttyACM*`
3. Si hay proceso, ciérralo: `sudo pkill -f arduino_bridge`

---

## 📖 Comandos de Referencia Rápida

```bash
# ============================================
# EN WINDOWS POWERSHELL (como Administrador)
# ============================================

# Ver dispositivos USB
usbipd list

# Bind (solo primera vez)
usbipd bind --busid 1-1

# Attach a WSL2 (cada conexión)
usbipd attach --wsl --busid 1-1

# Detach de WSL2
usbipd detach --busid 1-1

# ============================================
# EN WSL2 (Ubuntu)
# ============================================

# Ver puertos seriales
ls -l /dev/ttyACM* /dev/ttyUSB*

# Probar conexión Arduino
cd ~/WS/ws_smart_trolley_ai_v2
python3 src/smart_t_ai_v2/scripts/test_arduino_connection.py

# Lanzar robot real
setup_smart
ros2 launch smart_t_ai_v2 launch_robot.launch.py

# Controlar robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Ver odometría
ros2 topic echo /odom

# Monitorear estado Arduino
ros2 topic echo /arduino_status
```

---

## 📚 Referencias

- **Documentación oficial Microsoft**: https://learn.microsoft.com/en-us/windows/wsl/connect-usb
- **usbipd-win GitHub**: https://github.com/dorssel/usbipd-win
- **HARDWARE_GUIDE.md**: En este mismo directorio, información del sistema ROS2

---

## 💡 Consejos

1. **Mantén PowerShell Admin abierto** mientras trabajas con el Arduino
2. **Usa autocompletar**: Después de escribir `usbipd attach --wsl --busid ` presiona Tab
3. **Script de conexión rápida**: Puedes crear un `.bat` en Windows para automatizar
4. **Puerto USB estable**: Usa siempre el mismo puerto USB para mantener el mismo BUSID
5. **Cable de calidad**: Los cables baratos causan desconexiones intermitentes

---

## ✅ Checklist de Primera Configuración

- [ ] usbipd-win instalado en Windows
- [ ] PowerShell ejecutado como Administrador
- [ ] Arduino conectado y visible en `usbipd list`
- [ ] `usbipd bind` ejecutado (solo una vez)
- [ ] `usbipd attach --wsl` ejecutado
- [ ] `/dev/ttyACM0` visible en WSL2
- [ ] Usuario en grupo dialout (verificado con `groups`)
- [ ] Test de conexión exitoso (`test_arduino_connection.py`)
- [ ] launch_robot.launch.py funcional
- [ ] teleop_twist_keyboard controlando el robot

---

¡Ahora estás listo para usar tu Arduino con ROS2 en WSL2! 🎉
