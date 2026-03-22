# CONECTAR CONTROL DJI A WSL2

## 📋 PASOS

### 1. Identificar el control en Windows
Abre PowerShell como Administrador:
```powershell
usbipd list
```

Busca líneas que contengan:
- "DJI"
- "Remote Controller"
- "DJIGO" (app DJI)
- Vendor ID común: 0x2CA3 (DJI)

Ejemplo:
```
BUSID  VID:PID    DEVICE
2-1    2ca3:001f  DJI Remote Controller
```

### 2. Conectar a WSL2
```powershell
# Si es la primera vez
usbipd bind --busid 2-1

# Conectar a WSL
usbipd attach --wsl --busid 2-1
```

### 3. Verificar en Linux
```bash
# Ver devices
ls -l /dev/input/event*

# Ver nombres
for i in /dev/input/event*; do
    echo "$i → $(cat /sys/class/input/$(basename $i)/device/name 2>/dev/null)"
done

# Test rápido
sudo evtest /dev/input/event0
# Mueve los sticks - deberías ver "Event time..."
```

## 🎮 CONTROLES DJI COMUNES

### DJI Mavic series (RC, RC Pro, RC-N1)
- Conexión: USB-C
- Sticks: Hall effect (muy precisos)
- Botones: C1, C2, Record, Shutter
- Wheels: Dial/Gimbal

### DJI Mini RC
- Conexión: USB-C
- Sticks: 2 analógicos
- Simple pero funcional

### DJI Smart Controller
- Conexión: USB-C
- Android integrado
- Muchos botones

## 💡 VENTAJAS vs Control Genérico
✅ USB directo (sin problemas de pairing)
✅ Drivers Linux estándar
✅ Sticks Hall effect (sin drift)
✅ Calibración profesional
✅ Botones programables

## 🚨 POSIBLES ISSUES
⚠️ Algunos modelos requieren "activación" en la app DJI primero
⚠️ Si tiene pantalla Android, puede necesitar modo MTP deshabilitado
