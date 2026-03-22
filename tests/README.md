# Tests — Smart Trolley AI v2

Scripts de prueba standalone (NO son nodos ROS2). Se ejecutan directamente con Python o bash.

## Hardware Tests

| Script | Descripción | Puerto |
|--------|-------------|--------|
| `test_arduino_direct.py` | Test básico serial Arduino | `/dev/ttyACM0` |
| `test_bluetooth_arduino.py` | Test conexión Bluetooth HC-05 | `/dev/rfcomm0` |
| `test_camera.py` | Test cámara USB/Kinect | `/dev/video0` |
| `test_movement.py` | Test movimiento básico (requiere Arduino) | serial |
| `test_pid_heading.py` | Test PID de heading con IMU | serial |
| `test_5m.py` | Test recorrido 5 metros recto | serial |
| `test_5m_rotate.py` | Test recorrido 5 metros + rotación | serial |
| `test_joy_buttons.py` | Test botones del joystick Stadia | `/dev/input/` |
| `test_stadia_calibration.py` | Calibración ejes del Stadia | `/dev/input/` |

## Connectivity Tests

| Script | Descripción |
|--------|-------------|
| `test_joystick_pairing.sh` | Test emparejamiento BT joystick |
| `test_simple.sh` | Test rápido sistema básico |
| `test_system.sh` | Test completo del sistema |

## Uso

```bash
# Test hardware sin ROS
python3 tests/test_arduino_direct.py

# Test con ROS activo
source install/setup.bash
python3 tests/test_movement.py
```
