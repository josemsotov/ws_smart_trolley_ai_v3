#!/usr/bin/env python3
"""Test directo de comunicación con Arduino"""

import serial
import time

# Configuración
PORT = '/dev/ttyACM0'
BAUD = 115200

print("=" * 60)
print("PRUEBA DIRECTA DE ARDUINO")
print("=" * 60)
print()

try:
    # Abrir puerto serial
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # Esperar inicialización
    print(f"✓ Conectado a {PORT} @ {BAUD} baud")
    print()
    
    # Limpiar buffer
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    # 1. Test de status
    print("[1/3] Probando comando 's' (status)...")
    ser.write(b's\n')
    time.sleep(0.5)
    response = ser.read_all().decode('utf-8', errors='ignore')
    if response:
        print(f"  Respuesta: {response.strip()}")
    else:
        print("  ⚠️ Sin respuesta")
    print()
    
    # 2. Test de velocidad
    print("[2/3] Probando comando 'v 0.3 0' (avanzar)...")
    ser.write(b'v 0.3 0\n')
    time.sleep(2)
    print("  ✓ Comando enviado - ¿Se movió el robot?")
    print()
    
    # 3. Detener
    print("[3/3] Deteniendo 'v 0 0'...")
    ser.write(b'v 0 0\n')
    time.sleep(0.5)
    print("  ✓ Robot detenido")
    print()
    
    # Leer encoders
    print("[4/4] Leyendo encoders 'e'...")
    ser.write(b'e\n')
    time.sleep(0.5)
    response = ser.read_all().decode('utf-8', errors='ignore')
    if response:
        print(f"  Encoders: {response.strip()}")
    else:
        print("  ⚠️ Sin respuesta de encoders")
    
    ser.close()
    print()
    print("=" * 60)
    print("RESULTADO:")
    print("• Si viste respuestas = Arduino funciona")
    print("• Si el robot NO se movió = Revisar:")
    print("  1. Alimentación de motores (12V conectado?)")
    print("  2. ZS-X11H drivers conectados")
    print("  3. Cables de motor conectados")
    print("=" * 60)
    
except serial.SerialException as e:
    print(f"✗ Error: {e}")
    print("  - ¿Arduino conectado?")
    print("  - Ejecuta: ls -l /dev/ttyACM*")
except Exception as e:
    print(f"✗ Error inesperado: {e}")
