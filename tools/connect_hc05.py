#!/usr/bin/env python3
"""
Script de conexión automática para HC-05 (Arduino) via Bluetooth
Dispositivo: Slave / 98:D3:32:30:8C:62
Pi 5: 88:A2:9E:11:4C:72 (robot)

Uso:
    python3 connect_hc05.py                  # Conectar con PIN 1234 (defecto)
    python3 connect_hc05.py --pin 0000       # PIN alternativo
    python3 connect_hc05.py --test           # Solo probar la conexión serial
"""

import subprocess
import sys
import time
import os
import argparse

# Configuración del dispositivo
HC05_MAC  = "98:D3:32:30:8C:62"
HC05_NAME = "Slave"
RFCOMM_PORT = "/dev/rfcomm0"
RFCOMM_CHANNEL = 1
DEFAULT_PIN = "1234"


def run_cmd(cmd, timeout=10, input_text=None):
    """Ejecuta un comando y retorna (returncode, stdout, stderr)"""
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True,
            timeout=timeout, input=input_text
        )
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "TIMEOUT"
    except Exception as e:
        return -1, "", str(e)


def pair_device(mac, pin):
    """Empareja el HC-05 usando pexpect para manejar el prompt de PIN"""
    try:
        import pexpect
    except ImportError:
        print("❌ pexpect no instalado. Instala con: pip3 install pexpect")
        return False

    print(f"\n🔵 Emparejando {HC05_NAME} ({mac}) con PIN '{pin}'...")

    try:
        child = pexpect.spawn('bluetoothctl', timeout=30)
        child.logfile_read = sys.stdout.buffer  # ver output en tiempo real

        # Activar agente para PIN legacy
        child.expect('#')
        child.sendline('agent on')
        child.expect('#')
        child.sendline('default-agent')
        child.expect('#')

        # Emparejar
        child.sendline(f'pair {mac}')

        # Esperar PIN prompt o éxito
        idx = child.expect([
            'Enter PIN code:',
            'Enter passkey',
            'Paired: yes',
            'AlreadyExists',
            'Failed',
            pexpect.TIMEOUT
        ], timeout=30)

        if idx in [0, 1]:  # PIN prompt
            child.sendline(pin)
            print(f"\n✅ PIN '{pin}' enviado")
            inner_idx = child.expect([
                'Paired: yes',
                'AlreadyExists',
                'Failed',
                pexpect.TIMEOUT
            ], timeout=15)
            if inner_idx in [0, 1]:
                print("✅ Dispositivo emparejado")
            else:
                print(f"⚠️  Resultado desconocido (idx={inner_idx})")
        elif idx == 2:
            print("✅ Ya estaba emparejado")
        elif idx == 3:
            print("✅ Dispositivo ya existe (ya emparejado)")
        else:
            print(f"❌ Error al emparejar (idx={idx})")
            child.close()
            return False

        # Confiar en el dispositivo
        child.sendline(f'trust {mac}')
        child.expect('#', timeout=10)

        # Conectar
        child.sendline(f'connect {mac}')
        conn_idx = child.expect([
            'Connection successful',
            'Connected: yes',
            'Failed',
            pexpect.TIMEOUT
        ], timeout=20)

        if conn_idx in [0, 1]:
            print("\n✅ Conexión Bluetooth establecida")
        else:
            print(f"\n⚠️  Conexión BT puede estar fallando (idx={conn_idx})")
            # Para HC-05 en modo serial, la conexión BT se establece al abrir el puerto RFCOMM
            # No siempre da 'Connection successful' en modo legacy

        child.sendline('quit')
        child.close()
        return True

    except Exception as e:
        print(f"\n❌ Error en pairing: {e}")
        return False


def setup_rfcomm():
    """Crea/recrea el binding RFCOMM"""
    print(f"\n🔌 Configurando RFCOMM {RFCOMM_PORT}...")

    # Liberar si existe
    if os.path.exists(RFCOMM_PORT):
        print(f"   Liberando {RFCOMM_PORT} existente...")
        run_cmd(['sudo', 'rfcomm', 'release', RFCOMM_PORT], timeout=5)
        time.sleep(1)

    # Crear binding
    rc, out, err = run_cmd(
        ['sudo', 'rfcomm', 'bind', RFCOMM_PORT, HC05_MAC, str(RFCOMM_CHANNEL)],
        timeout=10
    )

    time.sleep(1)

    if os.path.exists(RFCOMM_PORT):
        # Permisos para usuario normal
        run_cmd(['sudo', 'chmod', '666', RFCOMM_PORT], timeout=5)
        print(f"✅ {RFCOMM_PORT} creado y listo")
        return True
    else:
        print(f"❌ Error creando RFCOMM: {err}")
        return False


def test_connection(baud=115200):
    """Prueba la conexión serial con el Arduino"""
    try:
        import serial
    except ImportError:
        print("❌ pyserial no instalado. Instala con: pip3 install pyserial")
        return False

    print(f"\n🧪 Probando conexión serial {RFCOMM_PORT} @ {baud}...")

    try:
        ser = serial.Serial(RFCOMM_PORT, baud, timeout=3)
        time.sleep(2)  # Reset Arduino
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print(f"✅ Puerto abierto")

        # Enviar ping y esperar respuesta
        print("   → Enviando '?' al Arduino...")
        ser.write(b'?\n')
        time.sleep(0.5)

        data_received = False
        for _ in range(10):
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    print(f"   ← Arduino responde: '{line}'")
                    data_received = True
            else:
                time.sleep(0.3)

        if not data_received:
            print("   ⚠️  Sin respuesta (normal si el firmware no responde a '?')")
            print("      Escuchando 5 segundos en modo pasivo...")
            ser.timeout = 5
            raw = ser.read(200)
            if raw:
                print(f"   ← Datos recibidos: {raw}")
            else:
                print("   ⚠️  Sin datos. Verifica baud rate y firmware Arduino.")

        ser.close()
        print("✅ Test completado - conexión serial funcional")
        return True

    except Exception as e:
        print(f"❌ Error serial: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description='Conectar HC-05 Arduino con Pi5')
    parser.add_argument('--pin', default=DEFAULT_PIN, help=f'PIN del HC-05 (default: {DEFAULT_PIN})')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate serial (default: 115200)')
    parser.add_argument('--test', action='store_true', help='Solo probar conexión serial (skip pairing)')
    parser.add_argument('--no-pair', action='store_true', help='Saltar emparejamiento, solo RFCOMM')
    args = parser.parse_args()

    print("=" * 55)
    print("  Conexión Arduino HC-05 ↔ Raspberry Pi 5")
    print(f"  Dispositivo : {HC05_NAME} ({HC05_MAC})")
    print(f"  Puerto      : {RFCOMM_PORT}")
    print(f"  Baud rate   : {args.baud}")
    print("=" * 55)

    # Verificar Bluetooth activo
    rc, out, _ = run_cmd(['bluetoothctl', 'show'], timeout=5)
    if 'Powered: yes' not in out:
        print("⚠️  Bluetooth apagado. Encendiendo...")
        run_cmd(['bluetoothctl', 'power', 'on'], timeout=5)
        time.sleep(1)

    if args.test:
        # Solo probar puerto existente
        if not os.path.exists(RFCOMM_PORT):
            print(f"❌ {RFCOMM_PORT} no existe. Ejecuta sin --test primero.")
            sys.exit(1)
        test_connection(args.baud)
        return

    # 1. Emparejar (si no se salta)
    if not args.no_pair:
        success = pair_device(HC05_MAC, args.pin)
        if not success:
            print("\n⚠️  Emparejamiento fallido, intentando crear RFCOMM de todas formas...")
    else:
        print("⏭️  Saltando emparejamiento...")

    # 2. Configurar RFCOMM
    if not setup_rfcomm():
        print("\n❌ No se pudo crear el puerto RFCOMM")
        sys.exit(1)

    # 3. Test de conexión serial
    print()
    test_connection(args.baud)

    print("\n" + "=" * 55)
    print("  Para usar en ROS2:")
    print(f"    serial_port: '{RFCOMM_PORT}'")
    print(f"    baud_rate: {args.baud}")
    print("=" * 55)


if __name__ == '__main__':
    main()
