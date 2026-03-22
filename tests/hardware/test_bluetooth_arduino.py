#!/usr/bin/env python3
"""
Test de Comunicación Bluetooth con Arduino
Prueba la conexión serial vía Bluetooth sin ROS.

Uso:
    python3 test_bluetooth_arduino.py /dev/rfcomm0
    python3 test_bluetooth_arduino.py /dev/rfcomm0 --baud 38400
    python3 test_bluetooth_arduino.py /dev/rfcomm0 --interactive
"""

import serial
import time
import sys
import argparse


class BluetoothArduinoTester:
    """Clase para probar comunicación Bluetooth con Arduino"""
    
    def __init__(self, port, baud_rate=115200, timeout=2.0):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = None
    
    def connect(self):
        """Conectar al puerto Bluetooth"""
        try:
            print(f"🔌 Conectando a {self.port} @ {self.baud_rate} baud...")
            self.ser = serial.Serial(
                self.port,
                self.baud_rate,
                timeout=self.timeout
            )
            time.sleep(2)  # Esperar reset Arduino
            
            # Limpiar buffers
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            print(f"✅ Conectado exitosamente a {self.port}")
            return True
            
        except serial.SerialException as e:
            print(f"❌ Error al conectar: {e}")
            return False
        except Exception as e:
            print(f"❌ Error inesperado: {e}")
            return False
    
    def disconnect(self):
        """Desconectar del puerto"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("🔌 Desconectado")
    
    def send_command(self, cmd):
        """
        Enviar comando al Arduino.
        
        Args:
            cmd: Comando a enviar (string)
            
        Returns:
            bool: True si se envió correctamente
        """
        if not self.ser or not self.ser.is_open:
            print("❌ Puerto no conectado")
            return False
        
        try:
            # Asegurar que termina con newline
            if not cmd.endswith('\n'):
                cmd += '\n'
            
            self.ser.write(cmd.encode('utf-8'))
            self.ser.flush()
            print(f"📤 Enviado: {cmd.strip()}")
            return True
            
        except Exception as e:
            print(f"❌ Error enviando comando: {e}")
            return False
    
    def read_response(self, lines=5, timeout=2.0):
        """
        Leer respuesta del Arduino.
        
        Args:
            lines: Número máximo de líneas a leer
            timeout: Timeout en segundos
            
        Returns:
            list: Lista de líneas recibidas
        """
        if not self.ser or not self.ser.is_open:
            print("❌ Puerto no conectado")
            return []
        
        responses = []
        start_time = time.time()
        
        try:
            while len(responses) < lines and (time.time() - start_time) < timeout:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        responses.append(line)
                        print(f"📥 Recibido: {line}")
                else:
                    time.sleep(0.01)
            
            return responses
            
        except Exception as e:
            print(f"❌ Error leyendo respuesta: {e}")
            return responses
    
    def test_echo(self):
        """Test básico de echo/ping"""
        print("\n" + "="*60)
        print("TEST 1: Echo/Ping")
        print("="*60)
        
        # Intentar comandos comunes
        test_commands = ["?", "STATUS", "INFO", "V"]
        
        for cmd in test_commands:
            print(f"\nProbando comando: '{cmd}'")
            if self.send_command(cmd):
                responses = self.read_response(lines=3, timeout=1.5)
                if responses:
                    print(f"✅ Respuesta recibida ({len(responses)} líneas)")
                else:
                    print("⚠️  Sin respuesta")
                time.sleep(0.5)
    
    def test_encoder_read(self):
        """Test de lectura de encoders"""
        print("\n" + "="*60)
        print("TEST 2: Lectura de Encoders")
        print("="*60)
        
        # Comandos típicos del firmware MOTOR-INTERFACE-V-13
        encoder_commands = ["E", "ENCODER", "ENC"]
        
        for cmd in encoder_commands:
            print(f"\nProbando comando de encoder: '{cmd}'")
            if self.send_command(cmd):
                responses = self.read_response(lines=5, timeout=1.0)
                if responses:
                    print(f"✅ Datos de encoder recibidos")
                    for resp in responses:
                        if any(c.isdigit() for c in resp):
                            print(f"   📊 {resp}")
                else:
                    print("⚠️  Sin respuesta de encoders")
                time.sleep(0.5)
    
    def test_motor_command(self):
        """Test de comando de motores (velocidad 0)"""
        print("\n" + "="*60)
        print("TEST 3: Comando de Motores (velocidad 0 - seguro)")
        print("="*60)
        
        # Enviar velocidad 0 (seguro, no mueve el robot)
        motor_commands = [
            "M 0 0",      # Formato típico: M <izq> <der>
            "MOTOR 0 0",
            "S 0 0"       # Set motor
        ]
        
        for cmd in motor_commands:
            print(f"\nProbando comando de motor: '{cmd}'")
            if self.send_command(cmd):
                responses = self.read_response(lines=2, timeout=1.0)
                if responses:
                    print(f"✅ Comando aceptado")
                else:
                    print("⚠️  Sin confirmación")
                time.sleep(0.5)
    
    def test_continuous_read(self, duration=10):
        """Test de lectura continua"""
        print("\n" + "="*60)
        print(f"TEST 4: Lectura Continua ({duration}s)")
        print("="*60)
        print("Leyendo todos los datos que lleguen...")
        
        start_time = time.time()
        line_count = 0
        
        try:
            while (time.time() - start_time) < duration:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"📥 [{time.time()-start_time:.1f}s] {line}")
                        line_count += 1
                else:
                    time.sleep(0.01)
            
            print(f"\n✅ Lectura completada. Líneas recibidas: {line_count}")
            
        except KeyboardInterrupt:
            print(f"\n⚠️  Interrumpido por usuario. Líneas recibidas: {line_count}")
    
    def interactive_mode(self):
        """Modo interactivo para enviar comandos manualmente"""
        print("\n" + "="*60)
        print("MODO INTERACTIVO")
        print("="*60)
        print("Escribe comandos y presiona Enter.")
        print("Comandos especiales:")
        print("  'read'  - Leer datos durante 5 segundos")
        print("  'exit'  - Salir")
        print("="*60)
        
        try:
            while True:
                cmd = input("\n> ").strip()
                
                if not cmd:
                    continue
                
                if cmd.lower() == 'exit':
                    break
                
                if cmd.lower() == 'read':
                    self.test_continuous_read(duration=5)
                    continue
                
                # Enviar comando
                if self.send_command(cmd):
                    # Leer respuesta
                    print("Esperando respuesta...")
                    responses = self.read_response(lines=10, timeout=2.0)
                    if not responses:
                        print("⚠️  Sin respuesta")
        
        except KeyboardInterrupt:
            print("\n\n⚠️  Saliendo del modo interactivo...")
    
    def run_all_tests(self):
        """Ejecutar todos los tests automáticos"""
        print("\n" + "🔧"*30)
        print("INICIANDO TESTS DE COMUNICACIÓN BLUETOOTH")
        print("🔧"*30)
        
        if not self.connect():
            return False
        
        try:
            # Test 1: Echo
            self.test_echo()
            time.sleep(1)
            
            # Test 2: Encoders
            self.test_encoder_read()
            time.sleep(1)
            
            # Test 3: Motores
            self.test_motor_command()
            time.sleep(1)
            
            # Test 4: Lectura continua
            self.test_continuous_read(duration=5)
            
            print("\n" + "="*60)
            print("✅ TESTS COMPLETADOS")
            print("="*60)
            
        except KeyboardInterrupt:
            print("\n\n⚠️  Tests interrumpidos por usuario")
        
        finally:
            self.disconnect()
        
        return True


def main():
    parser = argparse.ArgumentParser(
        description='Test de comunicación Bluetooth con Arduino',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Ejemplos:
  %(prog)s /dev/rfcomm0
  %(prog)s /dev/rfcomm0 --baud 38400
  %(prog)s /dev/rfcomm0 --interactive
  %(prog)s /dev/rfcomm0 --continuous 30
        '''
    )
    
    parser.add_argument('port', help='Puerto Bluetooth (ej: /dev/rfcomm0)')
    parser.add_argument('--baud', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    parser.add_argument('--timeout', type=float, default=2.0,
                       help='Timeout en segundos (default: 2.0)')
    parser.add_argument('--interactive', '-i', action='store_true',
                       help='Modo interactivo')
    parser.add_argument('--continuous', '-c', type=int, metavar='SECONDS',
                       help='Solo lectura continua por N segundos')
    
    args = parser.parse_args()
    
    # Crear tester
    tester = BluetoothArduinoTester(
        port=args.port,
        baud_rate=args.baud,
        timeout=args.timeout
    )
    
    # Modo continuo
    if args.continuous:
        if tester.connect():
            try:
                tester.test_continuous_read(duration=args.continuous)
            finally:
                tester.disconnect()
        return
    
    # Modo interactivo
    if args.interactive:
        if tester.connect():
            try:
                tester.interactive_mode()
            finally:
                tester.disconnect()
        return
    
    # Modo automático (todos los tests)
    tester.run_all_tests()


if __name__ == '__main__':
    main()
