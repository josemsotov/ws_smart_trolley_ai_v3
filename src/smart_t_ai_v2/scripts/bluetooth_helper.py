#!/usr/bin/env python3
"""
Bluetooth Connection Helper for Arduino Bridge
Gestiona la conexión Bluetooth con módulos HC-05/HC-06

Funciones:
- Escanear dispositivos Bluetooth disponibles
- Emparejar módulo HC-05/HC-06
- Crear puerto RFCOMM
- Obtener dirección MAC del módulo
"""

import subprocess
import sys
import time
import os


class BluetoothHelper:
    """Helper class para gestionar conexiones Bluetooth con Arduino"""
    
    def __init__(self, logger=None):
        self.logger = logger
        self.log = logger.info if logger else print
        self.log_error = logger.error if logger else print
    
    def scan_devices(self, timeout=10):
        """
        Escanea dispositivos Bluetooth disponibles.
        
        Returns:
            list: Lista de tuplas (address, name)
        """
        self.log(f'Escaneando dispositivos Bluetooth ({timeout}s)...')
        devices = []
        
        try:
            # Usar bluetoothctl para escanear
            subprocess.run(['bluetoothctl', 'power', 'on'], 
                          capture_output=True, timeout=5)
            subprocess.run(['bluetoothctl', 'scan', 'on'], 
                          capture_output=True, timeout=1)
            time.sleep(timeout)
            subprocess.run(['bluetoothctl', 'scan', 'off'], 
                          capture_output=True, timeout=1)
            
            # Obtener lista de dispositivos
            result = subprocess.run(['bluetoothctl', 'devices'], 
                                   capture_output=True, text=True, timeout=5)
            
            for line in result.stdout.splitlines():
                # Formato: "Device XX:XX:XX:XX:XX:XX Name"
                if line.startswith('Device '):
                    parts = line.split(maxsplit=2)
                    if len(parts) >= 3:
                        address = parts[1]
                        name = parts[2]
                        devices.append((address, name))
                        self.log(f'  Encontrado: {name} ({address})')
            
            return devices
            
        except subprocess.TimeoutExpired:
            self.log_error('Timeout escaneando dispositivos')
            return []
        except Exception as e:
            self.log_error(f'Error escaneando: {e}')
            return []
    
    def find_device_by_name(self, device_name, timeout=10):
        """
        Busca un dispositivo Bluetooth por nombre.
        
        Args:
            device_name: Nombre del dispositivo (ej: 'HC-05', 'HC-06')
            timeout: Tiempo máximo de escaneo
            
        Returns:
            str: Dirección MAC del dispositivo o None
        """
        devices = self.scan_devices(timeout)
        
        for address, name in devices:
            if device_name.lower() in name.lower():
                self.log(f'Dispositivo encontrado: {name} → {address}')
                return address
        
        self.log_error(f'No se encontró dispositivo con nombre: {device_name}')
        return None
    
    def pair_device(self, address):
        """
        Empareja un dispositivo Bluetooth.
        
        Args:
            address: Dirección MAC del dispositivo
            
        Returns:
            bool: True si se emparejó correctamente
        """
        self.log(f'Emparejando dispositivo {address}...')
        
        try:
            # Intentar emparejar
            subprocess.run(['bluetoothctl', 'pair', address], 
                          capture_output=True, timeout=30)
            
            # Confiar en el dispositivo
            subprocess.run(['bluetoothctl', 'trust', address], 
                          capture_output=True, timeout=5)
            
            self.log(f'Dispositivo {address} emparejado y confiado')
            return True
            
        except subprocess.TimeoutExpired:
            self.log_error('Timeout emparejando dispositivo')
            return False
        except Exception as e:
            self.log_error(f'Error emparejando: {e}')
            return False
    
    def connect_device(self, address):
        """
        Conecta a un dispositivo Bluetooth emparejado.
        
        Args:
            address: Dirección MAC del dispositivo
            
        Returns:
            bool: True si se conectó correctamente
        """
        self.log(f'Conectando a {address}...')
        
        try:
            result = subprocess.run(['bluetoothctl', 'connect', address], 
                                   capture_output=True, text=True, timeout=30)
            
            if 'Connected: yes' in result.stdout or 'Connection successful' in result.stdout:
                self.log(f'Conectado exitosamente a {address}')
                return True
            else:
                self.log_error(f'Fallo al conectar: {result.stdout}')
                return False
                
        except subprocess.TimeoutExpired:
            self.log_error('Timeout conectando dispositivo')
            return False
        except Exception as e:
            self.log_error(f'Error conectando: {e}')
            return False
    
    def create_rfcomm(self, address, channel=1, port='/dev/rfcomm0'):
        """
        Crea un puerto serial RFCOMM.
        
        Args:
            address: Dirección MAC del dispositivo
            channel: Canal RFCOMM (generalmente 1)
            port: Puerto a crear (ej: /dev/rfcomm0)
            
        Returns:
            str: Ruta del puerto creado o None
        """
        self.log(f'Creando puerto RFCOMM {port}...')
        
        try:
            # Verificar si ya existe
            if os.path.exists(port):
                self.log(f'{port} ya existe, liberando...')
                subprocess.run(['sudo', 'rfcomm', 'release', port], 
                             capture_output=True, timeout=5)
                time.sleep(1)
            
            # Crear puerto RFCOMM
            result = subprocess.run(
                ['sudo', 'rfcomm', 'bind', port, address, str(channel)],
                capture_output=True, text=True, timeout=10
            )
            
            if os.path.exists(port):
                self.log(f'Puerto RFCOMM creado: {port}')
                # Dar permisos
                subprocess.run(['sudo', 'chmod', '666', port], 
                             capture_output=True, timeout=5)
                return port
            else:
                self.log_error(f'Fallo al crear {port}: {result.stderr}')
                return None
                
        except Exception as e:
            self.log_error(f'Error creando RFCOMM: {e}')
            return None
    
    def setup_bluetooth_connection(self, device_name=None, address=None, 
                                   channel=1, port='/dev/rfcomm0'):
        """
        Setup completo de conexión Bluetooth.
        
        Args:
            device_name: Nombre del dispositivo (ej: 'HC-05')
            address: Dirección MAC (opcional si se proporciona device_name)
            channel: Canal RFCOMM
            port: Puerto a crear
            
        Returns:
            str: Ruta del puerto serial o None
        """
        # Si no se proporciona address, buscar por nombre
        if not address and device_name:
            address = self.find_device_by_name(device_name)
            if not address:
                return None
        
        if not address:
            self.log_error('No se proporcionó dirección MAC ni nombre de dispositivo')
            return None
        
        # Emparejar si es necesario
        self.pair_device(address)
        
        # Conectar
        if not self.connect_device(address):
            self.log_error('No se pudo conectar al dispositivo')
            return None
        
        # Crear puerto RFCOMM
        return self.create_rfcomm(address, channel, port)


def main():
    """CLI para gestionar conexión Bluetooth"""
    helper = BluetoothHelper()
    
    if len(sys.argv) < 2:
        print("Uso:")
        print("  bluetooth_helper.py scan                    - Escanear dispositivos")
        print("  bluetooth_helper.py connect <nombre>        - Conectar por nombre")
        print("  bluetooth_helper.py connect <MAC> [canal]   - Conectar por MAC")
        print()
        print("Ejemplos:")
        print("  bluetooth_helper.py scan")
        print("  bluetooth_helper.py connect HC-05")
        print("  bluetooth_helper.py connect 00:14:03:06:7D:9E")
        sys.exit(1)
    
    command = sys.argv[1]
    
    if command == 'scan':
        devices = helper.scan_devices()
        print(f'\nEncontrados {len(devices)} dispositivos')
        
    elif command == 'connect':
        if len(sys.argv) < 3:
            print("Error: se requiere nombre o dirección MAC")
            sys.exit(1)
        
        target = sys.argv[2]
        channel = int(sys.argv[3]) if len(sys.argv) > 3 else 1
        
        # Determinar si es MAC o nombre
        if ':' in target:
            # Es una dirección MAC
            port = helper.setup_bluetooth_connection(address=target, channel=channel)
        else:
            # Es un nombre de dispositivo
            port = helper.setup_bluetooth_connection(device_name=target, channel=channel)
        
        if port:
            print(f'\n✅ Conexión exitosa: {port}')
            print(f'Usa este puerto en arduino_bridge: serial_port: "{port}"')
        else:
            print('\n❌ Fallo en la conexión')
            sys.exit(1)
    
    else:
        print(f"Comando desconocido: {command}")
        sys.exit(1)


if __name__ == '__main__':
    main()
