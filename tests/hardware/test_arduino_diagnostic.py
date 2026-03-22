#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
  SMART TROLLEY V5 — Diagnóstico Completo de Hardware Arduino
═══════════════════════════════════════════════════════════════════════════════

Diagnóstica TODOS los dispositivos conectados al Arduino Mega:
  1. Conexión serial (USB o Bluetooth)
  2. Firmware (versión / status)
  3. Encoders Hall (izquierdo + derecho)
  4. IMU MPU9250 (giroscopio Z + acelerómetro X + temperatura)
  5. GPS (NMEA via Arduino o puerto directo)
  6. Motores (test breve de giro + medición de respuesta de encoders)
  7. Monitor en tiempo real de todos los sensores

Uso:
  python3 tests/hardware/test_arduino_diagnostic.py
  python3 tests/hardware/test_arduino_diagnostic.py --port /dev/ttyUSB0
  python3 tests/hardware/test_arduino_diagnostic.py --port /dev/rfcomm0 --baud 38400
  python3 tests/hardware/test_arduino_diagnostic.py --monitor        # Solo monitor live
  python3 tests/hardware/test_arduino_diagnostic.py --skip-motors    # Sin mover motores
  python3 tests/hardware/test_arduino_diagnostic.py --gps-port /dev/ttyUSB1  # GPS directo

Protocolo firmware MOTOR-INTERFACE-V-13:
  Comandos PC → Arduino:
    s         → status firmware
    e         → leer encoders    → "e <left> <right>"
    i         → leer IMU         → "i <yaw_rate_dps> <accel_x_g>"
    g         → leer GPS         → "g <lat> <lon> <fix> <sats> <hdop>"
    r         → reset encoders
    v <l> <a> → velocidad (m/s, rad/s)

Requiere: pip install pyserial
═══════════════════════════════════════════════════════════════════════════════
"""

import serial
import serial.tools.list_ports
import time
import sys
import os
import argparse
import threading
import math
import glob
from datetime import datetime

# ──────────────────────────────────────────────────────────────────────────────
#  ANSI Colors
# ──────────────────────────────────────────────────────────────────────────────
C_RESET  = '\033[0m'
C_BOLD   = '\033[1m'
C_DIM    = '\033[2m'
C_RED    = '\033[91m'
C_GREEN  = '\033[92m'
C_YELLOW = '\033[93m'
C_BLUE   = '\033[94m'
C_MAGENTA= '\033[95m'
C_CYAN   = '\033[96m'
C_WHITE  = '\033[97m'

PASS  = f'{C_GREEN}[  PASS  ]{C_RESET}'
FAIL  = f'{C_RED}[  FAIL  ]{C_RESET}'
WARN  = f'{C_YELLOW}[  WARN  ]{C_RESET}'
INFO  = f'{C_CYAN}[  INFO  ]{C_RESET}'
SKIP  = f'{C_DIM}[  SKIP  ]{C_RESET}'


# ──────────────────────────────────────────────────────────────────────────────
#  Helpers
# ──────────────────────────────────────────────────────────────────────────────

def header(title):
    w = 70
    print(f'\n{C_BOLD}{C_BLUE}{"═"*w}{C_RESET}')
    print(f'{C_BOLD}{C_BLUE}  {title}{C_RESET}')
    print(f'{C_BOLD}{C_BLUE}{"─"*w}{C_RESET}')


def subheader(title):
    print(f'\n{C_CYAN}  ┌─ {title} ─┐{C_RESET}')


def result(label, passed, detail=''):
    tag = PASS if passed else FAIL
    detail_str = f'  {C_DIM}→ {detail}{C_RESET}' if detail else ''
    print(f'  {tag} {label}{detail_str}')


def info(msg):
    print(f'  {INFO} {msg}')


def warn(msg):
    print(f'  {WARN} {C_YELLOW}{msg}{C_RESET}')


def bar(value, max_val, width=20, unit=''):
    """Barra de progreso para valores analógicos."""
    filled = int(min(abs(value) / max_val, 1.0) * width)
    bar_str = '█' * filled + '░' * (width - filled)
    sign = '+' if value >= 0 else '-'
    return f'{sign}[{bar_str}] {value:+.3f}{unit}'


def find_arduino_ports():
    """Detecta automáticamente puertos Arduino/serial disponibles."""
    candidates = []
    for p in serial.tools.list_ports.comports():
        desc = (p.description or '').lower()
        vid  = p.vid or 0
        # Arduino Mega/Uno: vid 0x2341, 0x2A03, 0x1A86 (CH340)
        if vid in (0x2341, 0x2A03, 0x1A86, 0x0403) or \
           'arduino' in desc or 'ch340' in desc or 'ft232' in desc or \
           'acm' in p.device.lower() or ('usb' in p.device.lower() and 'gps' not in desc):
            candidates.append(p)
    return candidates


def find_gps_ports():
    """Detecta puertos GPS directos (NMEA)."""
    candidates = []
    for p in serial.tools.list_ports.comports():
        desc = (p.description or '').lower()
        if 'gps' in desc or 'u-blox' in desc or 'gnss' in desc or \
           'neo' in desc or 'sirf' in desc:
            candidates.append(p)
    # También buscar /dev/ttyUSB* que no sean Arduino
    return candidates


# ──────────────────────────────────────────────────────────────────────────────
#  Clase principal de diagnóstico
# ──────────────────────────────────────────────────────────────────────────────

class ArduinoDiagnostic:

    def __init__(self, port, baud, gps_port=None, skip_motors=False):
        self.port = port
        self.baud = baud
        self.gps_port = gps_port
        self.skip_motors = skip_motors
        self.ser = None
        self.gps_ser = None

        # Resultados
        self.results = {
            'connection': None,
            'firmware':   None,
            'encoders':   None,
            'imu':        None,
            'gps':        None,
            'motors':     None,
        }
        # Datos de sensores para el monitor
        self._live_data = {}
        self._monitor_running = False

    # ──────────────────────────────────────────────────────────────────────────
    #  Conexión
    # ──────────────────────────────────────────────────────────────────────────

    def connect(self):
        header('1 / CONEXIÓN SERIAL')
        is_bt = 'rfcomm' in self.port
        timeout = 0.5 if is_bt else 0.2

        info(f'Puerto: {C_WHITE}{self.port}{C_RESET}  Baud: {C_WHITE}{self.baud}{C_RESET}')
        info(f'Modo: {"Bluetooth (rfcomm)" if is_bt else "USB"}')

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=timeout)

            if not is_bt:
                info('Esperando reset Arduino (2s)...')
                time.sleep(2.0)
            else:
                time.sleep(0.5)

            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            result('Serial abierto correctamente', True,
                   f'{self.port} @ {self.baud} baud')
            self.results['connection'] = True
            return True

        except serial.SerialException as e:
            result('Serial abierto correctamente', False, str(e))
            self.results['connection'] = False
            self._print_port_help()
            return False

    def _print_port_help(self):
        print(f'\n  {C_YELLOW}Sugerencias:{C_RESET}')
        ports = find_arduino_ports()
        if ports:
            print(f'  {C_CYAN}  Puertos detectados:{C_RESET}')
            for p in ports:
                print(f'     {p.device}  {C_DIM}({p.description}){C_RESET}')
        else:
            print(f'  {C_DIM}  No se detectaron puertos Arduino/serial.{C_RESET}')
        print(f'  {C_DIM}  • ls -l /dev/ttyACM* /dev/ttyUSB* /dev/rfcomm*{C_RESET}')
        print(f'  {C_DIM}  • sudo usermod -a -G dialout $USER  (luego re-login){C_RESET}')

    # ──────────────────────────────────────────────────────────────────────────
    #  Comunicación serial básica
    # ──────────────────────────────────────────────────────────────────────────

    def _send(self, cmd):
        """Envía un comando (sin newline) + '\n'."""
        if not self.ser or not self.ser.is_open:
            return False
        try:
            self.ser.write((cmd + '\n').encode())
            self.ser.flush()
            return True
        except Exception:
            return False

    def _read_lines(self, timeout=1.0, max_lines=10):
        """Lee líneas disponibles hasta timeout."""
        lines = []
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        lines.append(line)
                        if len(lines) >= max_lines:
                            break
                except Exception:
                    break
            else:
                time.sleep(0.01)
        return lines

    def _query(self, cmd, timeout=0.8):
        """Envía comando y lee respuesta."""
        self.ser.reset_input_buffer()
        self._send(cmd)
        return self._read_lines(timeout=timeout)

    # ──────────────────────────────────────────────────────────────────────────
    #  2. Firmware
    # ──────────────────────────────────────────────────────────────────────────

    def test_firmware(self):
        header('2 / FIRMWARE ARDUINO')
        lines = self._query('s', timeout=1.0)

        if lines:
            result('Respuesta al comando "s" (status)', True, lines[0])
            # Buscar versión en la respuesta
            for l in lines:
                if any(kw in l.lower() for kw in ['v', 'motor', 'trolley', 'ready', 'ok']):
                    info(f'Firmware: {C_WHITE}{l}{C_RESET}')
            self.results['firmware'] = True
        else:
            result('Respuesta al comando "s" (status)', False,
                   'Sin respuesta — verificar firmware')
            self.results['firmware'] = False

        # Mostrar todas las líneas recibidas
        for i, l in enumerate(lines):
            print(f'     {C_DIM}[{i}] {l}{C_RESET}')

        return bool(lines)

    # ──────────────────────────────────────────────────────────────────────────
    #  3. Encoders Hall
    # ──────────────────────────────────────────────────────────────────────────

    def test_encoders(self):
        header('3 / ENCODERS HALL')

        # Primero reset para tener valores conocidos
        self._send('r')
        time.sleep(0.2)

        # Leer 5 muestras consecutivas
        samples = []
        for attempt in range(5):
            lines = self._query('e', timeout=0.5)
            for l in lines:
                if l.startswith('e '):
                    parts = l.split()
                    if len(parts) >= 3:
                        try:
                            left  = int(parts[1])
                            right = int(parts[2])
                            samples.append((left, right))
                        except ValueError:
                            pass
            time.sleep(0.1)

        if not samples:
            result('Encoders respondiendo', False,
                   'Sin respuesta al comando "e" — verificar firmware y encoders')
            self.results['encoders'] = False
            return False

        # Mostrar muestras
        result('Encoders respondiendo', True, f'{len(samples)} muestras obtenidas')
        print(f'\n  {"Muestra":>8}  {"Izquierdo":>12}  {"Derecho":>12}')
        print(f'  {"─"*8}  {"─"*12}  {"─"*12}')
        for i, (l, r) in enumerate(samples):
            print(f'  {i+1:>8}  {l:>12}  {r:>12}')

        # Verificar si los encoders están incrementando (girar ruedas a mano)
        left_var  = max(s[0] for s in samples) - min(s[0] for s in samples)
        right_var = max(s[1] for s in samples) - min(s[1] for s in samples)

        last_l, last_r = samples[-1]
        result('Encoder izquierdo conectado', True,
               f'valor={last_l}  (variación={left_var} pulsos)')
        result('Encoder derecho conectado', True,
               f'valor={last_r}  (variación={right_var} pulsos)')

        if left_var == 0 and right_var == 0:
            warn('Encoders con variación 0 — robot en reposo (normal) o '
                 'sensor desconectado. Girar ruedas manualmente para verificar.')
        else:
            info(f'Movimiento detectado: izq={left_var} pulsos, der={right_var} pulsos')

        self._live_data['encoders'] = samples[-1]
        self.results['encoders'] = True
        return True

    # ──────────────────────────────────────────────────────────────────────────
    #  4. IMU MPU9250
    # ──────────────────────────────────────────────────────────────────────────

    def test_imu(self):
        header('4 / IMU MPU9250')

        # Leer 10 muestras IMU
        samples = []
        for _ in range(10):
            lines = self._query('i', timeout=0.5)
            for l in lines:
                if l.startswith('i '):
                    parts = l.split()
                    # Protocolo básico: "i <yaw_rate_dps> <accel_x_g>"
                    if len(parts) >= 3:
                        try:
                            yaw_rate = float(parts[1])   # dps
                            accel_x  = float(parts[2])   # g
                            temp     = float(parts[3]) if len(parts) >= 4 else None
                            samples.append({'yaw': yaw_rate, 'ax': accel_x, 'temp': temp})
                        except ValueError:
                            pass
                    # Protocolo extendido: "i <yaw> <ax> <ay> <az> <gx> <gy> <gz> <temp>"
                    elif len(parts) >= 8:
                        try:
                            d = {
                                'yaw': float(parts[1]),
                                'ax':  float(parts[2]),
                                'ay':  float(parts[3]),
                                'az':  float(parts[4]),
                                'gx':  float(parts[5]),
                                'gy':  float(parts[6]),
                                'gz':  float(parts[7]),
                                'temp': float(parts[8]) if len(parts) >= 9 else None,
                            }
                            samples.append(d)
                        except ValueError:
                            pass
            time.sleep(0.08)

        if not samples:
            result('MPU9250 respondiendo', False,
                   'Sin respuesta al comando "i" — ¿Perfil A activo? '
                   '(sensor_profile: A en hardware.yaml)')
            warn('El firmware Perfil B (Hall+Opto) puede no tener soporte IMU activo.')
            warn('Verificar en firmware: USE_IMU / SENSOR_PROFILE define.')
            self.results['imu'] = False
            return False

        result('MPU9250 respondiendo', True, f'{len(samples)} muestras')

        # Estadísticas
        yaw_vals = [s['yaw'] for s in samples]
        ax_vals  = [s['ax']  for s in samples]

        yaw_mean = sum(yaw_vals) / len(yaw_vals)
        yaw_std  = math.sqrt(sum((v - yaw_mean)**2 for v in yaw_vals) / len(yaw_vals))
        ax_mean  = sum(ax_vals) / len(ax_vals)
        ax_std   = math.sqrt(sum((v - ax_mean)**2 for v in ax_vals) / len(ax_vals))

        print(f'\n  {C_BOLD}Giroscopio Z (yaw rate):{C_RESET}')
        for s in samples[-5:]:
            print(f'    {bar(s["yaw"], 250.0, unit="°/s")}')
        print(f'  Media={yaw_mean:+.3f}°/s  Desv.Est={yaw_std:.4f}')

        print(f'\n  {C_BOLD}Acelerómetro X:{C_RESET}')
        for s in samples[-5:]:
            print(f'    {bar(s["ax"], 2.0, unit="g")}')
        print(f'  Media={ax_mean:+.4f}g  Desv.Est={ax_std:.5f}')

        # Temperatura
        temps = [s['temp'] for s in samples if s.get('temp') is not None]
        if temps:
            temp_mean = sum(temps) / len(temps)
            print(f'\n  {C_BOLD}Temperatura interna:{C_RESET}  {temp_mean:.1f}°C')
            temp_ok = 15 <= temp_mean <= 85
            result('Temperatura IMU en rango (15-85°C)', temp_ok,
                   f'{temp_mean:.1f}°C')

        # Verificaciones
        result('Giroscopio Z en reposo (<5°/s)', abs(yaw_mean) < 5.0,
               f'media={yaw_mean:+.3f}°/s')
        result('Acelerómetro X en reposo (~0g)', abs(ax_mean) < 0.3,
               f'media={ax_mean:+.4f}g')
        result('Ruido bajo (std yaw <0.5°/s)', yaw_std < 0.5,
               f'std={yaw_std:.4f}°/s')

        if abs(ax_mean) > 1.5:
            warn(f'Acelerómetro X alto ({ax_mean:.3f}g) — ¿El robot está inclinado?')

        self._live_data['imu'] = samples[-1]
        self.results['imu'] = True
        return True

    # ──────────────────────────────────────────────────────────────────────────
    #  5. GPS
    # ──────────────────────────────────────────────────────────────────────────

    def test_gps(self):
        header('5 / GPS')

        gps_found = False

        # ── A) Intentar vía Arduino (comando g) ──────────────────────────────
        print(f'\n  {C_CYAN}[A] GPS vía Arduino (comando "g"){C_RESET}')
        gps_arduino_samples = []

        for attempt in range(5):
            lines = self._query('g', timeout=1.2)
            for l in lines:
                if l.startswith('g '):
                    parts = l.split()
                    # Esperado: "g <lat> <lon> <fix> <sats> <hdop>"
                    if len(parts) >= 4:
                        try:
                            data = {
                                'lat':  float(parts[1]),
                                'lon':  float(parts[2]),
                                'fix':  int(parts[3]),
                                'sats': int(parts[4]) if len(parts) > 4 else -1,
                                'hdop': float(parts[5]) if len(parts) > 5 else -1.0,
                                'alt':  float(parts[6]) if len(parts) > 6 else None,
                                'speed': float(parts[7]) if len(parts) > 7 else None,
                            }
                            gps_arduino_samples.append(data)
                        except (ValueError, IndexError):
                            pass
                # También puede venir como NMEA directo si el firmware hace pass-through
                elif l.startswith('$GP') or l.startswith('$GN'):
                    gps_arduino_samples.append({'nmea': l})
            time.sleep(0.2)

        if gps_arduino_samples:
            gps_found = True
            result('GPS vía Arduino ("g") respondiendo', True,
                   f'{len(gps_arduino_samples)} muestras')
            for s in gps_arduino_samples[:3]:
                if 'nmea' in s:
                    print(f'     NMEA: {C_WHITE}{s["nmea"]}{C_RESET}')
                else:
                    fix_str = {0: 'Sin fix', 1: 'GPS', 2: 'DGPS', 3: 'PPS'}.get(s['fix'], '?')
                    print(f'     Lat={s["lat"]:.6f}°  Lon={s["lon"]:.6f}°  '
                          f'Fix={fix_str}  Sats={s["sats"]}  HDOP={s["hdop"]:.1f}')
                    if s.get('alt') is not None:
                        print(f'     Alt={s["alt"]:.1f}m  '
                              f'Speed={s["speed"]:.2f}m/s' if s.get('speed') else '')

            last = gps_arduino_samples[-1]
            if 'fix' in last:
                result('GPS tiene fix', last['fix'] > 0,
                       f'fix={last["fix"]}, sats={last["sats"]}')
                if last['fix'] == 0:
                    warn('Sin fix GPS — asegúrate de estar en exterior con '
                         'cielo despejado (tarda 1-2 min en frío).')
        else:
            result('GPS vía Arduino ("g") respondiendo', False,
                   'Sin respuesta — firmware puede no tener soporte GPS aún')
            warn('Si el GPS está conectado al Arduino pero el firmware no tiene '
                 'comando "g", agregar handler en firmware y arduino_bridge.py')

        # ── B) GPS directo por puerto serial (NMEA standalone) ───────────────
        print(f'\n  {C_CYAN}[B] GPS directo por puerto serial (NMEA){C_RESET}')

        gps_ports_to_try = []
        if self.gps_port:
            gps_ports_to_try.append(self.gps_port)

        # Auto-detectar puertos GPS (excluir el puerto ya conectado al Arduino)
        detected = find_gps_ports()
        for p in detected:
            if p.device != self.port:
                gps_ports_to_try.append(p.device)

        # Escanear baudrates comunes de GPS
        GPS_BAUDS = [9600, 115200, 38400, 57600]

        if not gps_ports_to_try:
            # Escanear todos los ttyUSB/ttyACM que no sean el Arduino
            all_usb = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
            for dev in all_usb:
                if dev != self.port:
                    gps_ports_to_try.append(dev)

        gps_direct_found = False
        for gps_p in gps_ports_to_try:
            info(f'Probando {gps_p} como GPS directo...')
            for baud in GPS_BAUDS:
                nmea_lines = self._probe_nmea_port(gps_p, baud, timeout=2.5)
                if nmea_lines:
                    gps_direct_found = True
                    gps_found = True
                    result(f'GPS NMEA directo en {gps_p} @ {baud}', True,
                           f'{len(nmea_lines)} frases NMEA')
                    for l in nmea_lines[:4]:
                        print(f'     {C_WHITE}{l}{C_RESET}')
                    self._parse_nmea_summary(nmea_lines)
                    break
            if gps_direct_found:
                break

        if not gps_direct_found and not gps_arduino_samples:
            warn('GPS no detectado en ningún puerto.')
            info('Opciones:')
            print(f'  {C_DIM}  1. GPS conectado al Arduino → agregar comando "g" al firmware')
            print(f'     2. GPS en serial separado → especificar con --gps-port /dev/ttyUSB1')
            print(f'     3. GPS no conectado aún (agregar en futuro){C_RESET}')

        self.results['gps'] = gps_found
        return gps_found

    def _probe_nmea_port(self, port, baud, timeout=2.5):
        """Intenta abrir un puerto y leer frases NMEA."""
        nmea_lines = []
        try:
            with serial.Serial(port, baud, timeout=0.5) as s:
                time.sleep(0.3)
                t0 = time.time()
                while time.time() - t0 < timeout:
                    if s.in_waiting:
                        try:
                            line = s.readline().decode('ascii', errors='ignore').strip()
                            if line.startswith('$') and ',' in line and len(line) > 10:
                                nmea_lines.append(line)
                        except Exception:
                            pass
                    else:
                        time.sleep(0.05)
        except (serial.SerialException, PermissionError, OSError):
            pass
        return nmea_lines

    def _parse_nmea_summary(self, lines):
        """Extrae y muestra datos clave de frases NMEA."""
        for line in lines:
            try:
                parts = line.split(',')
                sentence = parts[0][1:]  # Quitar '$'

                if sentence.endswith('GGA') and len(parts) >= 10:
                    lat_raw = parts[2]; lat_d = parts[3]
                    lon_raw = parts[4]; lon_d = parts[5]
                    fix_q   = parts[6]
                    sats    = parts[7]
                    hdop    = parts[8]
                    alt     = parts[9]

                    if lat_raw and lon_raw:
                        lat = _nmea_to_decimal(lat_raw, lat_d)
                        lon = _nmea_to_decimal(lon_raw, lon_d)
                        fix_str = {'0': 'Sin fix', '1': 'GPS', '2': 'DGPS',
                                   '3': 'PPS', '4': 'RTK', '5': 'Float RTK'}.get(fix_q, '?')
                        print(f'     {C_BOLD}GGA{C_RESET}: Lat={lat:.6f}°  '
                              f'Lon={lon:.6f}°  Fix={fix_str}  '
                              f'Sats={sats}  HDOP={hdop}  Alt={alt}m')
                        result('GPS tiene fix (GGA)', fix_q not in ('0', ''),
                               f'fix={fix_str}, sats={sats}')

                elif sentence.endswith('RMC') and len(parts) >= 10:
                    status  = parts[2]   # A=active, V=void
                    lat_raw = parts[3]; lat_d = parts[4]
                    lon_raw = parts[5]; lon_d = parts[6]
                    speed_k = parts[7]
                    date    = parts[9]

                    if lat_raw and lon_raw:
                        lat = _nmea_to_decimal(lat_raw, lat_d)
                        lon = _nmea_to_decimal(lon_raw, lon_d)
                        spd = float(speed_k) * 0.514444 if speed_k else 0.0  # knots → m/s
                        print(f'     {C_BOLD}RMC{C_RESET}: Lat={lat:.6f}°  '
                              f'Lon={lon:.6f}°  Speed={spd:.2f}m/s  '
                              f'Status={"ACTIVO" if status == "A" else "VOID"}  '
                              f'Fecha={date}')

                elif sentence.endswith('GSV') and len(parts) >= 4:
                    total_msgs = parts[1]
                    msg_num    = parts[2]
                    total_sats = parts[3]
                    print(f'     {C_BOLD}GSV{C_RESET}: Satélites en vista={total_sats}')

            except (ValueError, IndexError):
                continue

    # ──────────────────────────────────────────────────────────────────────────
    #  6. Motores
    # ──────────────────────────────────────────────────────────────────────────

    def test_motors(self):
        header('6 / MOTORES (test breve)')

        if self.skip_motors:
            result('Test de motores', None, 'OMITIDO (--skip-motors)')
            print(f'  {SKIP} Test de motores omitido por flag --skip-motors')
            self.results['motors'] = None
            return True

        print(f'\n  {C_YELLOW}⚠  Este test moverá los motores brevemente.{C_RESET}')
        print(f'  {C_YELLOW}   Asegúrate de que el robot tenga espacio libre.{C_RESET}')
        try:
            answer = input(f'\n  ¿Continuar? [S/n]: ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            answer = 'n'

        if answer in ('n', 'no'):
            print(f'  {SKIP} Test de motores omitido por el usuario.')
            self.results['motors'] = None
            return True

        # Reset encoders
        self._send('r')
        time.sleep(0.2)

        # Leer encoders antes
        enc_before = self._get_encoders()
        info(f'Encoders antes del movimiento: izq={enc_before[0]}, der={enc_before[1]}')

        motor_results = []

        # Test 1: Avance corto (0.2 m/s, 0.8 seg)
        subheader('Test 1: Avance 0.2 m/s durante 0.8s')
        self._send('v 0.2 0.0')
        time.sleep(0.8)
        self._send('v 0.0 0.0')
        time.sleep(0.3)

        enc_after_fwd = self._get_encoders()
        delta_l = enc_after_fwd[0] - enc_before[0]
        delta_r = enc_after_fwd[1] - enc_before[1]
        fwd_ok = delta_l > 2 and delta_r > 2
        motor_results.append(('Avance: encoders incrementaron', fwd_ok,
                               f'Δizq={delta_l}, Δder={delta_r}'))
        result('Avance: encoders incrementaron', fwd_ok,
               f'Δizq={delta_l}, Δder={delta_r}')
        if not fwd_ok:
            warn('Encoders sin cambio — verificar alimentación 12V y drivers ZS-X11H')

        # Test 2: Retroceso corto
        subheader('Test 2: Retroceso 0.2 m/s durante 0.8s')
        enc_before_rev = enc_after_fwd
        self._send('v -0.2 0.0')
        time.sleep(0.8)
        self._send('v 0.0 0.0')
        time.sleep(0.3)

        enc_after_rev = self._get_encoders()
        delta_l_r = enc_before_rev[0] - enc_after_rev[0]
        delta_r_r = enc_before_rev[1] - enc_after_rev[1]
        rev_ok = delta_l_r > 2 and delta_r_r > 2
        result('Retroceso: encoders decrementaron', rev_ok,
               f'Δizq={delta_l_r}, Δder={delta_r_r}')

        # Test 3: Giro sobre el eje (rotación pura Z)
        subheader('Test 3: Rotación pura (giro Z) 0.5 rad/s durante 0.8s')
        enc_before_rot = enc_after_rev
        self._send('v 0.0 0.5')
        time.sleep(0.8)
        self._send('v 0.0 0.0')
        time.sleep(0.3)

        enc_after_rot = self._get_encoders()
        dl_rot = enc_after_rot[0] - enc_before_rot[0]
        dr_rot = enc_after_rot[1] - enc_before_rot[1]
        # En giro puro: una rueda avanza, la otra retrocede
        rot_ok = (dl_rot > 2 and dr_rot < -2) or (dl_rot < -2 and dr_rot > 2)
        result('Rotación: ruedas opuestas en encoders', rot_ok,
               f'Δizq={dl_rot}, Δder={dr_rot}')
        if not rot_ok and (abs(dl_rot) > 2 or abs(dr_rot) > 2):
            warn('Encoders se mueven en la misma dirección en rotación — '
                 'verificar conexión diferencial de motores')

        # Coherencia velocidades
        enc_before_coh = self._get_encoders()
        self._send('v 0.3 0.0')
        time.sleep(1.0)
        self._send('v 0.0 0.0')
        time.sleep(0.3)
        enc_after_coh = self._get_encoders()
        dl_coh = enc_after_coh[0] - enc_before_coh[0]
        dr_coh = enc_after_coh[1] - enc_before_coh[1]
        if dl_coh > 0 and dr_coh > 0:
            ratio = min(dl_coh, dr_coh) / max(dl_coh, dr_coh) if max(dl_coh, dr_coh) > 0 else 0
            result('Coherencia ruedas (izq/der)', ratio > 0.7,
                   f'ratio={ratio:.2f} (ideal=1.00), Δizq={dl_coh}, Δder={dr_coh}')
            if ratio < 0.7:
                warn(f'Asimetría ruedas ({ratio:.2f}) — puede indicar desgaste o '
                     'diferencia de PPR entre sensores')

        self.results['motors'] = True
        return True

    def _get_encoders(self):
        """Lee encoders y retorna (left, right)."""
        for _ in range(3):
            lines = self._query('e', timeout=0.5)
            for l in lines:
                if l.startswith('e '):
                    parts = l.split()
                    if len(parts) >= 3:
                        try:
                            return (int(parts[1]), int(parts[2]))
                        except ValueError:
                            pass
            time.sleep(0.1)
        return (0, 0)

    # ──────────────────────────────────────────────────────────────────────────
    #  7. Monitor en tiempo real
    # ──────────────────────────────────────────────────────────────────────────

    def run_monitor(self, duration=None):
        """Monitor en tiempo real de todos los sensores. Ctrl+C para salir."""
        header('MONITOR EN TIEMPO REAL')
        print(f'  {C_CYAN}Ctrl+C para salir{C_RESET}\n')

        enc_interval = 0.1    # 10 Hz
        imu_interval = 0.08   # ~12 Hz
        gps_interval = 1.0    # 1 Hz
        last_enc = 0.0
        last_imu = 0.0
        last_gps = 0.0

        enc_data = (0, 0)
        imu_data = None
        gps_data = None
        start_time = time.time()
        loop_count = 0

        try:
            while True:
                now = time.time()
                if duration and (now - start_time) > duration:
                    break

                # Leer serial acumulado
                while self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if line.startswith('e '):
                            p = line.split()
                            if len(p) >= 3:
                                enc_data = (int(p[1]), int(p[2]))
                        elif line.startswith('i '):
                            p = line.split()
                            if len(p) >= 3:
                                imu_data = {
                                    'yaw': float(p[1]),
                                    'ax':  float(p[2]),
                                    'temp': float(p[3]) if len(p) >= 4 else None,
                                }
                        elif line.startswith('g '):
                            p = line.split()
                            if len(p) >= 4:
                                gps_data = {
                                    'lat': float(p[1]),
                                    'lon': float(p[2]),
                                    'fix': int(p[3]),
                                    'sats': int(p[4]) if len(p) > 4 else -1,
                                }
                        elif line.startswith('$GP') or line.startswith('$GN'):
                            gps_data = {'nmea': line[:60]}
                    except Exception:
                        pass

                # Enviar solicitudes periódicas
                if now - last_enc >= enc_interval:
                    self._send('e')
                    last_enc = now

                if now - last_imu >= imu_interval:
                    self._send('i')
                    last_imu = now

                if now - last_gps >= gps_interval:
                    self._send('g')
                    last_gps = now

                # Render cada ~200ms
                loop_count += 1
                if loop_count % 20 == 0:
                    elapsed = now - start_time
                    os.system('clear' if os.name != 'nt' else 'cls')
                    print(f'{C_BOLD}{C_BLUE}══ SMART TROLLEY — Monitor Hardware  '
                          f'[{elapsed:.0f}s]  Ctrl+C=salir ══{C_RESET}')
                    print(f'{C_DIM}  Puerto: {self.port} @ {self.baud}{C_RESET}\n')

                    # Encoders
                    print(f'{C_BOLD}  ENCODERS HALL{C_RESET}')
                    print(f'    Izquierdo: {C_WHITE}{enc_data[0]:>8}{C_RESET} pulsos')
                    print(f'    Derecho  : {C_WHITE}{enc_data[1]:>8}{C_RESET} pulsos')

                    # IMU
                    print(f'\n{C_BOLD}  IMU MPU9250{C_RESET}')
                    if imu_data:
                        yaw_b = bar(imu_data['yaw'], 250.0, unit='°/s')
                        ax_b  = bar(imu_data['ax'],  2.0,   unit='g')
                        print(f'    Giroscopio Z : {C_CYAN}{yaw_b}{C_RESET}')
                        print(f'    Aceleróm.  X : {C_CYAN}{ax_b}{C_RESET}')
                        if imu_data.get('temp'):
                            print(f'    Temperatura  : {C_CYAN}{imu_data["temp"]:.1f}°C{C_RESET}')
                    else:
                        print(f'    {C_DIM}Sin datos IMU{C_RESET}')

                    # GPS
                    print(f'\n{C_BOLD}  GPS{C_RESET}')
                    if gps_data:
                        if 'nmea' in gps_data:
                            print(f'    NMEA : {C_GREEN}{gps_data["nmea"]}{C_RESET}')
                        else:
                            fix_str = {0:'Sin fix', 1:'GPS', 2:'DGPS'}.get(
                                gps_data.get('fix',0), '?')
                            fix_c = C_GREEN if gps_data.get('fix',0) > 0 else C_RED
                            print(f'    Lat  : {C_WHITE}{gps_data.get("lat", 0):.6f}°{C_RESET}')
                            print(f'    Lon  : {C_WHITE}{gps_data.get("lon", 0):.6f}°{C_RESET}')
                            print(f'    Fix  : {fix_c}{fix_str}{C_RESET}  '
                                  f'Sats={gps_data.get("sats",-1)}')
                    else:
                        print(f'    {C_DIM}Sin datos GPS (--gps-port o firmware comando "g"){C_RESET}')

                    print(f'\n{C_DIM}  Tasa: enc=10Hz  imu=12Hz  gps=1Hz{C_RESET}')

                time.sleep(0.01)

        except KeyboardInterrupt:
            print(f'\n  {C_CYAN}Monitor detenido.{C_RESET}')

    # ──────────────────────────────────────────────────────────────────────────
    #  Resumen final
    # ──────────────────────────────────────────────────────────────────────────

    def print_summary(self):
        header('RESUMEN DEL DIAGNÓSTICO')

        labels = {
            'connection': 'Conexión serial',
            'firmware':   'Firmware Arduino',
            'encoders':   'Encoders Hall',
            'imu':        'IMU MPU9250',
            'gps':        'GPS',
            'motors':     'Motores',
        }

        all_ok = True
        for key, label in labels.items():
            r = self.results.get(key)
            if r is True:
                tag = PASS
            elif r is False:
                tag = FAIL
                all_ok = False
            else:
                tag = SKIP

            print(f'  {tag} {label}')

        print()
        if all_ok:
            print(f'  {C_GREEN}{C_BOLD}✓ Todo el hardware diagnosticado correctamente.{C_RESET}')
        else:
            print(f'  {C_YELLOW}{C_BOLD}⚠ Algunos tests fallaron. Ver detalles arriba.{C_RESET}')

        print(f'\n  {C_DIM}Timestamp: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}{C_RESET}')
        print(f'  {C_DIM}Puerto: {self.port} @ {self.baud}{C_RESET}\n')

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                self._send('v 0.0 0.0')  # Seguridad: detener motores
                time.sleep(0.1)
                self.ser.close()
            except Exception:
                pass


# ──────────────────────────────────────────────────────────────────────────────
#  Utilidad NMEA
# ──────────────────────────────────────────────────────────────────────────────

def _nmea_to_decimal(raw, direction):
    """Convierte NMEA dddmm.mmmm + N/S/E/W a grados decimales."""
    try:
        raw = raw.strip()
        if '.' not in raw:
            return 0.0
        dot_pos = raw.index('.')
        # Grados: todo antes de los últimos 2 dígitos enteros
        deg_len = dot_pos - 2
        if deg_len < 1:
            return 0.0
        degrees = float(raw[:deg_len])
        minutes = float(raw[deg_len:])
        decimal = degrees + minutes / 60.0
        if direction in ('S', 'W'):
            decimal = -decimal
        return decimal
    except (ValueError, IndexError):
        return 0.0


# ──────────────────────────────────────────────────────────────────────────────
#  main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Smart Trolley V5 — Diagnóstico completo de hardware Arduino',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('--port', default=None,
                        help='Puerto serial Arduino (default: auto-detectar)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Baudrate (default: 115200; BT usar 38400)')
    parser.add_argument('--gps-port', default=None,
                        help='Puerto GPS directo (NMEA), p.ej. /dev/ttyUSB1')
    parser.add_argument('--skip-motors', action='store_true',
                        help='Omitir test de motores (no mover ruedas)')
    parser.add_argument('--monitor', action='store_true',
                        help='Solo ejecutar monitor en tiempo real')
    parser.add_argument('--monitor-only', action='store_true',
                        help='Alias de --monitor')
    args = parser.parse_args()

    # ── Banner ────────────────────────────────────────────────────────────────
    print(f'\n{C_BOLD}{C_BLUE}{"═"*70}{C_RESET}')
    print(f'{C_BOLD}{C_BLUE}  SMART TROLLEY V5 — Diagnóstico de Hardware{C_RESET}')
    print(f'{C_BOLD}{C_BLUE}  Firmware: MOTOR-INTERFACE-V-13{C_RESET}')
    print(f'{C_BLUE}{"─"*70}{C_RESET}')
    print(f'  Fecha  : {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
    print(f'  SO     : {os.uname().sysname} {os.uname().machine}')
    print(f'{"═"*70}{C_RESET}\n')

    # ── Auto-detectar puerto ──────────────────────────────────────────────────
    port = args.port
    if not port:
        candidates = find_arduino_ports()
        # Preferir /dev/ttyACM0 (Arduino Mega USB)
        preferred = [p for p in candidates if 'ACM' in p.device]
        if preferred:
            port = preferred[0].device
            info(f'Puerto auto-detectado: {C_WHITE}{port}{C_RESET}  '
                 f'({preferred[0].description})')
        elif candidates:
            port = candidates[0].device
            info(f'Puerto auto-detectado: {C_WHITE}{port}{C_RESET}  '
                 f'({candidates[0].description})')
        else:
            # Fallback al default
            port = '/dev/ttyACM0'
            warn(f'No se detectó Arduino automáticamente. Usando {port}. '
                 f'Especifica con --port si es diferente.')

    # ── Ejecutar diagnóstico ──────────────────────────────────────────────────
    diag = ArduinoDiagnostic(
        port=port,
        baud=args.baud,
        gps_port=args.gps_port,
        skip_motors=args.skip_motors,
    )

    if not diag.connect():
        sys.exit(1)

    if args.monitor or args.monitor_only:
        diag.run_monitor()
        diag.close()
        sys.exit(0)

    # Suite completa de diagnóstico
    diag.test_firmware()
    diag.test_encoders()
    diag.test_imu()
    diag.test_gps()
    diag.test_motors()

    diag.print_summary()

    # Ofrecer monitor
    try:
        answer = input(f'\n  ¿Iniciar monitor en tiempo real? [S/n]: ').strip().lower()
        if answer not in ('n', 'no'):
            diag.run_monitor()
    except (EOFError, KeyboardInterrupt):
        pass

    diag.close()
    all_ok = all(v is not False for v in diag.results.values())
    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
