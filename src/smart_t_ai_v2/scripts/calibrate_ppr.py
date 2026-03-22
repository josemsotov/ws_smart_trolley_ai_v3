#!/usr/bin/env python3
"""
Calibración experimental de PPR de encoders del robot.

Metodología:
  - Hall sensor = 45 PPR (referencia física conocida)
  - Hace girar cada rueda individualmente TARGET_REVS vueltas
  - Cuenta pulsos Hall y Opto por separado (comando 'f')
  - Calcula PPR efectivo real = totalPulsos / revoluciones

Fórmula de velocidad del firmware (ROS2_Bridge.h):
  v_left  = linear + angular * wb/2
  v_right = linear - angular * wb/2  (wb = 0.48 m)

Para rueda IZQ sola: linear = V/2, angular = +V/wb
Para rueda DER sola: linear = V/2, angular = -V/wb
"""

import serial
import time
import sys

# ─── Configuración ───────────────────────────────────────────────────────────
PORT        = '/dev/ttyACM0'
BAUD        = 115200
HALL_PPR_REF  = 45          # Hall sensor = 45 PPR (referencia conocida)
TARGET_REVS   = 5           # Revoluciones por rueda (más = más preciso)
TARGET_HALL   = HALL_PPR_REF * TARGET_REVS   # 225 pulsos Hall = 5 rev
WHEEL_BASE    = 0.48        # m, separación entre ruedas (wb)
V_WHEEL       = 0.15        # m/s — velocidad de cada rueda (lento, preciso)
TIMEOUT_S     = 120         # s  — timeout máximo por prueba

# Comandos calculados para rueda individual
V_LIN = V_WHEEL / 2                    # 0.075 m/s
V_ANG = V_WHEEL / WHEEL_BASE           # 0.3125 rad/s

CMD_LEFT  = f"v {V_LIN:.4f} {+V_ANG:.4f}\n"   # v_L=V_WHEEL, v_R=0
CMD_RIGHT = f"v {V_LIN:.4f} {-V_ANG:.4f}\n"   # v_L=0, v_R=V_WHEEL
CMD_STOP  = "v 0.0 0.0\n"

# ─── Utilidades seriales ──────────────────────────────────────────────────────

def drain(s, delay=0.12):
    """Espera delay s y descarta todo lo que llegó (debug del firmware)."""
    time.sleep(delay)
    s.read(s.in_waiting or 1)

def send_vel(s, cmd):
    """Envía comando de velocidad y descarta respuesta de debug."""
    s.write(cmd.encode())
    drain(s, 0.15)

def read_f(s, retries=3):
    """
    Envía comando 'f' y retorna dict con pulsos Hall/Opto separados.
    Formato firmware: f hallL optoL hallR optoR totalL totalR
    """
    for _ in range(retries):
        s.flushInput()
        s.write(b'f\n')
        deadline = time.time() + 1.0
        buf = ''
        while time.time() < deadline:
            if s.in_waiting:
                buf += s.read(s.in_waiting).decode('utf-8', errors='replace')
            for line in buf.split('\n'):
                line = line.strip()
                if line.startswith('f '):
                    parts = line.split()
                    if len(parts) == 7:
                        try:
                            return {
                                'hallL' : int(parts[1]),
                                'optoL' : int(parts[2]),
                                'hallR' : int(parts[3]),
                                'optoR' : int(parts[4]),
                                'totalL': int(parts[5]),
                                'totalR': int(parts[6]),
                            }
                        except ValueError:
                            pass
            time.sleep(0.04)
    return None

# ─── Prueba de una rueda ──────────────────────────────────────────────────────

def test_wheel(s, side):
    """
    Gira la rueda 'side' ('IZQ' o 'DER') durante TARGET_REVS vueltas
    usando el conteo de Hall como referencia de revolución.
    Retorna PPR efectivo medido.
    """
    if side == 'IZQ':
        vel_cmd  = CMD_LEFT
        hk, ok, tk = 'hallL', 'optoL', 'totalL'
        ang_str  = f"+{V_ANG:.4f}"
    else:
        vel_cmd  = CMD_RIGHT
        hk, ok, tk = 'hallR', 'optoR', 'totalR'
        ang_str  = f"-{V_ANG:.4f}"

    print(f"\n{'═'*58}")
    print(f"  🔧  PRUEBA RUEDA {side}")
    print(f"      Objetivo  : {TARGET_REVS} rev  ({TARGET_HALL} pulsos Hall)")
    print(f"      Comando   : v {V_LIN:.4f} {ang_str}  → v_{side}={V_WHEEL} m/s, v_otra=0")
    print(f"{'═'*58}")

    # ── Reset contadores ──────────────────────────────────────────────────
    s.write(b'r\n')
    drain(s, 0.4)

    d0 = read_f(s)
    if d0:
        print(f"  Reset OK → hallL={d0['hallL']} optoL={d0['optoL']} "
              f"hallR={d0['hallR']} optoR={d0['optoR']}")
    else:
        print("  ⚠ No se pudo verificar reset")

    # ── Arrancar motor ────────────────────────────────────────────────────
    print(f"\n  ▶ Arrancando rueda {side} a {V_WHEEL} m/s ...\n")
    send_vel(s, vel_cmd)

    t_start   = time.time()
    t_last_v  = t_start
    last_hall = -1

    print(f"  {'t(s)':>6}  {'Hall':>5}  {'Opto':>5}  {'Total':>6}  {'Rev':>5}  {'PPReff':>7}")
    print(f"  {'─'*48}")

    hall_final = opto_final = total_final = 0

    while True:
        now     = time.time()
        elapsed = now - t_start

        # ── Re-enviar velocidad antes del timeout (1 000 ms firmware) ──
        if now - t_last_v > 0.5:
            send_vel(s, vel_cmd)
            t_last_v = time.time()

        # ── Leer encoders ──────────────────────────────────────────────
        d = read_f(s)
        if d is None:
            if elapsed > TIMEOUT_S:
                print(f"\n  ⚠ TIMEOUT sin respuesta del firmware!")
                break
            time.sleep(0.05)
            continue

        hall  = d[hk];  opto  = d[ok];  total = d[tk]
        revs  = hall / HALL_PPR_REF
        ppr_e = total / revs if revs > 0 else 0.0

        hall_final  = hall
        opto_final  = opto
        total_final = total

        # Mostrar solo cuando cambia Hall (evita spam)
        if hall != last_hall:
            print(f"  {elapsed:>6.1f}  {hall:>5}  {opto:>5}  {total:>6}  {revs:>5.2f}  {ppr_e:>7.1f}")
            last_hall = hall

        # ── Objetivo alcanzado ─────────────────────────────────────────
        if hall >= TARGET_HALL:
            break

        # ── Timeout ────────────────────────────────────────────────────
        if elapsed > TIMEOUT_S:
            print(f"\n  ⚠ TIMEOUT!  Hall={hall}/{TARGET_HALL}  elapsed={elapsed:.1f}s")
            break

    # ── Parar motor ───────────────────────────────────────────────────────
    send_vel(s, CMD_STOP)
    time.sleep(0.6)

    # Lectura final (captura pulsos de inercia residual)
    d_fin = read_f(s)
    elapsed_total = time.time() - t_start

    if d_fin:
        hall_final  = d_fin[hk]
        opto_final  = d_fin[ok]
        total_final = d_fin[tk]

    revs_f   = hall_final / HALL_PPR_REF if hall_final > 0 else 0
    ppr_eff  = total_final / revs_f      if revs_f  > 0 else 0
    opto_ppr = opto_final  / revs_f      if revs_f  > 0 else 0

    print(f"\n  {'─'*48}")
    print(f"  📊  RESULTADO RUEDA {side}:")
    print(f"      Tiempo         : {elapsed_total:.1f} s")
    print(f"      Hall pulsos    : {hall_final:4d}   → {revs_f:.3f} rev")
    print(f"      Opto pulsos    : {opto_final:4d}   → {opto_ppr:.1f} PPR equivalente")
    print(f"      Total pulsos   : {total_final:4d}")
    print(f"      PPR efectivo   : {ppr_eff:.2f}")

    if opto_final == 0:
        print(f"      ⚠  Optoencoders NO contribuyen  → PPR real = {HALL_PPR_REF}")
    else:
        pct_h = hall_final  / total_final * 100
        pct_o = opto_final  / total_final * 100
        print(f"      ✓  Hall {pct_h:.0f}%  +  Opto {pct_o:.0f}%  → PPR real = {ppr_eff:.1f}")

    return ppr_eff


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    print()
    print("═" * 58)
    print("  CALIBRACIÓN EXPERIMENTAL DE PPR — ENCODERS ROBOT")
    print("═" * 58)
    print(f"  Hall PPR (referencia física) : {HALL_PPR_REF} PPR")
    print(f"  Revoluciones por prueba      : {TARGET_REVS}  rev  ({TARGET_HALL} pulsos)")
    print(f"  Velocidad rueda              : {V_WHEEL} m/s  (lento para precisión)")
    print(f"  Puerto                       : {PORT} @ {BAUD}")
    print("═" * 58)

    # ── Abrir puerto serial ───────────────────────────────────────────────
    try:
        s = serial.Serial(PORT, BAUD, timeout=2)
    except serial.SerialException as e:
        print(f"\n  ✗ Error puerto serial: {e}")
        sys.exit(1)

    time.sleep(2.5)
    s.flushInput()

    # Verificar conexión con reset inicial
    s.write(b'r\n')
    drain(s, 0.4)
    print(f"  Conexión establecida  ({PORT} @ {BAUD})")
    print()
    print("  ⚠  ADVERTENCIA: El robot girará cada rueda individualmente.")
    print("     Asegúrate de que las ruedas estén elevadas o el robot")
    print("     tenga espacio para rotar sobre sí mismo.")
    print()
    input("  Pulsa ENTER para iniciar la calibración... ")

    ppr_izq = ppr_der = None

    try:
        # ── Prueba rueda IZQ ──────────────────────────────────────────────
        ppr_izq = test_wheel(s, 'IZQ')

        print(f"\n  ⏸  Pausa 3 s entre pruebas ...")
        time.sleep(3)

        # ── Prueba rueda DER ──────────────────────────────────────────────
        ppr_der = test_wheel(s, 'DER')

    finally:
        send_vel(s, CMD_STOP)
        drain(s, 0.3)
        s.close()

    # ── Resumen ───────────────────────────────────────────────────────────
    print(f"\n{'═'*58}")
    print(f"  RESUMEN CALIBRACIÓN")
    print(f"{'═'*58}")

    # Filtrar mediciones válidas (PPR=0 significa fallo de sensor, no medición real)
    valid = {k: v for k, v in [('IZQ', ppr_izq), ('DER', ppr_der)]
             if v is not None and v > 5}
    failed = [k for k, v in [('IZQ', ppr_izq), ('DER', ppr_der)]
              if v is None or v <= 5]

    if failed:
        print(f"  ⚠ Ruedas con sensor Hall FALLIDO: {', '.join(failed)}")
        print(f"    → Hall desconectado/cable suelto — no usadas en el cálculo")
        print()

    if valid:
        ppr_avg = sum(valid.values()) / len(valid)
        for side, ppr in [('IZQ', ppr_izq), ('DER', ppr_der)]:
            if ppr is not None and ppr > 5:
                print(f"  PPR Rueda {side}  : {ppr:.2f}  ✓")
            else:
                print(f"  PPR Rueda {side}  : ---     ⚠ sensor fallido")
        print(f"  PPR Promedio   : {ppr_avg:.2f}  (de {len(valid)} rueda/s válida/s)")
        print()

        if abs(ppr_avg - 45) < 5:
            rec_ppr = 45.0
            print(f"  ✓ CONFIRMADO: Solo Hall funciona (Opto=0)")
        elif abs(ppr_avg - 85) < 8:
            rec_ppr = 85.0
            print(f"  ✓ CONFIRMADO: Hall + Opto funcionan")
        else:
            rec_ppr = round(ppr_avg, 1)
            print(f"  ⚠ PPR medido fuera de rango esperado (45 o 85)")

        print()
        print(f"  → Configurar en config/arduino_params.yaml:")
        print(f"    ppr: {rec_ppr}")
        print()
        print(f"  → Si es necesario reconstruir el workspace:")
        print(f"    cd ~/WS/WorkSpace/ws_smart_trolley_ai_v2")
        print(f"    colcon build --packages-select smart_t_ai_v2")

    print(f"{'═'*58}")
    print("  Motor parado. Calibración completada.")


if __name__ == '__main__':
    main()
