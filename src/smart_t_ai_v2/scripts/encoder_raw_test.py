#!/usr/bin/env python3
"""
encoder_raw_test.py — Prueba directa de encoders sin ROS
Envía comando de movimiento real y monitorea la respuesta 'e L R' del firmware.

USO (sin ros, sin build):
  python3 src/smart_t_ai_v2/scripts/encoder_raw_test.py

El script NO requiere arduino_bridge ni ROS corriendo.
Abre el puerto serie directamente.
"""

import serial
import time
import sys

PORT  = '/dev/ttyACM0'
BAUD  = 115200
SPEED = 0.10   # m/s — velocidad de prueba (segura)
DURATION = 5.0  # segundos de movimiento


def run():
    print(f"\n{'═'*62}")
    print(" ENCODER RAW TEST — Smart Trolley (sin ROS)")
    print(f"{'═'*62}")
    print(f" Puerto : {PORT} @ {BAUD}")
    print(f" Velocidad de prueba : {SPEED} m/s durante {DURATION} s")
    print(f"{'─'*62}\n")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.2)
    except serial.SerialException as e:
        print(f"❌  No se pudo abrir {PORT}: {e}")
        sys.exit(1)

    print("⏳  Esperando reset del Arduino (2 s)…")
    time.sleep(2.0)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # ── Leer encoders en reposo ──────────────────────────────────────
    print("\n1) Encoders en REPOSO (antes de mover):")
    ser.write(b'e\n')
    ser.flush()
    time.sleep(0.15)
    while ser.in_waiting:
        raw = ser.readline().decode('utf-8', errors='replace').strip()
        print(f"   << {raw}")

    # ── Mandar velocidad y monitorear encoders ───────────────────────
    print(f"\n2) Enviando  v {SPEED:.2f} 0.00  →  monitoreando encoders…")
    print(f"   {'Tiempo':>7}   {'Respuesta del Arduino'}")
    print(f"   {'─'*7}   {'─'*40}")

    cmd_v = f"v {SPEED:.3f} 0.000\n".encode()
    cmd_e = b'e\n'

    left_prev  = None
    right_prev = None
    left_max   = 0
    right_max  = 0
    enc_samples = 0
    enc_moving  = 0

    t0 = time.time()
    last_enc = 0.0

    while time.time() - t0 < DURATION:
        elapsed = time.time() - t0

        # Enviar velocidad cada 100 ms
        ser.write(cmd_v)
        ser.flush()

        # Pedir encoders cada 200 ms
        if elapsed - last_enc >= 0.2:
            last_enc = elapsed
            ser.write(cmd_e)
            ser.flush()

        # Leer TODO lo que llegue del Arduino
        time.sleep(0.05)
        while ser.in_waiting:
            raw = ser.readline().decode('utf-8', errors='replace').strip()
            if not raw:
                continue

            if raw.startswith('e '):
                parts = raw.split()
                if len(parts) == 3:
                    try:
                        L = int(parts[1])
                        R = int(parts[2])
                        enc_samples += 1
                        left_max  = max(left_max,  abs(L))
                        right_max = max(right_max, abs(R))
                        changed = ''
                        if left_prev is not None:
                            dL = L - left_prev
                            dR = R - right_prev
                            if dL != 0 or dR != 0:
                                enc_moving += 1
                                changed = f'  ΔL={dL:+d} ΔR={dR:+d}  ← CAMBIO ✓'
                        left_prev  = L
                        right_prev = R
                        print(f"   {elapsed:6.2f}s   e {L:6d} {R:6d}{changed}")
                    except ValueError:
                        print(f"   {elapsed:6.2f}s   [mal formato] {raw}")
                else:
                    print(f"   {elapsed:6.2f}s   [partes={len(parts)}] {raw}")
            else:
                # Mensajes de debug del firmware (no son encoders)
                print(f"   {elapsed:6.2f}s   [fw] {raw}")

    # ── Parar robot ──────────────────────────────────────────────────
    ser.write(b'v 0.000 0.000\n')
    ser.flush()
    time.sleep(0.3)
    # Drenar mensajes finales
    while ser.in_waiting:
        raw = ser.readline().decode('utf-8', errors='replace').strip()
        if raw:
            print(f"   [stop] {raw}")

    ser.close()

    # ── Reporte final ────────────────────────────────────────────────
    print(f"\n{'═'*62}")
    print(" RESULTADO:")
    print(f"{'─'*62}")
    print(f"  Muestras de encoder recibidas : {enc_samples}")
    print(f"  Muestras con cambio (ΔL o ΔR) : {enc_moving}")
    print(f"  Máximo absoluto Left encoder  : {left_max}")
    print(f"  Máximo absoluto Right encoder : {right_max}")
    print(f"{'─'*62}")

    if enc_samples == 0:
        print("  ❌  El firmware NO respondió 'e L R' al comando 'e\\n'")
        print("      → Protocolo diferente o firmware no tiene comando 'e'")
        print("      → Revisa el firmware Arduino (busca el handler de 'e')")
    elif enc_moving == 0 and left_max == 0 and right_max == 0:
        print("  ❌  Encoders siempre devuelven e 0 0 durante el movimiento")
        print("      Posibles causas:")
        print("      A) El firmware RESETEA el contador en cada petición 'e'")
        print("         (envía DELTA en vez de acumulado — bug de protocolo)")
        print("      B) ISR del encoder no está habilitada en el firmware")
        print("         (attachInterrupt() no se llamó en setup())")
        print("      C) Las señales Hall llegan pero a pines sin interrupción")
        print("         (solo pines 2,3,18,19,20,21 soportan INT en Mega)")
    elif enc_moving == 0 and (left_max > 0 or right_max > 0):
        print("  ⚠️   Encoders mostraron valores ≠ 0 pero sin incremento")
        print("      → Firmware envía DELTA (no acumulado)")
        print(f"     → PPR aparente por muestra: L_max={left_max} R_max={right_max}")
        ppr_est = max(left_max, right_max)
        print(f"     → Si {DURATION}s a {SPEED} m/s ≈ {SPEED*DURATION:.2f} m,")
        n_rev = (SPEED * DURATION) / (2 * 3.14159 * 0.10)
        print(f"       n_rev ≈ {n_rev:.2f} rev → PPR_est = max_count / n_rev ≈ {ppr_est/n_rev:.0f}")
    else:
        n_rev = (SPEED * DURATION) / (2 * 3.14159 * 0.10)
        avg_ppr = ((left_max + right_max) / 2) / n_rev if n_rev > 0 else 0
        print(f"  ✅  Encoders FUNCIONAN — cuentan durante el movimiento")
        print(f"      L_max={left_max}  R_max={right_max}")
        print(f"      Revoluciones estimadas ({SPEED} m/s × {DURATION}s): {n_rev:.2f} rev")
        print(f"      PPR estimado: {avg_ppr:.0f}  (referencia: 45 Hall)")

    print(f"{'═'*62}\n")


if __name__ == '__main__':
    if len(sys.argv) > 1:
        PORT = sys.argv[1]
    run()
