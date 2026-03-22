#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════
  CALIBRACIÓN DEL CONTROL GOOGLE STADIA - Smart Trolley V5
═══════════════════════════════════════════════════════════════════════
  Lee /dev/input/js0 directamente (sin ROS).
  Te pide una acción a la vez, confirma, y sigue al siguiente paso.
  Al final genera stadia_teleop.yaml automáticamente.

  Uso:  python3 test_stadia_calibration.py
  (ejecutar directamente en la terminal del robot)
═══════════════════════════════════════════════════════════════════════
"""
import struct, sys, os, time, select

JS_DEVICE = '/dev/input/js0'
JS_EVENT_SIZE = 8
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS   = 0x02
JS_EVENT_INIT   = 0x80

C_G = '\033[92m'
C_Y = '\033[93m'
C_R = '\033[91m'
C_C = '\033[96m'
C_B = '\033[1m'
C_D = '\033[2m'
C_0 = '\033[0m'


def flush(js):
    while select.select([js], [], [], 0.0)[0]:
        js.read(JS_EVENT_SIZE)


def read_events(js, seconds=8.0):
    flush(js)
    evts = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        if select.select([js], [], [], 0.1)[0]:
            data = js.read(JS_EVENT_SIZE)
            _, val, etype, num = struct.unpack('IhBB', data)
            if etype & JS_EVENT_INIT:
                continue
            evts.append((etype, num, val))
    return evts


def detect_button(evts):
    for etype, num, val in evts:
        if etype == JS_EVENT_BUTTON and val == 1:
            return {'type': 'button', 'index': num}
    for etype, num, val in evts:
        if etype == JS_EVENT_AXIS and abs(val) > 20000:
            return {'type': 'axis', 'index': num, 'value': val}
    return None


def detect_axis(evts):
    axis_max = {}
    for etype, num, val in evts:
        if etype == JS_EVENT_AXIS:
            if abs(val) > abs(axis_max.get(num, 0)):
                axis_max[num] = val
    if not axis_max:
        return None
    best = max(axis_max, key=lambda k: abs(axis_max[k]))
    if abs(axis_max[best]) > 5000:
        return {'index': best, 'max_value': axis_max[best]}
    return None


def ask(js, prompt, mode='button', secs=8):
    print(f"\n  {C_Y}▶ {prompt}{C_0}")
    print(f"    {C_D}Tienes {secs} seg. Hazlo AHORA...{C_0}", end='', flush=True)
    evts = read_events(js, secs)
    if not evts:
        print(f"\r    {C_R}✘ Nada detectado (timeout)                    {C_0}")
        return None
    if mode == 'button':
        r = detect_button(evts)
        if r is None:
            print(f"\r    {C_R}✘ Ningún botón detectado                   {C_0}")
            return None
        if r['type'] == 'button':
            print(f"\r    {C_G}✔ BOTÓN {r['index']}                            {C_0}")
        else:
            print(f"\r    {C_G}✔ EJE {r['index']} (analógico, val={r.get('value',0)})     {C_0}")
        return r
    else:
        r = detect_axis(evts)
        if r is None:
            print(f"\r    {C_R}✘ Ningún eje detectado                     {C_0}")
            return None
        v = r['max_value']
        print(f"\r    {C_G}✔ EJE {r['index']} (valor={v:+d})                  {C_0}")
        return r


def main():
    if not os.path.exists(JS_DEVICE):
        print(f"\n{C_R}ERROR: {JS_DEVICE} no existe{C_0}")
        sys.exit(1)

    js = open(JS_DEVICE, 'rb')
    time.sleep(0.3)
    flush(js)

    print(f"\n{C_B}{C_C}{'='*60}{C_0}")
    print(f"{C_B}{C_C}  CALIBRACIÓN STADIA - Smart Trolley V5{C_0}")
    print(f"{C_B}{C_C}{'='*60}{C_0}")
    print(f"\n  Dispositivo: {JS_DEVICE}")
    print(f"  Sigue las instrucciones paso a paso.")

    results = {}

    # FASE 1: BOTONES
    print(f"\n{C_B}{C_C}── FASE 1: BOTONES DE HOMBRO ──{C_0}")
    input(f"\n  {C_D}Presiona ENTER cuando estés listo...{C_0}")

    for name, desc in [
        ('L1', 'Presiona L1 (bumper SUPERIOR IZQUIERDO)'),
        ('R1', 'Presiona R1 (bumper SUPERIOR DERECHO)'),
        ('L2', 'Presiona L2 (gatillo INFERIOR IZQUIERDO) a fondo'),
        ('R2', 'Presiona R2 (gatillo INFERIOR DERECHO) a fondo'),
    ]:
        results[name] = ask(js, desc, 'button')
        time.sleep(1.5)
        flush(js)

    # FASE 2: STICKS
    print(f"\n{C_B}{C_C}── FASE 2: STICKS ──{C_0}")
    input(f"\n  {C_D}Presiona ENTER cuando estés listo...{C_0}")

    results['LSY'] = ask(js, "Stick IZQUIERDO → ADELANTE (arriba) al máximo y suelta", 'axis')
    time.sleep(1.5)
    flush(js)

    results['LSX'] = ask(js, "Stick IZQUIERDO → IZQUIERDA al máximo y suelta", 'axis')
    time.sleep(1.5)
    flush(js)

    # FASE 3: BOTONES CARA
    print(f"\n{C_B}{C_C}── FASE 3: BOTONES CARA (5s cada uno) ──{C_0}")
    for name, desc in [('A','Presiona A'), ('B','Presiona B'),
                        ('X','Presiona X'), ('Y','Presiona Y')]:
        results[name] = ask(js, desc, 'button', secs=5)
        time.sleep(0.8)
        flush(js)

    js.close()

    # RESUMEN
    print(f"\n{C_B}{C_C}{'='*60}{C_0}")
    print(f"{C_B}{C_C}  RESUMEN{C_0}")
    print(f"{C_B}{C_C}{'='*60}{C_0}")

    print(f"\n  {C_B}Hombro:{C_0}")
    for n in ['L1','R1','L2','R2']:
        r = results.get(n)
        if r and r.get('type') == 'button':
            print(f"    {n}: {C_G}btn {r['index']}{C_0}")
        elif r and r.get('type') == 'axis':
            print(f"    {n}: {C_Y}eje {r['index']} (analógico){C_0}")
        else:
            print(f"    {n}: {C_R}no detectado{C_0}")

    print(f"\n  {C_B}Sticks:{C_0}")
    for n, label in [('LSY','Izq Adelante'), ('LSX','Izq Izquierda')]:
        r = results.get(n)
        if r:
            print(f"    {label}: {C_G}eje {r['index']} (val={r['max_value']:+d}){C_0}")
        else:
            print(f"    {label}: {C_R}no detectado{C_0}")

    print(f"\n  {C_B}Cara:{C_0}")
    for n in ['A','B','X','Y']:
        r = results.get(n)
        if r and r.get('type') == 'button':
            print(f"    {n}: {C_G}btn {r['index']}{C_0}")
        else:
            print(f"    {n}: {C_D}(skip){C_0}")

    # GENERAR YAML
    l1 = results.get('L1') or {}
    r1 = results.get('R1') or {}
    lsy = results.get('LSY') or {}
    lsx = results.get('LSX') or {}

    enable_btn = l1.get('index', -1) if l1.get('type') == 'button' else -1
    turbo_btn  = r1.get('index', -1) if r1.get('type') == 'button' else -1

    if enable_btn < 0:
        print(f"\n  {C_R}⚠ L1 no es botón digital.{C_0}")
        try:
            enable_btn = int(input(f"  {C_Y}Índice para L1 (enable): {C_0}"))
        except:
            enable_btn = 5
    if turbo_btn < 0:
        print(f"\n  {C_R}⚠ R1 no es botón digital.{C_0}")
        try:
            turbo_btn = int(input(f"  {C_Y}Índice para R1 (turbo): {C_0}"))
        except:
            turbo_btn = 14

    axis_lin = lsy.get('index', 1)
    axis_ang = lsx.get('index', 0)
    fwd_val  = lsy.get('max_value', 1)
    left_val = lsx.get('max_value', 1)

    s_lin = 0.3 if fwd_val > 0 else -0.3
    s_ang = 0.6 if left_val > 0 else -0.6
    t_lin = round(s_lin / 0.3 * 0.5, 1)
    t_ang = round(s_ang / 0.6 * 1.0, 1)

    btn_lines = []
    for n in ['L1','R1','L2','R2','A','B','X','Y']:
        r = results.get(n)
        if r and r.get('type') == 'button':
            btn_lines.append(f"#    {n} = btn {r['index']}")
        elif r and r.get('type') == 'axis':
            btn_lines.append(f"#    {n} = eje {r['index']}")

    yaml = f"""# ═══════════════════════════════════════════════════════════
#  Google Stadia Controller - Teleop config (Bluetooth)
#  Generado: {time.strftime('%Y-%m-%d %H:%M:%S')}
#
#  Mapeo calibrado:
{chr(10).join(btn_lines)}
#    Stick Izq Y = eje {axis_lin} (adelante={fwd_val:+d})
#    Stick Izq X = eje {axis_ang} (izquierda={left_val:+d})
#
#  L1 (mantener) + Stick = drive normal
#  R1 (mantener) + Stick = drive turbo
#  Soltar = PARAR (deadman)
# ═══════════════════════════════════════════════════════════

teleop_twist_joy:
  ros__parameters:
    axis_linear:
      x: {axis_lin}
    axis_angular:
      yaw: {axis_ang}
    scale_linear:
      x: {s_lin}
    scale_angular:
      yaw: {s_ang}
    scale_linear_turbo:
      x: {t_lin}
    scale_angular_turbo:
      yaw: {t_ang}
    enable_button: {enable_btn}
    enable_turbo_button: {turbo_btn}
    publish_stamped_twist: false
    require_enable_button: true

joy_node:
  ros__parameters:
    device_id: 0
    deadzone: 0.12
    autorepeat_rate: 20.0
"""

    print(f"\n{C_B}{C_C}── CONFIGURACIÓN ──{C_0}")
    print(f"  L1 enable : btn {C_G}{enable_btn}{C_0}")
    print(f"  R1 turbo  : btn {C_G}{turbo_btn}{C_0}")
    print(f"  Linear    : eje {C_G}{axis_lin}{C_0}, scale={s_lin}")
    print(f"  Angular   : eje {C_G}{axis_ang}{C_0}, scale={s_ang}")

    confirm = input(f"\n  {C_Y}¿Guardar? (s/n): {C_0}").strip().lower()
    if confirm in ('s','si','sí','y','yes',''):
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'src','smart_t_ai_v2','config','stadia_teleop.yaml')
        with open(p, 'w') as f:
            f.write(yaml)
        print(f"\n  {C_G}✔ Guardado: {p}{C_0}")
        print(f"\n  {C_B}Para probar:{C_0}")
        print(f"    ros2 launch smart_t_ai_v2 launch_robot.launch.py \\")
        print(f"        use_lidar:=false use_camera:=false use_rviz:=false\n")
    else:
        print(f"\n  {C_Y}No guardado.{C_0}\n")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n\n  {C_Y}Cancelado.{C_0}\n")
    except PermissionError:
        print(f"\n{C_R}ERROR: sin permiso para {JS_DEVICE}{C_0}")
        print(f"  sudo chmod 666 {JS_DEVICE}")
