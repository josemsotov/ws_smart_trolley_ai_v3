#!/usr/bin/env python3
"""
opto_ppr_confirm.py — Smart Trolley V5
Confirmación experimental del PPR del encoder óptico y calibración del PPR total.

═══════════════════════════════════════════════════════════════════
FUNDAMENTO MATEMÁTICO
─────────────────────────────────────────────────────────────────
Sean N las revoluciones físicas que hace una rueda durante el test:

  Fase 1 (Hall, ppr=45):
    dist_A = N × 2π × r   ← referencia exacta (Hall PPR = 45, conocido)

  Fase 2 (Hall+Opto, trial_ppr=117 por defecto):
    dist_B = (ppr_real / trial_ppr) × N × 2π × r

  Por tanto:
    ppr_real = trial_ppr × (dist_B / dist_A)
    opto_ppr = ppr_real - hall_ppr (45)

Si dist_B == dist_A  →  trial_ppr es correcto  → 72 PPR confirmado.
Si dist_B  < dist_A  →  ppr_real < trial_ppr   → opto tiene MENOS pulsos.
Si dist_B  > dist_A  →  ppr_real > trial_ppr   → opto tiene MÁS pulsos.

El script corre en 2 fases separadas.  Los datos de la Fase 1 se
guardan en /tmp/opto_ppr_phase1.json para que la Fase 2 los use.
═══════════════════════════════════════════════════════════════════

Uso completo:

  # ── Fase 1: referencia Hall (45 PPR) ─────────────────────────
  # Terminal A  (arduino_bridge Profile A):
  ros2 launch smart_t_ai_v2 opto_ppr_test.launch.py profile:=A

  # ── Fase 2: Hall + Opto combinados (trial PPR = 117) ─────────
  # Terminal A  (arduino_bridge Profile B):
  ros2 launch smart_t_ai_v2 opto_ppr_test.launch.py profile:=B

  # También puedes lanzar solo el nodo (si la bridge ya corre):
  ros2 run smart_t_ai_v2 opto_ppr_confirm.py --ros-args -p phase:=1
  ros2 run smart_t_ai_v2 opto_ppr_confirm.py --ros-args -p phase:=2 -p trial_ppr:=117
"""

import json
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# Ruta donde se guardan los datos de la Fase 1
PHASE1_FILE = '/tmp/opto_ppr_phase1.json'
RESULTS_FILE = '/tmp/opto_ppr_results.json'

BAR  = '═' * 62
LINE = '─' * 62


class OptoPPRConfirm(Node):
    """
    Nodo de calibración de PPR del encoder óptico.

    Parámetros ROS:
      phase        (int, default=1)    Fase a ejecutar: 1 = Hall ref, 2 = combinado
      trial_ppr    (int, default=117)  PPR total de prueba (hall_ppr + opto_ppr_estimado)
      hall_ppr     (int, default=45)   PPR del sensor Hall (conocido, referencia)
      drive_speed  (float, default=0.10)  Velocidad de avance (m/s) — mantén < 0.15
      drive_time   (float, default=6.0)   Duración del avance (s)
      wheel_radius (float, default=0.10)  Radio de la rueda (m)
    """

    def __init__(self):
        super().__init__('opto_ppr_confirm')

        # ── Declarar parámetros ───────────────────────────────────────
        self.declare_parameter('phase',        1)       # 1 ó 2
        self.declare_parameter('trial_ppr',    117)     # 45 Hall + 72 Opto
        self.declare_parameter('hall_ppr',     45)      # referencia conocida
        self.declare_parameter('drive_speed',  0.10)    # m/s (lento para precisión)
        self.declare_parameter('drive_time',   6.0)     # segundos
        self.declare_parameter('wheel_radius', 0.10)    # metros

        self.phase        = self.get_parameter('phase').value
        self.trial_ppr    = self.get_parameter('trial_ppr').value
        self.hall_ppr     = self.get_parameter('hall_ppr').value
        self.drive_speed  = self.get_parameter('drive_speed').value
        self.drive_time   = self.get_parameter('drive_time').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # ── Estado de odometría ───────────────────────────────────────
        self._odom_lock      = threading.Lock()
        self._odom_x         = 0.0
        self._odom_y         = 0.0
        self._odom_vx        = 0.0   # velocidad lineal medida por encoders
        self._odom_start_x   = 0.0
        self._odom_start_y   = 0.0
        self._odom_received  = False

        # ── Pub / Sub ─────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # ── Hilo de test (nunca bloquear callbacks ROS) ───────────────
        self._thread = threading.Thread(target=self._main, daemon=True)
        self._thread.start()

    # ─── Callbacks ────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        with self._odom_lock:
            self._odom_x  = msg.pose.pose.position.x
            self._odom_y  = msg.pose.pose.position.y
            self._odom_vx = msg.twist.twist.linear.x
            self._odom_received = True

    # ─── Utilidades ────────────────────────────────────────────────────

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _dist_traveled(self) -> float:
        with self._odom_lock:
            dx = self._odom_x - self._odom_start_x
            dy = self._odom_y - self._odom_start_y
        return math.hypot(dx, dy)

    def _wait_odom(self, timeout: float = 10.0) -> bool:
        t0 = time.time()
        while not self._odom_received:
            if time.time() - t0 > timeout:
                self.get_logger().error(
                    '❌ No llega /odom. ¿Está activo arduino_bridge?')
                return False
            time.sleep(0.1)
        return True

    def _check_encoders_alive(self) -> bool:
        """
        Verifica que los encoders responden: mueve el robot 1 segundo
        y confirma que odom.x cambia. Devuelve True si hay señal de encoder.
        """
        self.get_logger().info(f'\n{LINE}')
        self.get_logger().info('🔍  PRE-CHECK: verificando encoders…')
        self.get_logger().info('    Moviendo a 0.10 m/s durante 1 s…')

        with self._odom_lock:
            x0 = self._odom_x

        cmd = Twist()
        cmd.linear.x = float(self.drive_speed)
        t0 = time.time()
        while (time.time() - t0) < 1.0 and rclpy.ok():
            self._cmd_pub.publish(cmd)
            time.sleep(0.05)
        self._stop()
        time.sleep(0.4)

        with self._odom_lock:
            x1 = self._odom_x
            vx = self._odom_vx

        delta = abs(x1 - x0)
        self.get_logger().info(
            f'    odom.x antes: {x0:.5f} m  →  después: {x1:.5f} m  '
            f'(Δ={delta:.5f} m)   vx medido: {vx:.4f} m/s')

        if delta < 0.001 and abs(vx) < 0.001:
            self.get_logger().error(f'{LINE}')
            self.get_logger().error(
                '❌  ENCODERS NO RESPONDEN — odom.x no cambió tras mover 1 s.')
            self.get_logger().error(
                '    Posibles causas:')
            self.get_logger().error(
                '      1. Cable del encoder Hall desconectado del Arduino')
            self.get_logger().error(
                '         (revisa los conectores junto al driver de motores)')
            self.get_logger().error(
                '      2. encoder_ppr configurado pero firmware no envía "e L R"')
            self.get_logger().error(
                '         (verifica protocolo: ros2 topic echo /arduino_status)')
            self.get_logger().error(
                '      3. Alimentación de los sensores Hall (5 V) ausente')
            self.get_logger().error(f'{LINE}')
            self.get_logger().error(
                '    ► Diagnóstico rápido sin sudo:')
            self.get_logger().error(
                '      ros2 topic echo /odom --field pose.pose.position.x')
            self.get_logger().error(
                '      (mueve el robot a mano y comprueba si el valor cambia)')
            self.get_logger().error(f'{LINE}')
            return False

        self.get_logger().info(
            f'    ✅  Encoders OK  (Δ={delta*1000:.1f} mm en 1 s)')
        self.get_logger().info(LINE)

        # Volver atrás 1 segundo para compensar el desplazamiento del pre-check
        self.get_logger().info(
            '    ↩  Retrocediendo 1 s para volver al punto de inicio…')
        cmd.linear.x = -float(self.drive_speed)
        t0 = time.time()
        while (time.time() - t0) < 1.0 and rclpy.ok():
            self._cmd_pub.publish(cmd)
            time.sleep(0.05)
        self._stop()
        time.sleep(0.5)
        return True

    def _drive_forward(self) -> float | None:
        """
        Verifica encoders, avanza drive_time segundos mostrando
        velocidad medida en tiempo real, devuelve distancia odom (m).
        """
        if not self._wait_odom():
            return None

        # Verificar que los encoders dan señal antes de medir
        if not self._check_encoders_alive():
            return None

        # Guardar posición inicial
        with self._odom_lock:
            self._odom_start_x = self._odom_x
            self._odom_start_y = self._odom_y

        expected = self.drive_speed * self.drive_time
        self.get_logger().info(
            f'\n  ▶ Avanzando {self.drive_time:.1f}s a {self.drive_speed:.2f} m/s '
            f'(nominal ≈ {expected:.3f} m)…')
        self.get_logger().info(
            '    Tiempo(s)   odom.x(m)   vx_medido(m/s)')
        self.get_logger().info('    ' + '─' * 40)

        cmd = Twist()
        cmd.linear.x = float(self.drive_speed)

        t0 = time.time()
        last_log = 0.0
        while (time.time() - t0) < self.drive_time and rclpy.ok():
            self._cmd_pub.publish(cmd)
            elapsed = time.time() - t0
            # Log cada segundo
            if elapsed - last_log >= 1.0:
                with self._odom_lock:
                    cx = self._odom_x
                    vx = self._odom_vx
                dist = self._dist_traveled()
                self.get_logger().info(
                    f'    {elapsed:6.1f} s    {cx:+.4f} m    {vx:+.4f} m/s    '
                    f'(dist={dist:.4f} m)')
                last_log = elapsed
            time.sleep(0.05)

        self._stop()
        time.sleep(0.5)   # dejar que el odom se estabilice

        return self._dist_traveled()

    # ─── Fase 1: referencia Hall ────────────────────────────────────────

    def _phase1(self):
        self.get_logger().info(f'\n{BAR}')
        self.get_logger().info('FASE 1 — REFERENCIA HALL (45 PPR)')
        self.get_logger().info(LINE)
        self.get_logger().info(
            '  arduino_bridge debe estar en:  sensor_profile:=A  encoder_ppr:=45')
        self.get_logger().info(BAR)
        self.get_logger().info(
            '⚠  Coloca el robot en la posición de SALIDA, camino despejado.')
        self.get_logger().info(
            '   El robot avanzará en línea recta.')
        time.sleep(4.0)

        dist_A = self._drive_forward()
        if dist_A is None:
            self.get_logger().error('Fase 1 falló — sin datos de odom.')
            return

        n_revs  = dist_A / (2.0 * math.pi * self.wheel_radius)
        nominal = self.drive_speed * self.drive_time

        self.get_logger().info(f'\n{LINE}')
        self.get_logger().info('RESULTADO FASE 1 — Hall referencia:')
        self.get_logger().info(
            f'  Velocidad comandada : {self.drive_speed:.3f} m/s')
        self.get_logger().info(
            f'  Tiempo             : {self.drive_time:.1f} s')
        self.get_logger().info(
            f'  Distancia nominal  : {nominal:.4f} m')
        self.get_logger().info(
            f'  Distancia odom (A) : {dist_A:.4f} m   ← referencia Hall')
        self.get_logger().info(
            f'  Revoluciones rueda : {n_revs:.3f} rev')

        # Guardar datos para Fase 2
        data = {
            'dist_A':       dist_A,
            'hall_ppr':     self.hall_ppr,
            'drive_speed':  self.drive_speed,
            'drive_time':   self.drive_time,
            'wheel_radius': self.wheel_radius,
            'n_revs':       n_revs,
        }
        with open(PHASE1_FILE, 'w') as f:
            json.dump(data, f, indent=2)

        opto_trial = self.trial_ppr - self.hall_ppr

        self.get_logger().info(f'\n✅  Fase 1 guardada → {PHASE1_FILE}')
        self.get_logger().info(f'\n{BAR}')
        self.get_logger().info('📋  SIGUIENTE PASO — FASE 2:')
        self.get_logger().info(LINE)
        self.get_logger().info(
            '  1. Lleva el robot de vuelta al PUNTO DE INICIO')
        self.get_logger().info(
            '  2. Detén el launch actual (Ctrl+C)')
        self.get_logger().info(
            '  3. Lanza la Fase 2:')
        self.get_logger().info(
            '     ros2 launch smart_t_ai_v2 opto_ppr_test.launch.py \\')
        self.get_logger().info(
            f'       profile:=B trial_ppr:={self.trial_ppr}')
        self.get_logger().info(
            f'     (prueba: Hall {self.hall_ppr} + Opto {opto_trial} = {self.trial_ppr} total)')
        self.get_logger().info(BAR)

    # ─── Fase 2: Hall + Opto combinados ────────────────────────────────

    def _phase2(self):
        # Cargar datos de Fase 1
        if not os.path.exists(PHASE1_FILE):
            self.get_logger().error(
                f'❌  No existe {PHASE1_FILE}. Ejecuta la Fase 1 primero.')
            return

        with open(PHASE1_FILE) as f:
            p1 = json.load(f)

        dist_A      = p1['dist_A']
        hall_ppr    = p1.get('hall_ppr', 45)
        n_revs_ref  = p1.get('n_revs', dist_A / (2.0 * math.pi * self.wheel_radius))
        opto_trial  = self.trial_ppr - hall_ppr

        self.get_logger().info(f'\n{BAR}')
        self.get_logger().info('FASE 2 — HALL + OPTO COMBINADOS')
        self.get_logger().info(LINE)
        self.get_logger().info(
            f'  Trial PPR: {self.trial_ppr}  '
            f'(Hall {hall_ppr} + Opto {opto_trial})')
        self.get_logger().info(
            f'  Referencia Fase 1: dist_A = {dist_A:.4f} m  '
            f'({n_revs_ref:.3f} rev)')
        self.get_logger().info(
            '  arduino_bridge debe estar en:  '
            f'sensor_profile:=B  encoder_ppr:={self.trial_ppr}')
        self.get_logger().info(BAR)
        self.get_logger().info(
            '⚠  Coloca el robot en la MISMA posición de inicio que Fase 1.')
        time.sleep(4.0)

        dist_B = self._drive_forward()
        if dist_B is None:
            self.get_logger().error('Fase 2 falló — sin datos de odom.')
            return

        # ── Cálculo PPR real ─────────────────────────────────────────
        #
        # Para las mismas N revoluciones físicas:
        #   dist_A = N × 2π × r               (Hall, siempre exacto)
        #   dist_B = (ppr_real / trial_ppr)
        #            × N × 2π × r
        #
        # → ppr_real = trial_ppr × (dist_B / dist_A)
        #
        if dist_A <= 0.0:
            self.get_logger().error('dist_A = 0, no se puede dividir.')
            return

        ratio      = dist_B / dist_A
        ppr_real   = self.trial_ppr * ratio
        opto_real  = ppr_real - hall_ppr

        err_pct = abs(ppr_real - self.trial_ppr) / self.trial_ppr * 100.0
        confirmed = err_pct < 5.0   # ±5 % → confirmado

        resolution_mm = (2.0 * math.pi * self.wheel_radius / ppr_real) * 1000.0
        improvement   = ppr_real / hall_ppr

        self.get_logger().info(f'\n{BAR}')
        self.get_logger().info('══  RESULTADO FINAL — CONFIRMACIÓN OPTO PPR  ══')
        self.get_logger().info(LINE)
        self.get_logger().info(
            f'  Distancia Fase 1 (Hall ref, dist_A) : {dist_A:.4f} m')
        self.get_logger().info(
            f'  Distancia Fase 2 (Combinado, dist_B): {dist_B:.4f} m')
        self.get_logger().info(
            f'  Ratio  dist_B / dist_A               : {ratio:.5f}')
        self.get_logger().info(LINE)
        self.get_logger().info(
            f'  Trial total PPR        : {self.trial_ppr}  '
            f'(Hall {hall_ppr} + Opto {opto_trial} estimado)')
        self.get_logger().info(
            f'  PPR total MEDIDO       : {ppr_real:.1f}')
        self.get_logger().info(
            f'  PPR opto MEDIDO        : {opto_real:.1f}')
        self.get_logger().info(
            f'  Error respecto trial   : {err_pct:.1f} %')
        self.get_logger().info(LINE)

        if confirmed:
            ppr_rounded = round(ppr_real)
            opto_rounded = ppr_rounded - hall_ppr
            self.get_logger().info(
                f'\n✅  CONFIRMADO: Opto encoder ≈ {opto_rounded} PPR')
            self.get_logger().info(
                f'    Total = {ppr_rounded} PPR  '
                f'(Hall {hall_ppr} + Opto {opto_rounded})')
            self.get_logger().info(
                f'    Resolución lineal : {resolution_mm:.2f} mm / pulso')
            self.get_logger().info(
                f'    Mejora vs Hall solo: {improvement:.1f}× más preciso')
            self.get_logger().info(f'\n{LINE}')
            self.get_logger().info('📌  CONFIGURACIÓN RECOMENDADA para arduino_bridge:')
            self.get_logger().info(
                f'    sensor_profile:=B  encoder_ppr:={ppr_rounded}')
            self.get_logger().info(LINE)
            self.get_logger().info(
                '📌  ACTUALIZA arduino_params.yaml:')
            self.get_logger().info(
                f'    encoder_ppr: {ppr_rounded}')
            self.get_logger().info(
                f'    sensor_profile: \'B\'')
        else:
            ppr_suggested  = round(ppr_real)
            opto_suggested = ppr_suggested - hall_ppr
            self.get_logger().warn(
                f'\n⚠️   Trial PPR {self.trial_ppr} NO confirmado  '
                f'(error = {err_pct:.1f} %)')
            self.get_logger().warn(
                f'    PPR real calculado : {ppr_real:.1f}')
            self.get_logger().warn(
                f'    Opto real calculado: {opto_real:.1f}')
            self.get_logger().warn(
                f'\n📌  Prueba de nuevo con trial_ppr:={ppr_suggested}')
            self.get_logger().warn(
                f'    ros2 launch smart_t_ai_v2 opto_ppr_test.launch.py \\')
            self.get_logger().warn(
                f'      profile:=B trial_ppr:={ppr_suggested}')

        self.get_logger().info(BAR)

        # Guardar resultados JSON
        results = {
            'trial_ppr':    self.trial_ppr,
            'hall_ppr':     hall_ppr,
            'opto_trial':   opto_trial,
            'dist_A':       round(dist_A, 5),
            'dist_B':       round(dist_B, 5),
            'ratio':        round(ratio, 5),
            'ppr_real':     round(ppr_real, 2),
            'opto_real':    round(opto_real, 2),
            'error_pct':    round(err_pct, 2),
            'confirmed':    confirmed,
            'resolution_mm': round(resolution_mm, 3),
        }
        with open(RESULTS_FILE, 'w') as f:
            json.dump(results, f, indent=2)
        self.get_logger().info(f'  Resultados JSON → {RESULTS_FILE}')

    # ─── Hilo principal ─────────────────────────────────────────────────

    def _main(self):
        """Hilo del test. Nunca bloquea los callbacks de ROS."""
        time.sleep(2.5)   # esperar conexión arduino_bridge

        if self.phase == 1:
            self._phase1()
        elif self.phase == 2:
            self._phase2()
        else:
            self.get_logger().error(
                f'phase={self.phase} no válido. Usa 1 ó 2.')

        self._stop()
        self.get_logger().info('\nNodo opto_ppr_confirm terminado.')
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = OptoPPRConfirm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop()
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
