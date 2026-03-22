#!/usr/bin/env python3
"""
Test: 5m adelante, giro 180 (-z), 5m regreso, giro 180 (+z), parar.
Usa angulo ACUMULADO (delta incremental) para rotacion — inmune a wrapping ±180.
Resolucion encoder: ~3.3 deg/tick (45 PPR), tolerancia ancha para convergencia.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math, time

def log(msg):
    print(f'[{time.strftime("%H:%M:%S")}] {msg}', flush=True)

def normalize(a):
    """Normaliza angulo a [-pi, pi]"""
    while a > math.pi:  a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

class PositionTest(Node):
    def __init__(self):
        super().__init__('position_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.x = self.y = self.yaw = 0.0
        self.prev_yaw = 0.0
        self.odom_ok = False

        self.LIN_SPEED  = 0.3
        self.ANG_SPEED  = 0.35   # max rotation speed (rad/s)
        self.DIST_TOL   = 0.05
        self.ANG_TOL    = 0.15   # ~8.6 deg — wide for 3.3 deg encoder resolution

        self.state = 'WAIT'
        self.step  = 0
        self.start_x = self.start_y = 0.0
        self.target_dist = 0.0
        self.pause_end = 0.0
        self.log_mark = -1

        # Rotation tracking (accumulated angle)
        self.rot_target = 0.0     # target rotation amount (e.g. -pi)
        self.rot_accum  = 0.0     # accumulated rotation so far
        self.rot_settle = 0       # settling counter

        self.origin_x = self.origin_y = self.origin_yaw = 0.0
        self.results = []

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz control
        log('='*55)
        log(' TEST: 5m -> giro -180 -> 5m -> giro +180 -> stop')
        log('='*55)
        log('Esperando odometria...')

    def odom_cb(self, msg):
        new_yaw = math.atan2(
            2*(msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
               msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
            1 - 2*(msg.pose.pose.orientation.y**2 + msg.pose.pose.orientation.z**2))

        # Acumular delta de yaw cuando estamos rotando
        if self.odom_ok and self.state == 'ROT':
            delta = normalize(new_yaw - self.prev_yaw)
            self.rot_accum += delta

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = new_yaw
        self.prev_yaw = new_yaw

        if not self.odom_ok:
            self.odom_ok = True
            self.origin_x, self.origin_y, self.origin_yaw = self.x, self.y, self.yaw
            log(f'Odom OK: x={self.x:.3f} y={self.y:.3f} yaw={math.degrees(self.yaw):.1f}')

    def stop(self):
        t = Twist()
        for _ in range(5):
            self.cmd_pub.publish(t)

    def dist_from_start(self):
        return math.hypot(self.x - self.start_x, self.y - self.start_y)

    def loop(self):
        now = time.time()

        # --- PAUSE ---
        if self.state == 'PAUSE':
            self.stop()
            if now >= self.pause_end:
                self.state = 'NEXT'
            return

        # --- WAIT ODOM ---
        if self.state == 'WAIT':
            if self.odom_ok:
                log('Iniciando en 3s...')
                self.state = 'PAUSE'; self.pause_end = now + 3
            return

        # --- DONE ---
        if self.state == 'DONE':
            self.stop(); return

        # --- NEXT STEP ---
        if self.state == 'NEXT':
            self.step += 1; self.log_mark = -1

            if self.step == 1:
                log('\n>>> PASO 1: Avanzar 5m')
                self._start_forward(5.0)
            elif self.step == 2:
                log('\n>>> PASO 2: Girar 180 sentido horario (-z)')
                self._start_rotate(-math.pi)
            elif self.step == 3:
                log('\n>>> PASO 3: Avanzar 5m (regreso)')
                self._start_forward(5.0)
            elif self.step == 4:
                log('\n>>> PASO 4: Girar 180 sentido horario (-z) para quedar mirando al origen')
                self._start_rotate(-math.pi)
            elif self.step == 5:
                self._finish()
            return

        # --- FORWARD ---
        if self.state == 'FWD':
            d = self.dist_from_start()
            rem = self.target_dist - d
            if rem <= self.DIST_TOL:
                self.stop()
                r = f'Paso {self.step}: {d:.3f}m (obj {self.target_dist:.1f}m, err {abs(rem):.3f}m)'
                self.results.append(r); log(f'  OK: {r}')
                self._log_pos(); self.state='PAUSE'; self.pause_end=now+2
                return
            spd = self.LIN_SPEED
            if rem < 0.5: spd = max(0.1, self.LIN_SPEED * rem / 0.5)
            t = Twist(); t.linear.x = spd; self.cmd_pub.publish(t)
            m = int(d)
            if m > self.log_mark:
                self.log_mark = m
                log(f'  {d:.1f}m / {self.target_dist:.0f}m')
            return

        # --- ROTATE (accumulated angle approach) ---
        if self.state == 'ROT':
            remaining = self.rot_target - self.rot_accum  # rad left to rotate

            # Check if within tolerance
            if abs(remaining) <= self.ANG_TOL:
                self.rot_settle += 1
                self.stop()
                if self.rot_settle >= 6:  # ~0.3s settled at 20Hz
                    deg_done = math.degrees(self.rot_accum)
                    deg_target = math.degrees(self.rot_target)
                    err_deg = math.degrees(remaining)
                    r = f'Paso {self.step}: giro {deg_done:.1f} (obj {deg_target:.0f}, err {err_deg:.1f})'
                    self.results.append(r); log(f'  OK: {r}')
                    self._log_pos(); self.state='PAUSE'; self.pause_end=now+2
                return
            else:
                self.rot_settle = 0

            # Speed: proportional to remaining, with min/max
            abs_rem = abs(remaining)
            spd = self.ANG_SPEED * min(1.0, abs_rem / 0.6)  # decel below 34 deg
            spd = max(0.12, min(self.ANG_SPEED, spd))        # clamp [0.12, 0.35]
            spd = math.copysign(spd, remaining)               # direction

            t = Twist(); t.angular.z = spd; self.cmd_pub.publish(t)

            # Log progress every ~30 degrees (y debug cada 5°)
            deg_done = abs(math.degrees(self.rot_accum))
            deg_rem  = math.degrees(remaining)
            # DEBUG: mostrar rot_accum y yaw cada 5 grados
            mark5 = int(math.degrees(self.rot_accum) / 5) * 5
            if mark5 != getattr(self, '_dbg_mark5', None):
                self._dbg_mark5 = mark5
                log(f'  [DBG] accum={math.degrees(self.rot_accum):.1f}° '
                    f'yaw={math.degrees(self.yaw):.1f}° '
                    f'spd={spd:.2f} rem={deg_rem:.1f}°')
            m = int(deg_done / 30) * 30
            if m > 0 and m != self.log_mark:
                self.log_mark = m
                log(f'  girado {math.degrees(self.rot_accum):.1f} / {math.degrees(self.rot_target):.0f} deg'
                    f'  (resta={math.degrees(remaining):.1f}, spd={spd:.2f})')
            return

    def _start_forward(self, dist):
        self._log_pos()
        self.start_x, self.start_y = self.x, self.y
        self.target_dist = dist
        self.state = 'FWD'

    def _start_rotate(self, angle_rad):
        self._log_pos()
        self.rot_target = angle_rad      # target rotation (e.g., -pi)
        self.rot_accum  = 0.0            # reset accumulator
        self.rot_settle = 0
        self.prev_yaw   = self.yaw       # sync prev_yaw for clean deltas
        log(f'  yaw_actual={math.degrees(self.yaw):.1f}  rotacion_objetivo={math.degrees(angle_rad):.1f}')
        self.state = 'ROT'

    def _log_pos(self):
        log(f'  pos: x={self.x:.3f} y={self.y:.3f} yaw={math.degrees(self.yaw):.1f}')

    def _finish(self):
        self.stop()
        dx, dy = self.x - self.origin_x, self.y - self.origin_y
        d_err = math.hypot(dx, dy)
        a_err = abs(math.degrees(normalize(self.yaw - self.origin_yaw)))
        log('\n' + '='*55)
        log(' TEST COMPLETADO')
        self._log_pos()
        log('')
        for r in self.results: log(f'  {r}')
        log(f'\n  Error retorno: {d_err:.3f}m ({d_err/5*100:.1f}%)')
        log(f'  Error orientacion: {a_err:.1f} deg')
        log('='*55)
        self.state = 'DONE'

def main():
    rclpy.init()
    n = PositionTest()
    log('Ctrl+C para cancelar.')
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        log('Cancelado')
    finally:
        n.stop(); time.sleep(0.3); n.stop()
        n.destroy_node()
        try: rclpy.shutdown()
        except: pass
        log('Fin.')

if __name__ == '__main__':
    main()
