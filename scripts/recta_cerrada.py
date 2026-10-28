#!/usr/bin/env python3
"""Recta en lazo cerrado (RC-0, mejora del rol de Control).

Avanza hasta que /odom marque `distancia` metros y, mientras avanza, corrige el
rumbo para ir recto aunque las ruedas de un lado giren distinto que las del otro:

    angular.z = -(kp * error_rumbo + ky * error_lateral)

El rumbo se mide con la odometria (fuente:=odom, por defecto) o integrando el
giroscopio de /imu (fuente:=imu), que no depende del derrape de las ruedas.
En los ultimos `zona_frenado` metros baja la velocidad para no pasarse.

Uso:
  python3 scripts/recta_cerrada.py --ros-args -p distancia:=1.0 -p kp:=2.0
  python3 scripts/recta_cerrada.py --ros-args -p solo_reset:=true   (solo pone /odom en cero)
Ctrl+C = parada de emergencia (frena igual).
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(x, lim):
    return max(-lim, min(lim, x))


class RectaCerrada(Node):
    def __init__(self):
        super().__init__('recta_cerrada')
        p = self.declare_parameter
        self.dist = float(p('distancia', 1.0).value)
        self.v = float(p('v', 0.10).value)
        self.v_min = float(p('v_min', 0.03).value)
        self.zona_frenado = float(p('zona_frenado', 0.12).value)
        self.margen = float(p('margen_frenado', 0.0).value)
        self.kp = float(p('kp', 2.0).value)
        self.ky = float(p('ky', 0.0).value)
        self.w_max = float(p('w_max', 0.4).value)
        self.fuente = str(p('fuente', 'odom').value)
        self.signo_imu = float(p('signo_imu', 1.0).value)
        self.max_err = math.radians(float(p('max_error_deg', 20.0).value))
        self.solo_reset = bool(p('solo_reset', False).value)

        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rst = self.create_publisher(Empty, '/odom_reset', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.pose = None          # (x, y, yaw) de /odom
        self.imu_yaw = 0.0        # rumbo integrado del giroscopio
        self.imu_t = None
        self.bias = 0.0
        self.bias_samples = None  # lista mientras se estima el sesgo
        self.imu_ok = False

    # ---------- callbacks ----------
    def on_odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_imu(self, m):
        self.imu_ok = True
        gz = m.angular_velocity.z
        now = time.monotonic()
        if self.bias_samples is not None:
            self.bias_samples.append(gz)
        if self.imu_t is not None:
            dt = now - self.imu_t
            if 0.0 < dt < 0.5:
                self.imu_yaw += self.signo_imu * (gz - self.bias) * dt
        self.imu_t = now

    # ---------- utilidades ----------
    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_sub(self, pub, name, timeout=10.0):
        t0 = time.monotonic()
        while pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t0 > timeout:
                print(f'[ERROR] nadie escucha {name}')
                return False
        self.spin_for(1.0)  # margen para que termine el descubrimiento
        return True

    def stop(self):
        for _ in range(20):  # 2 s de freno con el mismo publicador
            self.cmd.publish(Twist())
            time.sleep(0.1)

    # ---------- prueba ----------
    def do_reset(self):
        if not self.wait_sub(self.rst, '/odom_reset (el bringup no corre)'):
            return False
        self.rst.publish(Empty())
        self.spin_for(0.5)
        print('>> Odometria en cero')
        return True

    def run(self):
        t0 = time.monotonic()
        while self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t0 > 5.0:
                print('[ERROR] no llega /odom: revisa el bringup')
                return
        if not self.wait_sub(self.cmd, '/cmd_vel'):
            return
        if self.fuente == 'imu':
            if not self.imu_ok:
                print('[ERROR] no llega /imu; usa fuente:=odom')
                return
            print('>> Midiendo el sesgo del giroscopio (no muevas el robot)...')
            self.bias_samples = []
            self.spin_for(1.5)
            self.bias = sum(self.bias_samples) / max(1, len(self.bias_samples))
            self.bias_samples = None
            self.imu_yaw = 0.0
        self.spin_for(0.3)
        x0, y0, th0 = self.pose
        c, s = math.cos(th0), math.sin(th0)
        print(f'>> Avanzando {self.dist:.2f} m a {self.v:.2f} m/s con correccion de rumbo '
              f'(fuente={self.fuente}, kp={self.kp}, ky={self.ky}). Ctrl+C = parada de emergencia')
        t_max = time.monotonic() + 3.0 * self.dist / max(self.v_min, 1e-3) + 10.0
        e_max = 0.0
        motivo = 'llego a la distancia'
        try:
            next_t = time.monotonic()
            while True:
                next_t += 0.05
                while time.monotonic() < next_t:
                    rclpy.spin_once(self, timeout_sec=max(0.0, next_t - time.monotonic()))
                x, y, th = self.pose
                dx, dy = x - x0, y - y0
                avance = dx * c + dy * s
                lateral = -dx * s + dy * c
                e_th = self.imu_yaw if self.fuente == 'imu' else wrap(th - th0)
                e_max = max(e_max, abs(e_th))
                if abs(e_th) > self.max_err:
                    motivo = 'ABORTADO: el rumbo se desvio demasiado (signo de correccion?)'
                    break
                falta = self.dist - self.margen - avance
                if falta <= 0.0:
                    break
                if time.monotonic() > t_max:
                    motivo = 'TIMEOUT'
                    break
                v_cmd = self.v if falta > self.zona_frenado else \
                    max(self.v_min, self.v * falta / self.zona_frenado)
                tw = Twist()
                tw.linear.x = v_cmd
                tw.angular.z = clamp(-(self.kp * e_th + self.ky * lateral), self.w_max)
                self.cmd.publish(tw)
        except KeyboardInterrupt:
            motivo = 'PARADA DE EMERGENCIA'
        finally:
            self.stop()
        print(f'>> Frenado ({motivo})')
        self.spin_for(1.0)
        x, y, th = self.pose
        dx, dy = x - x0, y - y0
        print(f'   segun /odom: avance {100 * (dx * c + dy * s):.1f} cm | '
              f'lateral {100 * (-dx * s + dy * c):+.1f} cm | rumbo final '
              f'{math.degrees(wrap(th - th0)):+.1f} deg | error de rumbo max {math.degrees(e_max):.1f} deg')


def main():
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=sys.argv)
    node = RectaCerrada()
    try:
        if node.solo_reset:
            ok = node.do_reset()
        else:
            node.run()
            ok = True
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
