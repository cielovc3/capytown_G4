#!/usr/bin/env python3
"""Diagnostico del giro: mide cuanto giro el robot segun 3 fuentes a la vez.

  - giroscopio (/imu): mide el giro REAL del cuerpo del robot (no depende de las ruedas)
  - /odom_raw: lo que cuentan los encoders de las ruedas (firmware)
  - /odom: lo anterior corregido con r y b_eff (lo que usa calibrate_beff)

Uso (en una terminal aparte, con el robot QUIETO):
  python3 scripts/medir_giro.py
  1) espera el mensaje 'LISTO'  (mide 2 s el sesgo del giroscopio)
  2) en la otra terminal lanza el giro (probar ... + Enter)
  3) cuando el robot se detenga, vuelve aqui y presiona Ctrl+C
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class MedirGiro(Node):
    def __init__(self):
        super().__init__('medir_giro')
        self.bias_samples = []
        self.bias = None
        self.gyro = 0.0
        self.t_imu = None
        self.last = {'/odom_raw': None, '/odom': None}
        self.acc = {'/odom_raw': 0.0, '/odom': 0.0}
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        for topic in self.acc:
            self.create_subscription(Odometry, topic, self.make_cb(topic), qos_profile_sensor_data)
        self.t0 = time.monotonic()
        self.create_timer(1.0, self.report)

    def on_imu(self, m):
        gz = m.angular_velocity.z
        now = time.monotonic()
        if self.bias is None:
            self.bias_samples.append(gz)
            if now - self.t0 >= 2.0 and len(self.bias_samples) > 10:
                self.bias = sum(self.bias_samples) / len(self.bias_samples)
                print(f'>> LISTO (sesgo del giroscopio = {math.degrees(self.bias):+.3f} deg/s). '
                      f'Lanza el giro y al terminar presiona Ctrl+C aqui.', flush=True)
                self.t_imu = now
                for k in self.last:
                    self.last[k] = None
                    self.acc[k] = 0.0
            return
        dt = now - self.t_imu
        if 0.0 < dt < 0.5:
            self.gyro += (gz - self.bias) * dt
        self.t_imu = now

    def make_cb(self, topic):
        def cb(m):
            if self.bias is None:
                return
            y = yaw_of(m.pose.pose.orientation)
            if self.last[topic] is not None:
                self.acc[topic] += wrap(y - self.last[topic])
            self.last[topic] = y
        return cb

    def report(self):
        if self.bias is not None:
            print(f'   giroscopio {math.degrees(self.gyro):7.1f} | odom_raw '
                  f'{math.degrees(self.acc["/odom_raw"]):7.1f} | odom {math.degrees(self.acc["/odom"]):7.1f} deg',
                  flush=True)


def main():
    rclpy.init()
    node = MedirGiro()
    print('>> Midiendo el sesgo del giroscopio 2 s: NO muevas el robot...', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    g = abs(math.degrees(node.gyro))
    raw = abs(math.degrees(node.acc['/odom_raw']))
    od = abs(math.degrees(node.acc['/odom']))
    print('\n================ RESULTADO DEL GIRO ================')
    print(f'  giroscopio (giro real del cuerpo): {g:7.1f} deg')
    print(f'  odom_raw   (encoders)            : {raw:7.1f} deg')
    print(f'  odom       (lo que usa el script): {od:7.1f} deg')
    if raw > 1:
        print(f'  giroscopio / encoders = {g / raw:.3f}   (si cambia entre corridas -> derrape variable)')
    print('====================================================')
    try:
        node.destroy_node()
        rclpy.try_shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
