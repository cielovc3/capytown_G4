#!/usr/bin/env python3
"""CapyTown cuadrado_controller — Experimentos 1, 2 y 3 SOLO con odometria.

Adaptacion de lane_controller.py (repo Robotica-Reto-2): el error ya no viene
de la camara (/lane_error) sino de /odom. La logica vive en core_cuadrado.py
(la misma que prueba 08_cuadrado_sim.py en la laptop).

    /odom (x, y, yaw)  ->  linea virtual + PID  ->  /cmd_vel (v, w)

Topicos
    suscribe : /odom           (nav_msgs/Odometry; cambiable con -p odom_topic:=/odom_raw)
    publica  : /cmd_vel        (geometry_msgs/Twist)
               /line_error     (std_msgs/Float32)  distancia a la linea virtual (m)
               /yaw_error      (std_msgs/Float32)  error de rumbo en los giros (rad)

Convenio de signo (igual que /lane_error):
    error > 0 -> robot a la IZQUIERDA de la linea -> w < 0 (girar a la derecha)

Uso (Pi5 / VM con ROS2 Humble). Un parametro por prueba:
    python3 cuadrado_controller.py --ros-args -p experimento:=recta -p controlador:=bangbang \
        -p line_offset:=0.10 -p log_csv:=exp1a.csv
    python3 cuadrado_controller.py --ros-args -p experimento:=recta -p kd:=0.0 -p line_offset:=0.10
    python3 cuadrado_controller.py --ros-args -p experimento:=giro90 -p kp_yaw:=10.0 -p kd_yaw:=0.0
    python3 cuadrado_controller.py --ros-args -p experimento:=cuadrado_abierto
    python3 cuadrado_controller.py --ros-args -p experimento:=cuadrado -p log_csv:=exp3b.csv

Los floats se pasan con punto decimal (kp:=3.0, no kp:=3).
Al terminar publica Twist() en cero e imprime el error de cierre.
"""
import csv
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from core_cuadrado import Params, Runner


def yaw_from_quat(q) -> float:
    """Yaw (rotacion en Z) de un geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CuadradoController(Node):
    def __init__(self):
        super().__init__("cuadrado_controller")
        d = Params()
        self.declare_parameters("", [
            ("experimento", d.experimento), ("controlador", d.controlador),
            ("linear_speed", d.v), ("side", d.side),
            ("kp", d.kp), ("ki", d.ki), ("kd", d.kd),
            ("kp_yaw", d.kp_yaw), ("ki_yaw", d.ki_yaw), ("kd_yaw", d.kd_yaw),
            ("integral_limit", d.i_limit), ("max_angular", d.max_w),
            ("w0", d.w0), ("line_offset", d.line_offset), ("bias_w", d.bias_w),
            ("turn_w", d.turn_w), ("turn_scale", d.turn_scale),
            ("odom_timeout", 0.5), ("control_rate", 30.0), ("log_csv", ""),
            ("odom_topic", "/odom"),
        ])
        gp = lambda n: self.get_parameter(n).value
        self.p = Params(
            experimento=gp("experimento"), controlador=gp("controlador"),
            v=float(gp("linear_speed")), side=float(gp("side")),
            kp=float(gp("kp")), ki=float(gp("ki")), kd=float(gp("kd")),
            kp_yaw=float(gp("kp_yaw")), ki_yaw=float(gp("ki_yaw")), kd_yaw=float(gp("kd_yaw")),
            i_limit=float(gp("integral_limit")), max_w=float(gp("max_angular")),
            w0=float(gp("w0")), line_offset=float(gp("line_offset")),
            bias_w=float(gp("bias_w")), turn_w=float(gp("turn_w")),
            turn_scale=float(gp("turn_scale")))
        self.timeout = float(gp("odom_timeout"))
        rate = float(gp("control_rate"))

        self.runner = Runner(self.p)
        self.pose = None                      # (x, y, yaw) del ultimo /odom
        self.last_rx = self.get_clock().now()
        self.last_stamp = self.get_clock().now()
        self.reported = False

        self.csv_file = None
        if gp("log_csv"):
            self.csv_file = open(gp("log_csv"), "w", newline="")
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(["t", "x", "y", "yaw", "v", "w", "e_line", "e_yaw", "estado"])

        self.sub = self.create_subscription(Odometry, gp("odom_topic"), self.on_odom, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_line = self.create_publisher(Float32, "/line_error", 10)
        self.pub_yaw = self.create_publisher(Float32, "/yaw_error", 10)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().info(
            f"cuadrado_controller listo: {self.p.experimento} ({self.p.controlador}) "
            f"kp={self.p.kp} ki={self.p.ki} kd={self.p.kd}  v={self.p.v} m/s. Esperando /odom...")

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self.last_rx = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = now
        if dt <= 0.0:
            return

        # Seguridad: sin odometria reciente => frenar (no repetir la ultima orden)
        age = (now - self.last_rx).nanoseconds * 1e-9
        if self.pose is None or age > self.timeout:
            self.pub.publish(Twist())
            return

        x, y, yaw = self.pose
        v, w, info = self.runner.update(x, y, yaw, dt)

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub.publish(cmd)
        self.pub_line.publish(Float32(data=float(info["e_line"])))
        self.pub_yaw.publish(Float32(data=float(info["e_yaw"])))

        if self.csv_file:
            self.csv.writerow([f"{self.runner.t:.3f}", f"{x:.4f}", f"{y:.4f}", f"{yaw:.4f}",
                               f"{v:.3f}", f"{w:.3f}", f"{info['e_line']:.4f}",
                               f"{info['e_yaw']:.4f}", info["state"]])

        if self.runner.done and not self.reported:
            self.reported = True
            self.get_logger().info(
                f"FIN. error de cierre = {info['closure'] * 100:.1f} cm, "
                f"yaw acumulado = {math.degrees(info['yaw_total']):.1f} deg, "
                f"tiempo = {self.runner.t:.1f} s")

    def close(self):
        if self.csv_file:
            self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = CuadradoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())       # frena al salir
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
