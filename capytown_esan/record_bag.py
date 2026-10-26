#!/usr/bin/env python3
"""Graba topics en un bag de ROS 2 (formato sqlite3) usando rosbag2_py.

Reemplaza a `ros2 bag record`: el contenedor del robot trae rosbag2_py pero no el
comando `ros2 bag`, y el robot no tiene internet para instalarlo.
El resultado es una carpeta de bag estandar (metadata.yaml + .db3).

Uso:
  ros2 run capytown_esan record_bag <carpeta_salida> [topic ...]
  (topics por defecto: /odom /odom_raw /cmd_vel /tf)   Ctrl+C para terminar.
"""
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.serialization import serialize_message
from rclpy.utilities import remove_ros_args
import rosbag2_py
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

TYPES = {
    '/odom': (Odometry, 'nav_msgs/msg/Odometry'),
    '/odom_raw': (Odometry, 'nav_msgs/msg/Odometry'),
    '/cmd_vel': (Twist, 'geometry_msgs/msg/Twist'),
    '/tf': (TFMessage, 'tf2_msgs/msg/TFMessage'),
}
DEFAULT_TOPICS = ['/odom', '/odom_raw', '/cmd_vel', '/tf']


class BagRecorder(Node):
    def __init__(self, uri, topics):
        super().__init__('record_bag')
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
                         rosbag2_py.ConverterOptions('', ''))
        # BEST_EFFORT acepta publicadores best-effort (micro-ROS) y reliable
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=100)
        self.counts = {}
        self.subs = []
        for topic in topics:
            msg_cls, type_str = TYPES[topic]
            self.writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=type_str, serialization_format='cdr'))
            self.counts[topic] = 0
            self.subs.append(self.create_subscription(msg_cls, topic, self._make_cb(topic), qos))
        self.timer = self.create_timer(2.0, self._status)
        self.get_logger().info(f'Grabando {topics} en {uri} ... (Ctrl+C para terminar)')

    def _make_cb(self, topic):
        def cb(msg):
            if self.writer is None:
                return
            self.writer.write(topic, serialize_message(msg),
                              self.get_clock().now().nanoseconds)
            self.counts[topic] += 1
        return cb

    def _status(self):
        print('  ' + ' | '.join(f'{t}: {n}' for t, n in self.counts.items()), flush=True)

    def close(self):
        w, self.writer = self.writer, None
        if w is not None:
            if hasattr(w, 'close'):
                w.close()
            del w  # al destruirse el writer se escribe metadata.yaml


def main():
    args = remove_ros_args(sys.argv)[1:]
    if not args:
        print(__doc__)
        return
    uri = os.path.abspath(args[0])
    topics = args[1:] or DEFAULT_TOPICS
    bad = [t for t in topics if t not in TYPES]
    if bad:
        print(f'[ERROR] topics no soportados: {bad}. Soportados: {list(TYPES)}')
        return
    if os.path.exists(uri):
        print(f'[ERROR] {uri} ya existe. Usa otro nombre o borra esa carpeta.')
        return
    os.makedirs(os.path.dirname(uri), exist_ok=True)

    rclpy.init()
    node = BagRecorder(uri, topics)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        counts = dict(node.counts)
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
        print(f'\nBag cerrado: {uri}')
        for t, n in counts.items():
            print(f'  {t}: {n} mensajes')


if __name__ == '__main__':
    main()
