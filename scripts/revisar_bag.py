#!/usr/bin/env python3
"""Diagnostico de un bag: compara /odom y /odom_raw y busca saltos.
Uso: python3 scripts/revisar_bag.py bags/rc0_recta_c3
Un paso normal a 0.10 m/s y ~11 Hz mide ~1 cm; un paso > 3 cm es un salto anormal."""
import math
import os
import sys

import yaml
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def storage_id(bag):
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        return yaml.safe_load(f)['rosbag2_bagfile_information']['storage_identifier']


bag = sys.argv[1]
r = SequentialReader()
r.open(StorageOptions(uri=bag, storage_id=storage_id(bag)), ConverterOptions('', ''))
data = {'/odom': [], '/odom_raw': []}
while r.has_next():
    topic, raw, stamp = r.read_next()
    if topic in data:
        p = deserialize_message(raw, Odometry).pose.pose.position
        data[topic].append((stamp * 1e-9, p.x, p.y))

print(f'== {bag}')
for topic, pts in data.items():
    if not pts:
        print(f'{topic}: sin mensajes')
        continue
    t0 = pts[0][0]
    path, jumps = 0.0, []
    for a, b in zip(pts, pts[1:]):
        s = math.hypot(b[1] - a[1], b[2] - a[2])
        path += s
        if s > 0.03:
            jumps.append((b[0] - t0, s))
    net = math.hypot(pts[-1][1] - pts[0][1], pts[-1][2] - pts[0][2])
    print(f'{topic}: {len(pts)} msgs | inicio ({pts[0][1]:.3f}, {pts[0][2]:.3f}) '
          f'fin ({pts[-1][1]:.3f}, {pts[-1][2]:.3f}) | neto {net * 100:.1f} cm | '
          f'recorrido {path * 100:.1f} cm | saltos > 3 cm: {len(jumps)}')
    for t, s in jumps[:5]:
        print(f'    salto de {s * 100:.1f} cm en t = {t:.1f} s')
