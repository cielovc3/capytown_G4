#!/bin/bash
# RC-0 mejora de Control: recta de 1.0 m en LAZO CERRADO (para por distancia de /odom
# y corrige el rumbo mientras avanza), grabando bag.
# Uso (con el bringup corriendo):  bash scripts/prueba_recta_cl.sh <nombre_bag> [parametros extra]
#   ej: bash scripts/prueba_recta_cl.sh rc0_recta_cl1
#       bash scripts/prueba_recta_cl.sh rc0_recta_cl2 -p kp:=3.0
# Ctrl+C durante el avance = parada de emergencia (frena y cierra el bag igual).
set -m
[ -z "$1" ] && { echo "Uso: bash scripts/prueba_recta_cl.sh <nombre_bag> [-p param:=valor ...]"; exit 1; }
REPO=/root/yahboomcar_ws/src/capytown_G4
BAG=$REPO/bags/$1
shift
[ -e "$BAG" ] && { echo "[ERROR] $BAG ya existe, usa otro nombre"; exit 1; }
mkdir -p "$REPO/bags"
REC_BIN=$(ros2 pkg prefix capytown_esan)/lib/capytown_esan/record_bag
NODO=$REPO/scripts/recta_cerrada.py

python3 "$NODO" --ros-args -p solo_reset:=true || exit 1
"$REC_BIN" "$BAG" > /tmp/record_bag.log 2>&1 &
REC=$!
sleep 3
python3 "$NODO" --ros-args "$@"
echo ">> Cerrando bag..."
sleep 1
kill -INT $REC; wait $REC

python3 - "$BAG" << 'PY'
import sys, math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
r = SequentialReader()
r.open(StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), ConverterOptions('', ''))
pts = []
while r.has_next():
    t, d, _ = r.read_next()
    if t == '/odom':
        p = deserialize_message(d, Odometry).pose.pose.position
        pts.append((p.x, p.y))
if not pts:
    print('[ERROR] el bag no tiene /odom. Revisa que el bringup este corriendo.')
    sys.exit(1)
dx, dy = pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]
print(f'\n  d_odom = {math.hypot(dx, dy) * 100:.1f} cm   (desvio lateral segun odom: {dy * 100:+.1f} cm)')
print(f'  bag: {sys.argv[1]}  ({len(pts)} mensajes /odom)')
PY
