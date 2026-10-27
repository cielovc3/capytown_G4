#!/bin/bash
# RC-0 Planificacion: recta a 0.10 m/s durante 10 s (100 msgs a 10 Hz) y freno, grabando bag.
# Uso (con el bringup corriendo):  bash scripts/prueba_recta.sh <nombre_bag>
# Ctrl+C durante el avance = parada de emergencia (frena y cierra el bag igual).
# v2: el avance y el freno salen del MISMO publicador (ya conectado al robot),
#     asi el freno no se pierde como pasaba con dos `ros2 topic pub` separados.
set -m
[ -z "$1" ] && { echo "Uso: bash scripts/prueba_recta.sh <nombre_bag>"; exit 1; }
REPO=/root/yahboomcar_ws/src/capytown_G4
BAG=$REPO/bags/$1
[ -e "$BAG" ] && { echo "[ERROR] $BAG ya existe, usa otro nombre"; exit 1; }
mkdir -p "$REPO/bags"
REC_BIN=$(ros2 pkg prefix capytown_esan)/lib/capytown_esan/record_bag

MOVER=/tmp/prueba_recta_mover.py
cat > "$MOVER" << 'PY'
import sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

try:  # que Ctrl+C no apague ROS antes de mandar el freno
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
except (ImportError, TypeError):
    rclpy.init()
node = Node('prueba_recta')


def wait_sub(pub, timeout=10.0):
    t0 = time.time()
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > timeout:
            return False
    time.sleep(1.0)  # margen para que el descubrimiento termine
    return True


if sys.argv[1] == 'reset':
    pub = node.create_publisher(Empty, '/odom_reset', 10)
    if not wait_sub(pub):
        print('[ERROR] nadie escucha /odom_reset: el bringup no esta corriendo')
        sys.exit(1)
    pub.publish(Empty())
    time.sleep(0.5)
    print('>> Odometria en cero')
else:
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    if not wait_sub(pub):
        print('[ERROR] nadie escucha /cmd_vel')
        sys.exit(1)
    go = Twist()
    go.linear.x = 0.10
    try:
        print('>> Avanzando a 0.10 m/s durante 10 s... (Ctrl+C = parada de emergencia)')
        t0 = time.time()
        for i in range(100):
            pub.publish(go)
            dt = t0 + (i + 1) * 0.1 - time.time()
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        print('\n>> PARADA DE EMERGENCIA')
    finally:
        for _ in range(20):  # 2 s de freno
            pub.publish(Twist())
            time.sleep(0.1)
        print('>> Frenado')
node.destroy_node()
rclpy.try_shutdown()
PY

python3 "$MOVER" reset || exit 1
"$REC_BIN" "$BAG" > /tmp/record_bag.log 2>&1 &
REC=$!
sleep 3
python3 "$MOVER" move
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
