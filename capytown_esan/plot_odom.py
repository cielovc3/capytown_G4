#!/usr/bin/env python3
"""RC-0 · Percepcion: graficas x-y y theta-t de los bags de odometria.

Uso (desde la carpeta del repo, dentro del Docker):
  python3 capytown_esan/plot_odom.py                    # todos los bags oficiales que existan
  python3 capytown_esan/plot_odom.py bags/rc0_giro      # uno o varios bags concretos
  ros2 run capytown_esan plot_odom                      # igual, si esta registrado en setup.py

Por cada bag genera plots/<bag>.png con dos graficas:
  - izquierda: trayectoria x-y (cm) en el marco inicial del robot (el robot arranca
    en (0,0) mirando hacia +x)
  - derecha:  orientacion theta (deg) en funcion del tiempo (s)
Se dibuja /odom (corregido con r y b_eff) y, en linea punteada, /odom_raw (firmware).
Ademas imprime un resumen por bag (avance, desvio lateral y giro total segun /odom).
"""
import math
import os
import sys

import yaml
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

OFICIALES = ['rc0_recta', 'rc0_giro', 'rc0_val_recta', 'rc0_val_recta_2', 'rc0_val_giro']
TOPICS = ('/odom', '/odom_raw')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def leer_bag(path):
    """Devuelve {topic: [(t_s, x_m, y_m, yaw_rad), ...]} con /odom y /odom_raw."""
    with open(os.path.join(path, 'metadata.yaml')) as f:
        storage = yaml.safe_load(f)['rosbag2_bagfile_information']['storage_identifier']
    r = SequentialReader()
    r.open(StorageOptions(uri=path, storage_id=storage), ConverterOptions('', ''))
    datos = {t: [] for t in TOPICS}
    while r.has_next():
        topic, raw, stamp = r.read_next()
        if topic in datos:
            p = deserialize_message(raw, Odometry).pose.pose
            datos[topic].append((stamp * 1e-9, p.position.x, p.position.y, yaw_of(p.orientation)))
    return datos


def a_marco_inicial(serie):
    """Lleva la serie al marco inicial del robot: empieza en (0,0), mirando a +x, theta=0.
    Devuelve listas t (s), x (cm), y (cm), theta (deg, desenrollado)."""
    t0, x0, y0, th0 = serie[0]
    c, s = math.cos(th0), math.sin(th0)
    T, X, Y, TH = [], [], [], []
    acum, prev = 0.0, th0
    for t, x, y, th in serie:
        dx, dy = x - x0, y - y0
        acum += wrap(th - prev)
        prev = th
        T.append(t - t0)
        X.append(100 * (dx * c + dy * s))
        Y.append(100 * (-dx * s + dy * c))
        TH.append(math.degrees(acum))
    return T, X, Y, TH


def graficar(nombre, rel, salida):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, (ax_xy, ax_th) = plt.subplots(1, 2, figsize=(11, 4.5))
    estilos = {'/odom': dict(color='tab:blue', lw=2, label='/odom (corregido)'),
               '/odom_raw': dict(color='tab:gray', lw=1.2, ls='--', label='/odom_raw (firmware)')}
    for topic in ('/odom_raw', '/odom'):
        if topic not in rel:
            continue
        T, X, Y, TH = rel[topic]
        ax_xy.plot(X, Y, **estilos[topic])
        ax_th.plot(T, TH, **estilos[topic])
    if '/odom' in rel:
        T, X, Y, TH = rel['/odom']
        ax_xy.plot(X[0], Y[0], 'o', color='tab:green', label='inicio')
        ax_xy.plot(X[-1], Y[-1], 's', color='tab:red', label='fin')
    ax_xy.set_title('Trayectoria x-y (marco inicial del robot)')
    ax_xy.set_xlabel('x [cm]  (hacia adelante)')
    ax_xy.set_ylabel('y [cm]  (+ = izquierda)')
    ax_xy.set_aspect('equal', adjustable='datalim')
    if '/odom' in rel:  # en un giro el robot casi no se desplaza: fija una escala minima legible
        X, Y = rel['/odom'][1], rel['/odom'][2]
        if max(max(X) - min(X), max(Y) - min(Y)) < 10:
            cx, cy = (max(X) + min(X)) / 2, (max(Y) + min(Y)) / 2
            ax_xy.set_xlim(cx - 10, cx + 10)
            ax_xy.set_ylim(cy - 10, cy + 10)
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(fontsize=8, loc='best')
    ax_th.set_title('Orientacion theta vs tiempo')
    ax_th.set_xlabel('t [s]')
    ax_th.set_ylabel('theta [deg]  (+ = antihorario)')
    ax_th.grid(True, alpha=0.3)
    ax_th.legend(fontsize=8, loc='best')
    fig.suptitle(f'{nombre}  ·  capytown_G4 RC-0', fontsize=12)
    fig.tight_layout()
    fig.savefig(salida, dpi=120)
    plt.close(fig)


def exportar_csv(rel, salida):
    T, X, Y, TH = rel['/odom']
    with open(salida, 'w') as f:
        f.write('t_s,x_cm,y_cm,theta_deg\n')
        for fila in zip(T, X, Y, TH):
            f.write(','.join(f'{v:.4f}' for v in fila) + '\n')


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    repo = os.getcwd()
    if args:
        bags = [os.path.abspath(a.rstrip('/')) for a in args]
    else:
        bags = [os.path.join(repo, 'bags', b) for b in OFICIALES
                if os.path.isdir(os.path.join(repo, 'bags', b))]
        if not bags:
            print('[ERROR] No encontre bags oficiales. Ejecuta desde la carpeta del repo '
                  '(cd /root/yahboomcar_ws/src/capytown_G4) o pasa la ruta de un bag.')
            return 1
    out_dir = os.path.join(repo, 'plots')
    os.makedirs(out_dir, exist_ok=True)

    try:
        import matplotlib  # noqa: F401
        hay_mpl = True
    except ImportError:
        hay_mpl = False
        print('[AVISO] matplotlib no esta instalado: exporto CSV en plots/ en lugar de PNG.')

    print(f'{"bag":<18} {"dur[s]":>7} {"avance[cm]":>11} {"lateral[cm]":>12} {"giro[deg]":>10}  archivo')
    for bag in bags:
        nombre = os.path.basename(bag)
        if not os.path.isfile(os.path.join(bag, 'metadata.yaml')):
            print(f'{nombre:<18} [ERROR] no es una carpeta de bag (falta metadata.yaml)')
            continue
        datos = leer_bag(bag)
        rel = {t: a_marco_inicial(s) for t, s in datos.items() if len(s) >= 2}
        if '/odom' not in rel:
            print(f'{nombre:<18} [ERROR] el bag no tiene /odom')
            continue
        T, X, Y, TH = rel['/odom']
        if hay_mpl:
            salida = os.path.join(out_dir, f'{nombre}.png')
            graficar(nombre, rel, salida)
        else:
            salida = os.path.join(out_dir, f'{nombre}_odom.csv')
            exportar_csv(rel, salida)
        print(f'{nombre:<18} {T[-1]:7.1f} {X[-1]:11.1f} {Y[-1]:+12.1f} {TH[-1]:+10.1f}  '
              f'{os.path.relpath(salida, repo)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
