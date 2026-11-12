#!/usr/bin/env python3
"""CapyTown scripts_pdi — nucleo del experimento "linea virtual y cuadrado de 2 x 2 m".

Adaptado de lane_controller.py (repo Robotica-Reto-2): en lugar de recibir
/lane_error de la camara, el error se calcula con la ODOMETRIA (/odom).
No importa rclpy: se prueba en la laptop (08_cuadrado_sim.py) y el nodo
cuadrado_controller.py usa exactamente esta misma clase.

Experimentos (parametro `experimento`):
    recta             1 lado de `side` m siguiendo una LINEA VIRTUAL
    giro90            un giro de +90 deg (respuesta al escalon)
    cuadrado          4 lados + 4 giros, todo con PID
    cuadrado_abierto  4 lados + 4 giros por tiempo, sin realimentacion

Linea virtual: pasa por el punto donde empieza el lado y apunta hacia yaw_obj.
    d > 0  -> el robot esta a la IZQUIERDA de la linea
    e = d + line_offset            (line_offset > 0: arranca a la izquierda)
    w = -(P + I + D)               (e > 0 -> girar a la derecha, w < 0)
Es el mismo convenio de signo de /lane_error del paquete con camara.
"""
import math
from dataclasses import dataclass

import numpy as np


# --------------------------------------------------------------------------
# Robot MicroROS-Pi5: chasis de 4 ruedas con 4 motores (tipo oruga / skid-steer).
# Datos del plano del fabricante (mm -> m).
# --------------------------------------------------------------------------
ROBOT = dict(
    largo=0.23796, ancho=0.153, alto=0.12418,   # cuerpo
    d_rueda=0.048, a_rueda=0.0215,              # llanta de goma 48 x 21.5 mm
    rpm_max=450.0,                              # motor 310 con encoder (450 +- 10 rpm, sin carga)
)
ROBOT["pista"] = ROBOT["ancho"] - ROBOT["a_rueda"]          # centro a centro de rueda (estimada)
ROBOT["v_max"] = ROBOT["rpm_max"] / 60.0 * math.pi * ROBOT["d_rueda"]   # m/s sin carga
ROBOT["radio_giro"] = math.hypot(ROBOT["largo"], ROBOT["ancho"]) / 2.0  # radio que barre al girar sobre su eje


def ruedas_rpm(v, w, chi=1.0):
    """rpm de las ruedas izquierda y derecha para una orden (v, w).

    Con 4 ruedas, girar obliga a las llantas a derrapar de lado: la pista
    efectiva es mayor que la geometrica, b_ef = chi * pista (chi >= 1; se
    calibra, ver 10 en README). El mismo lado izquierdo (o derecho) gira junto.
    """
    b_ef = chi * ROBOT["pista"]
    c = math.pi * ROBOT["d_rueda"]
    v_izq = v - w * b_ef / 2.0
    v_der = v + w * b_ef / 2.0
    return v_izq / c * 60.0, v_der / c * 60.0


def normalize(a: float) -> float:
    """Normaliza un angulo a [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class PID:
    """PID discreto con anti-windup (la misma ecuacion de la clase)."""

    def __init__(self, kp, ki, kd, i_limit=0.5):
        self.kp, self.ki, self.kd, self.i_limit = kp, ki, kd, i_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.e_prev = None

    def step(self, e, dt):
        self.integral = clamp(self.integral + e * dt, -self.i_limit, self.i_limit)
        d = 0.0 if self.e_prev is None else (e - self.e_prev) / dt
        self.e_prev = e
        return self.kp * e + self.ki * self.integral + self.kd * d


@dataclass
class Params:
    experimento: str = "cuadrado"   # recta | giro90 | cuadrado | cuadrado_abierto
    controlador: str = "pid"        # pid | bangbang  (solo afecta las rectas)
    v: float = 0.20                 # m/s
    side: float = 2.0               # m, largo de cada lado
    # PID de la linea (error en metros)
    kp: float = 3.0
    ki: float = 0.0
    kd: float = 4.0
    # PID del giro (error en radianes)
    kp_yaw: float = 3.0
    ki_yaw: float = 0.0
    kd_yaw: float = 0.4
    i_limit: float = 0.5
    max_w: float = 2.0              # rad/s
    w0: float = 0.25                # rad/s, amplitud del bang-bang
    deadband: float = 0.005         # m, zona muerta del bang-bang
    line_offset: float = 0.0        # m, arranca a la izquierda de la linea
    bias_w: float = 0.0             # rad/s, sesgo de motores inyectado
    yaw_tol: float = math.radians(2.0)
    settle_time: float = 0.3        # s dentro de la tolerancia para dar el giro por hecho
    turn_w: float = 0.8             # rad/s, giro del lazo abierto
    turn_scale: float = 1.0         # calibracion del giro (1/chi): alarga el tiempo de giro en lazo abierto
    n_sides: int = 4


class Runner:
    """Maquina de estados ESPERA -> AVANZA <-> GIRA -> FIN."""

    def __init__(self, p: Params):
        self.p = p
        self.pid_line = PID(p.kp, p.ki, p.kd, p.i_limit)
        self.pid_yaw = PID(p.kp_yaw, p.ki_yaw, p.kd_yaw, p.i_limit)
        self.state = "ESPERA"
        self.done = False
        self.closure_error = None
        self.yaw_total = 0.0
        self.t = 0.0

    def start(self, x, y, yaw):
        """Fija el origen: el primer /odom define el inicio y el rumbo 0."""
        self.x0, self.y0, self.yaw0 = x, y, yaw
        self.sx, self.sy = x, y            # inicio del lado actual
        self.k = 0                         # giros terminados
        self.yaw_obj = yaw                 # rumbo del lado actual
        self.yaw_target = yaw
        self.yaw_prev = yaw
        self.t_state = 0.0
        self.t_in_tol = 0.0
        self.pid_line.reset()
        self.pid_yaw.reset()
        if self.p.experimento == "giro90":
            self.yaw_target = yaw + math.pi / 2
            self.state = "GIRA"
        else:
            self.state = "AVANZA"

    def update(self, x, y, yaw, dt):
        """Una muestra del lazo. Devuelve (v, w, info) para /cmd_vel."""
        p = self.p
        if self.state == "ESPERA":
            self.start(x, y, yaw)
        self.t += dt
        self.t_state += dt
        self.yaw_total += normalize(yaw - self.yaw_prev)
        self.yaw_prev = yaw

        v = w = 0.0
        e_line = e_yaw = 0.0

        if self.state == "AVANZA":
            psi = self.yaw_obj
            dx, dy = x - self.sx, y - self.sy
            s = dx * math.cos(psi) + dy * math.sin(psi)      # avance sobre la linea
            d = -dx * math.sin(psi) + dy * math.cos(psi)     # + = a la izquierda
            e_line = d + p.line_offset
            v = p.v
            if p.experimento == "cuadrado_abierto":
                finished = self.t_state >= p.side / p.v      # solo por tiempo
            else:
                if p.controlador == "bangbang":
                    w = 0.0 if abs(e_line) < p.deadband else -math.copysign(p.w0, e_line)
                else:
                    w = -self.pid_line.step(e_line, dt)
                finished = s >= p.side
            if finished:
                self._side_done()

        elif self.state == "GIRA":
            e_yaw = normalize(self.yaw_target - yaw)
            if p.experimento == "cuadrado_abierto":
                w = p.turn_w
                if self.t_state >= (math.pi / 2) / (p.turn_w * p.turn_scale):
                    self._turn_done(x, y)
            else:
                w = self.pid_yaw.step(e_yaw, dt)
                self.t_in_tol = self.t_in_tol + dt if abs(e_yaw) < p.yaw_tol else 0.0
                if self.t_in_tol >= p.settle_time:
                    self._turn_done(x, y)

        if self.state == "FIN":
            if not self.done:
                self.done = True
                self.closure_error = math.hypot(x - self.x0, y - self.y0)
            v = w = 0.0
        else:
            w = clamp(w, -p.max_w, p.max_w) + p.bias_w

        info = dict(state=self.state, lado=self.k, e_line=e_line, e_yaw=e_yaw,
                    yaw_total=self.yaw_total, closure=self.closure_error)
        return v, w, info

    def _side_done(self):
        """Termino un lado recto."""
        if self.p.experimento == "recta":
            self.state = "FIN"
            return
        self.yaw_target = self.yaw_obj + math.pi / 2
        self.state = "GIRA"
        self.t_state = 0.0
        self.t_in_tol = 0.0
        self.pid_yaw.reset()

    def _turn_done(self, x, y):
        """Termino un giro de 90 deg: el siguiente lado arranca desde aqui."""
        self.k += 1
        self.yaw_obj = self.yaw_target
        if self.p.experimento == "giro90" or self.k >= self.p.n_sides:
            self.state = "FIN"
            return
        self.sx, self.sy = x, y
        self.state = "AVANZA"
        self.t_state = 0.0
        self.pid_line.reset()


# --------------------------------------------------------------------------
# Planta simulada: uniciclo con retardo de actuador y deslizamiento.
# --------------------------------------------------------------------------
def simular(p: Params, t_max=150.0, dt=1.0 / 30.0, tau=0.15,
            v_scale=1.0, w_scale=1.0, ruido_yaw=0.0, retardo=3, semilla=7):
    """Simula el robot. v_scale / w_scale < 1 imitan patinaje de ruedas.

    El controlador solo ve la pose (x, y, yaw), como con /odom, con `retardo`
    muestras de latencia (3 muestras = 0.1 s a 30 Hz).
    """
    rng = np.random.default_rng(semilla)
    run = Runner(p)
    x = y = th = 0.0
    v_act = w_act = 0.0
    buf = [(0.0, 0.0, 0.0)] * max(1, retardo)
    log = {k: [] for k in ("t", "x", "y", "yaw", "e_line", "e_yaw", "estado", "v", "w")}
    t = 0.0
    while t < t_max:
        buf.append((x, y, th))
        ox, oy, oth = buf.pop(0)          # pose retrasada, como llega /odom
        v_cmd, w_cmd, info = run.update(ox, oy, oth + rng.normal(0.0, ruido_yaw), dt)
        log["t"].append(t); log["x"].append(x); log["y"].append(y)
        log["yaw"].append(th); log["e_line"].append(info["e_line"])
        log["e_yaw"].append(info["e_yaw"]); log["estado"].append(info["state"])
        log["v"].append(v_cmd); log["w"].append(w_cmd)
        if run.done:
            break
        v_act += (v_cmd * v_scale - v_act) * dt / tau
        w_act += (w_cmd * w_scale - w_act) * dt / tau
        x += v_act * math.cos(th) * dt
        y += v_act * math.sin(th) * dt
        th += w_act * dt
        t += dt
    out = {k: np.array(v) for k, v in log.items()}
    out["closure"] = run.closure_error
    out["yaw_total_deg"] = math.degrees(run.yaw_total)
    out["terminado"] = run.done
    return out


class Planta:
    """La misma planta de `simular`, pero avanzando UNA muestra a la vez (para animar).

    Uso por muestra:
        ox, oy, oth = planta.medir()                 # pose con la latencia de /odom
        v, w, info = runner.update(ox, oy, oth, dt)
        planta.paso(v, w, dt)                         # el robot se mueve
    """

    def __init__(self, tau=0.15, v_scale=1.0, w_scale=1.0, retardo=3, ruido_yaw=0.0, semilla=7):
        self.tau, self.v_scale, self.w_scale = tau, v_scale, w_scale
        self.ruido_yaw = ruido_yaw
        self.retardo = max(1, retardo)
        self.rng = np.random.default_rng(semilla)
        self.reset()

    def reset(self):
        self.x = self.y = self.th = 0.0
        self.v_act = self.w_act = 0.0
        self.buf = [(0.0, 0.0, 0.0)] * self.retardo

    def medir(self):
        self.buf.append((self.x, self.y, self.th))
        ox, oy, oth = self.buf.pop(0)
        return ox, oy, oth + self.rng.normal(0.0, self.ruido_yaw)

    def paso(self, v_cmd, w_cmd, dt):
        self.v_act += (v_cmd * self.v_scale - self.v_act) * dt / self.tau
        self.w_act += (w_cmd * self.w_scale - self.w_act) * dt / self.tau
        self.x += self.v_act * math.cos(self.th) * dt
        self.y += self.v_act * math.sin(self.th) * dt
        self.th += self.w_act * dt

    def empujar(self, lateral, dtheta):
        """Perturbacion externa: desplaza el robot de lado y lo desvia (para ver como se recupera)."""
        self.x += -math.sin(self.th) * lateral
        self.y += math.cos(self.th) * lateral
        self.th += dtheta


def metricas(sim, p: Params):
    """Metricas del lab segun el experimento."""
    m = {}
    if p.experimento == "recta":
        e = np.abs(sim["e_line"])
        m["|e| medio (cm)"] = e.mean() * 100
        m["|e| maximo (cm)"] = e.max() * 100
        dentro = e <= 0.02
        t_as = next((sim["t"][i] for i in range(len(e)) if dentro[i:].all()), None)
        m["t asentamiento +-2 cm (s)"] = t_as
        m["error final (cm)"] = abs(sim["e_line"][-1]) * 100
    elif p.experimento == "giro90":
        yaw = np.degrees(sim["yaw"])
        m["sobreimpulso (%)"] = max(0.0, yaw.max() - 90.0) / 90.0 * 100
        dentro = np.abs(yaw - 90.0) <= 2.0
        t_as = next((sim["t"][i] for i in range(len(yaw)) if dentro[i:].all()), None)
        m["t asentamiento +-2 deg (s)"] = t_as
        m["error final (deg)"] = abs(90.0 - yaw[-1])
    else:
        rectas = np.array([s == "AVANZA" for s in sim["estado"]])
        m["error de cierre (cm)"] = None if sim["closure"] is None else sim["closure"] * 100
        m["yaw acumulado (deg)"] = sim["yaw_total_deg"]
        m["|e| medio en rectas (cm)"] = np.abs(sim["e_line"][rectas]).mean() * 100 if rectas.any() else 0.0
        m["tiempo total (s)"] = sim["t"][-1]
    return m
