# capytown_G4 — RC-0 Primer arranque y calibración

Robótica (08079) · ESAN · 2026-2

| Campo | Valor |
|---|---|
| Sección | S-003 |
| Variantes (calibración / validación) | A1 / A2 |
| Robot | MicroROS-Pi5 (roscar6) |
| ROS_DOMAIN_ID | 20 (fijo, por indicación del docente) |
| r final [m] | 0.03114 |
| b_eff final [m] | 0.2296 |
| W medido con regla [m] | 0.14 |

## Integrantes y roles
| Integrante | Rol |
|---|---|
| Daniela | Integración |
| Italo | Control |
| Hugo | Planificación |
| Cielo | Percepción |

## Resultados
| Prueba | Resultado |
|---|---|
| Recta (calibración A1) | error final ≤ 2 % con r = 0.03114 (`linea_recta.csv`) |
| Giro (calibración A1) | convergió: 357°, error 0.83 % con b_eff = 0.2296 (`calibration_log.csv`) |
| Validación A2, rectas | error 3.41 % y 3.09 %, desvío lateral ≤ 0.1 cm (`validacion.csv`) |
| Validación A2, giro | 363°, error 0.83 % (`validacion.csv`) |

## Cómo reproducir
1. En la Pi: `sh ~/ros2_humble.sh` (entra al contenedor `ros2_dev`).
2. `cd /root/yahboomcar_ws && colcon build --packages-select capytown_esan --symlink-install && source install/setup.bash`
3. `ros2 launch capytown_esan bringup.launch.py`
4. Recta: `bash scripts/prueba_recta.sh <nombre_bag>` · Giro: `ros2 run capytown_esan calibrate_beff --ros-args -p b_eff:=0.2296`
5. Gráficas: desde la carpeta del repo, `ros2 run capytown_esan plot_odom` (genera `plots/<bag>.png`).

## Entregables
- `config/wheel_params.yaml` — r y b_eff calibrados
- `setup_log.txt`, `setup_rqt_graph.png` — arranque (Parte A)
- `linea_recta.csv`, `calibration_log.csv`, `validacion.csv`
- `bags/rc0_recta`, `bags/rc0_giro`, `bags/rc0_val_recta`, `bags/rc0_val_recta_2`, `bags/rc0_val_giro`
- `capytown_esan/plot_odom.py` y `plots/`
- `predicciones.md`, `informe.pdf`

## Historial y archivos de apoyo
- `calibration_log_hw_anterior.csv`, `calibration_log_bruto.csv`, `calibration_pruebas.csv`: calibración del giro con el motor dañado (antes del cambio de motor y llanta).
- `calibration_log_bruto_hw_nuevo.csv`: todas las corridas con el motor nuevo, incluidas las de batería baja y cartulina suelta.
- `calibration_log_sin_cinta.csv`: calibración con el motor nuevo antes de fijar las llantas con masking tape.
- `bags/rc0_recta_c1` … `c11`: corridas de la recta (c8–c11 verifican r tras los cambios de hardware). `bags/rc0_giro_i*`: iteraciones del giro con el hardware anterior.
- `recta_lazo_cerrado.csv`, `bags/rc0_recta_cl1`, `scripts/recta_cerrada.py`: mejora propuesta (recta con corrección de rumbo), fuera del método oficial.
- `scripts/`: utilidades de prueba y diagnóstico (`prueba_recta.sh`, `revisar_bag.py`, `medir_giro.py`).

## Notas
- Los bags se grabaron con `capytown_esan/record_bag.py` (rosbag2_py) porque el robot no tiene internet para instalar `ros2 bag`.
- Se reemplazaron el motor y una llanta por una falla mecánica, y luego la batería. Tras cada cambio se rehízo b_eff y se verificó r.
- Las llantas patinaban sobre el aro; se fijaron con una capa de masking tape en las 4 ruedas. La calibración final de b_eff se hizo con las llantas fijas.
- El giroscopio del robot publica velocidad angular en cero, por lo que no se pudo usar para verificar los giros.


---

# capytown_G4 — RC-1 Control en lazo cerrado (Simulador y Robot Real)

## Parametros de ruido en simulacion
* **Niveles de ruido evaluados:** `low` (default) y `medium` (estres).
* **Nivel usado para sintonizacion principal:** `low`.

## Parametros finales (`config/rc1_pid.yaml`)
| Parametro | Valor Simulador | Valor Robot Real (`roscar6`) |
|---|---|---|
| `wheel_radius` [m] | 0.03114 | 0.0322 |
| `wheel_separation` [m] | 0.2296 | 0.2307 |
| `kp` / `ki` / `kd` (Recta 1D) | 3.0 / 0.50 / 4.0 | 3.0 / 0.05 / 2.5 |
| `kp` / `ki` / `kd` (Cuadrado 3B) | 3.0 / 0.00 / 4.0 | 1.8 / 0.00 / 1.5 |
| `kp_yaw` / `kd_yaw` | 10.0 / 0.8 | 4.5 / 0.8 |
| `linear_speed` / `max_angular` | 0.20 m/s / 1.00 rad/s | 0.10 m/s / 0.75 rad/s |

## Resultados Parte E (Robot Real `roscar6`)
| Experimento | Criterio Rubrica | Resultado Odom | Resultado Fisico Real |
|---|---|---|---|
| **E5 — Giro 90° (`run2`)** | Error final <= 3° | Error: 1.19° (t_s = 2.77 s) | **89.0°** (error 1.0°) |
| **E6 — Recta 1D (`run3`)** | Error final <= 5 cm | \|e\| medio 5.15 cm | **0.0 cm** de desvio lateral (avance 200 cm) |
| **E7 — Cuadrado 3A (Lazo abierto)** | Referencia base | Cierre: 60.20 cm (355.5°) | Cierre real: **65.0 cm** |
| **E7 — Cuadrado 3B (`run1-3`)** | Promedio <= 25 cm y < 3A | Promedio Odom: 10.31 cm | `run1`: 30.0 cm, `run2`: 19.0 cm, `run3`: 20.0 cm (**Promedio: 23.0 cm**) |

## Bitacora de re-sintonia (Simulador -> Robot Real)
1. **Recta 1D (Reduccion de `ki` y ajuste de `wheel_radius`):** Con las ganancias del simulador (`kp=3.0, ki=0.5, kd=4.0`), la friccion estatica inicial del piso genero *integral windup* y un desvio fisico de 13.5 cm (`run1`). Se redujo `ki` a `0.05` y `kd` a `2.5` (`run2`: 12.0 cm, avance 204 cm) y se ajusto `wheel_radius` de `0.0328` a `0.0322`, logrando **0.0 cm** de error lateral final en 200 cm (`run3`).
2. **Cuadrado 3B (Suavizado de `kp_yaw`, `kp` y `max_angular`):** En el chasis *skid-steer* de 4 ruedas fijas, las ganancias altas del simulador (`kp_yaw=10.0, kp=3.0`) provocaban patinaje lateral al corregir rumbo y girar en las esquinas. Se redujeron a `kp_yaw=4.5, kd_yaw=0.8` y `kp=1.8, kd=1.5` con `linear_speed=0.10 m/s` y `max_angular=0.75 rad/s`.
3. **Compensacion de patinaje en giro (`wheel_separation`):** A velocidad angular moderada (`0.75 rad/s`), se ajusto `wheel_separation` en `/odom_corrector` a `0.2307 m`, reduciendo el error de cierre fisico de 65.0 cm (lazo abierto 3A) a **30.0 cm (`run1`), 19.0 cm (`run2`) y 20.0 cm (`run3`)**, alcanzando un promedio de **23.0 cm** (<= 25 cm).

## Tabla de transferencia (Criterio E8)
**Prueba:** Recta 1D con las mismas ganancias del simulador (`kp=3.0, ki=0.5, kd=4.0`).
| Metrica | Simulador (`exp1d_sim`) | Robot Real (`recta_1d_real_run1`) | Analisis |
|---|---|---|---|
| Cierre real (desvio) | **0.54 cm** | **13.5 cm** | La friccion estatica del robot real genero *integral windup*, causando sobrecorreccion. |
| Accion tomada | n/a | Reduccion de `ki` a 0.05 y `kd` a 2.5 | Al eliminar el windup (`run3`), el desvio fisico bajo a 0.0 cm. |

## Como reproducir RC-1
1. Levantar driver y corrector de odometria:
   `ros2 launch capytown_esan bringup.launch.py`
2. Cargar parametros calibrados en `/odom_corrector`:
   `ros2 param set /odom_corrector wheel_radius 0.0322 && ros2 param set /odom_corrector wheel_separation 0.2307`
3. Ejecutar controlador PID (`scripts/rc1/cuadrado_controller.py`):
   `python3 scripts/rc1/cuadrado_controller.py --ros-args --params-file config/rc1_pid.yaml -p experimento:=cuadrado`
4. Generar graficas PNG de 3B:
   `python3 scripts/rc1/09_plot_csv.py csv/cuadrado_3b_real_run3.csv --experimento cuadrado --lado 2.0`

## Entregables RC-1 (Partes A a F)
- `csv/hoja_resultados.csv` — Tabla maestra con pruebas de simulacion y robot real.
- `config/rc1_pid.yaml` — Parametros odometricos y ganancias PID (simulador y robot).
- `csv/` y `plots/` — Series `.csv` y graficas `.png` para todos los experimentos.
- `bags/` — Grabaciones `ros2 bag` de las corridas reales (`rc1_real_*`).
- `predicciones_c.md` y `docs_paquete.md` — Predicciones teóricas y documentación.
- `informe.pdf` — Informe consolidado de resultados (1 pagina).

## Notas
* Se conserva la evidencia de la primera re-sintonia de giro (`giro90_real_kd04_run1.csv` y bag asociado) justificando el paso a `kd_yaw=0.8`.
