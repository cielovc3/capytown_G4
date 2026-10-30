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
