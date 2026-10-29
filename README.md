# capytown_G4 — RC-0 Primer Arranque

| Campo | Valor |
|---|---|
| Sección |S-003  |
| Variantes (calibración / validación) | A1 / A2 |
| ROS_DOMAIN_ID | 20 |
| r final [m] | 0.03114|
| b_eff final [m] |0.2296|
| W medido con regla [m] |0.14|

## Integrantes y roles
| Integrante | Rol |
|---|---|
| Daniela | Integración |
| Italo | Control |
| Hugo | Planificación |
| Cielo | Percepción |

## Cómo reproducir
1. En la Pi: `sh ~/ros2_humble.sh` (entra al contenedor `ros2_dev`).
2. `cd /root/yahboomcar_ws && colcon build --packages-select capytown_esan --symlink-install`
3. `export ROS_DOMAIN_ID=20 && source install/setup.bash`
4. `ros2 launch capytown_esan bringup.launch.py`

## Contenido
- `config/wheel_params.yaml` — r y b_eff calibrados
- `linea_recta.csv`, `calibration_log.csv`, `validacion.csv`, `setup_log.txt`, setup_rqt_graph.png.
- `bags/` — rc0_recta, rc0_giro, rc0_val_recta, rc0_val_gir
- `capytown_esan/plot_odom.py` y `plots/`
- `informe.pdf`

Documentación original del paquete: `docs_paquete.md`

 Los bags se grabaron con `capytown_esan/record_bag.py` (rosbag2_py) porque el robot no tiene internet para instalar `ros2 bag`.
 
  Nota: se reemplazaron el motor y una llanta por una falla mecánica. La calibración de b_eff se rehízo con el hardware nuevo (calibration_log.csv); el historial con el motor anterior está en calibration_log_hw_anterior.csv. r se verificó tras el cambio (c8, c9 en linea_recta.csv) y no requirió ajuste. Ademas, se tuvo que cambiar la bateria del robot, lo que afecta a toda la medicion.

Las llantas patinaban sobre el aro; se fijaron con una capa de masking tape en las 4 ruedas. Con eso se rehízo b_eff (calibration_log.csv) y se verificó r (c10, c11). El historial previo está en calibration_log_sin_cinta.csv.
