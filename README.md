# capytown_G4 — RC-0 Primer Arranque

| Campo | Valor |
|---|---|
| Sección |S-003  |
| Variantes (calibración / validación) | A1 / A2 |
| ROS_DOMAIN_ID | 20 |
| r final [m] | |
| b_eff final [m] | |
| W medido con regla [m] | |

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
