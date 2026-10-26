# capytown_G4 — RC-0 Primer Arranque

| Campo | Valor |
|---|---|
| Sección | |
| Variantes (calibración / validación) | |
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
- `linea_recta.csv`, `calibration_log.csv`, `validacion.csv`, `setup_log.txt`
- `bags/` — rc0_recta_cal, rc0_giro_cal, rc0_recta_val, rc0_giro_val
- `capytown_esan/plot_odom.py` y `plots/`
- `informe_rc0.pdf`

Documentación original del paquete: `docs_paquete.md`
