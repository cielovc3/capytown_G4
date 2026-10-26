# Predicciones RC-0 (escritas antes de medir)

**1. Recta 0.10 m/s × 10 s — ¿cuánto avanza según la cinta y según /odom?**
- Cinta: 100 cm  |  /odom: 100 cm
- Razonamiento: Pienso que el robot mantendrá una velocidad relativamente constante, aunque podrían existir pequeñas diferencias en las mediciones.

**2. Con b_eff = W, ¿el robot gira más o menos de 360°?**
- Predicción: Creo que el robot girará cerca de los 360°, pero posiblemente constante, aunque podrían existir pequeñas diferencias en las mediciones
- Razonamiento: Supongo que pueden existir errores en el movimiento de las ruedas que provoquen una pequeña diferencia en el ángulo final.

**3. ¿Qué pasa si dos grupos usan el mismo ROS_DOMAIN_ID?**
- Predicción: Creo que los robots de ambos grupos podrían detectarse entre sí e intercambiar información accidentalmente.
- Razonamiento: Supongo que, al compartir el mismo identificador de dominio, sus sistemas ROS 2 podrían comunicarse si están conectados a la misma red}
