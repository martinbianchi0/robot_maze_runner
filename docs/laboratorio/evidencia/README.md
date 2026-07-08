# Evidencia de laboratorio

Contenido principal:

- `parte_c_servoing.md`: explicacion del problema LIDAR/camara con el cono bajo,
  solucion `SERVO_TO_CONE` y evidencia de corridas.
- `logs/parte_c6_mission.log`: corrida con fallback de servoing visual.
- `logs/parte_c7_mission.log`: corrida de busqueda con cono fuera del campo
  inicial y navegacion directa.
- `logs/parte_c10_mission.log`: corrida final de servoing.
- `logs/traj_amcl_pose_*.csv`: muestras de trayectoria/pose usadas para figuras.

Figuras asociadas:

- `../../informe/figures/parte_c_cono_debug.png`
- `../../informe/figures/parte_c_cono_mask.png`
- `../../informe/figures/parte_c_trayectoria_c6.png`
- `../../informe/figures/parte_c_trayectoria_c7.png`

Los rosbags pesados quedan fuera de git por `.gitignore`; si existen localmente,
son insumos de reproduccion, no parte del paquete liviano del repo.
