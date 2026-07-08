# Laboratorio

Esta carpeta resume la evidencia del TurtleBot4 real. El camino principal de
Parte C usa:

- mapa: `maps/laberinto_lab_20260702.yaml`
- perfil: `config/parte_c/real.yaml`
- wrapper: `./shs/parte_c_tb4.sh --ns tb4_0 --map maps/laberinto_lab_20260702.yaml`

## Evidencia incluida

- [evidencia/parte_c_servoing.md](evidencia/parte_c_servoing.md): validacion del
  fallback `SERVO_TO_CONE` y corridas de busqueda/navegacion al cono.
- [evidencia/logs/](evidencia/logs): logs de `mission_node` y CSV de trayectoria.
- [../informe/figures/](../informe/figures): capturas de camara, mascara HSV,
  trayectorias y figuras de navegacion.

No hay videos versionados en el repo. Los scripts para grabar/reproducir
evidencia existen en `scripts/lab_record_all.sh`, `scripts/lab_replay_rviz.sh` y
`scripts/lab_record_rviz.sh`.

Notas de laboratorio anteriores quedaron en `docs/archive/laboratorio-20260702/`.
