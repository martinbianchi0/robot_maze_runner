# Figuras pendientes para el informe

## fig_arquitectura_general.png

- Seccion: Introduccion / Arquitectura Implementada.
- Mostrar: sensores, SLAM, mapa, MCL, A*, follower, detector de cono, FSM y kit
  de evidencia.
- Como obtener: diagrama manual o export desde draw.io/diagrams.net.
- Fuente: script/diagrama manual.
- Prioridad: alta.

## fig_mapa_laberinto.png

- Seccion: Parte A - SLAM y Mapa.
- Mostrar: `maps/laberinto_lab_20260702.pgm` con escala, origen si es posible y
  zonas relevantes del laberinto.
- Como obtener: exportar el PGM a PNG o usar `lab_make_report.py` si hay corrida.
- Fuente: mapa.
- Prioridad: alta.

## fig_flujo_navegacion.png

- Seccion: Parte B - Navegacion Autonoma.
- Mostrar: `/map`, costmap inflado, A*, `/plan`, follower y `/cmd_vel`.
- Como obtener: diagrama manual simple.
- Fuente: script/diagrama manual.
- Prioridad: media.

## fig_rviz_goal_nav.png

- Seccion: Parte B - Navegacion Autonoma.
- Mostrar: RViz con mapa, `/particlecloud`, `/amcl_pose`, `/goal_pose` y
  `/plan`.
- Como obtener: correr `./shs/navegar_tb4.sh --bag` o nav real y capturar RViz.
- Fuente: RViz o replay.
- Prioridad: alta.

## fig_obstaculo_nav_debug.png

- Seccion: Obstaculos No Mapeados.
- Mostrar: obstaculo tipo silla/patas, robot detenido o en recovery, ultimo path,
  estado `RECOVERY`/`IDLE`, `nav_debug.reason`.
- Como obtener: correr una prueba real con `lab_record_all.sh`, luego
  `lab_replay_rviz.sh` y capturar pantalla.
- Fuente: replay RViz.
- Prioridad: alta.

## fig_fsm_mision_cono.png

- Seccion: Parte C - Robot Real y Cono Rojo.
- Mostrar: estados de `mission_node.py`: LOCALIZE, SEARCH_CONE, PLAN_TO_CONE,
  NAVIGATE_TO_CONE, AVOID_OBSTACLE, REPLAN, VERIFY_CONE, DONE/FAILURE.
- Como obtener: diagrama manual desde `docs/decisiones/PARTE_C_MISION_FSM.md`.
- Fuente: script/diagrama manual.
- Prioridad: media.

## fig_detector_cono.png

- Seccion: Parte C - Robot Real y Cono Rojo.
- Mostrar: `/cone_debug_image` con bbox/centroide/bearing y `/cone_mask`.
- Como obtener: lanzar detector con `publish_debug: true`, capturar imagen desde
  RViz o `rqt_image_view`.
- Fuente: captura real o replay de bag de vision.
- Prioridad: alta.

## fig_rviz_replay_real.png

- Seccion: Evidencia Experimental y Reproducibilidad.
- Mostrar: mapa, trayectoria, ultimo path, goal, texto de `/nav_state`,
  `/mission_state`, deteccion de cono y si aparece `/nav_debug`.
- Como obtener:

```bash
./scripts/lab_replay_rviz.sh results/labo_demo/<timestamp> tb4_0
./scripts/lab_record_rviz.sh results/labo_demo/<timestamp>
```

- Fuente: replay RViz.
- Prioridad: alta.

