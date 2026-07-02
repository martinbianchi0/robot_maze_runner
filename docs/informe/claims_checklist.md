# Claims checklist para el informe

Usar este archivo antes de pasar el borrador a Overleaf. Si un claim no tiene
evidencia, debe quedar como TODO o formularse como limitacion.

## Claims seguros

- Se implemento una arquitectura modular con paquetes separados para SLAM,
  navegacion, percepcion y mision.
- La opcion seguida para Parte A es mapa grillado de ocupacion, consistente con
  Resultado V1 de la consigna.
- El paquete `maze_slam` implementa Grid-Based FastSLAM con particulas y mapas de
  ocupacion en log-odds.
- Existe un mapa real del laberinto en
  `maps/laberinto_lab_20260702.{yaml,pgm}`.
- La navegacion usa MCL, A*, seguimiento de path y una FSM de navegacion.
- `localizer.py` publica `/amcl_pose` y `/particlecloud`.
- `navigator.py` publica `/cmd_vel`, `/plan`, `/nav_state` y `/nav_debug`.
- `nav_tb4_live.launch.py` remapea `/cmd_vel` a `/<ns>/cmd_vel` para el TB4 real.
- La configuracion real usa `robot_radius + inflation = 0.20 m` para no cerrar
  los pasillos del laberinto.
- La FSM de mision no publica velocidades; emite goals y observa el estado del
  navegador.
- El goal del cono se valida contra mapa crudo/inflado antes de publicarse.
- El caso "cono visto a traves de pared/rejilla" esta cubierto por el rechazo del
  punto estimado sobre obstaculo del mapa crudo.
- El detector de cono publica detecciones y puede publicar imagen/mask de debug.
- El kit de evidencia guarda rosbag, CSV/JSONL, resumen Markdown, timeline y
  overlay de mapa.
- `lab_record_rviz.sh` es opcional y no debe romper si no hay `ffmpeg`.
- El smoke de silla sintetico valida bloqueo seguro ante obstaculo frontal
  persistente.
- Hay validaciones documentadas de `smoke_mission.sh reachable`, `wall` y tests
  de `maze_mission`/`maze_perception`.

## Claims pendientes

No afirmar hasta tener evidencia de laboratorio:

- "El robot completo la mision del cono en el entorno real."
- "El robot evito correctamente una silla real."
- "El sistema navego todo el laberinto real sin intervencion."
- "La MCL converge siempre sin teleop ni giro inicial."
- "La deteccion de cono rojo ignoro todos los distractores reales."
- "Los umbrales HSV son robustos a cualquier iluminacion del laboratorio."
- "El path planificado siempre queda centrado y no roza paredes."
- "La inflacion de 0.20 m es optima."
- "El TB4_1 es intercambiable con TB4_0 sin ajustes."
- "El replay RViz reproduce todos los topicos sin perdida."
- "El robot alcanzo el cono con aproximacion final precisa."
- "La recuperacion ante obstaculos no mapeados siempre encuentra un camino
  alternativo."

Frases permitidas si la prueba real falla parcialmente:

- "Se registro la falla mediante rosbag, eventos estructurados y replay RViz."
- "Se observo un caso de bloqueo seguro ante obstaculo no mapeado."
- "La causa probable se analizo a partir de `nav_debug`, trayectoria y path."
- "El resultado se reporta como brecha sim-to-real y no como validacion completa
  de autonomia."

