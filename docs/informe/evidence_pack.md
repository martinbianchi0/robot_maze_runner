# Evidence pack tecnico para informe

Fecha de preparacion: 2026-07-02.

Este documento es un volcado tecnico para armar el informe final. No busca ser
elegante: busca dejar trazabilidad entre consigna, codigo, decisiones y evidencia
disponible. No afirma resultados reales del proximo turno de laboratorio.

## Estado del repo

- Repositorio: `git@github.com:martinbianchi0/robot_maze_runner.git`.
- Branch actual: `labo`.
- Commit actual al preparar este pack: `c029498` (`labo: --bag para probar los shs sin robot`).
- Estado local luego de sincronizar: `labo` alineada con `origin/labo`.
- Comando usado para sincronizar:

```bash
git fetch --all --prune
git switch labo
git pull --ff-only origin labo
git status -sb
git log --oneline --decorate --graph -20
```

Commits recientes relevantes:

```text
c029498 (HEAD -> labo, origin/labo) labo: --bag para probar los shs sin robot
9ac95f8 labo: shs y SLAM del TB4 asumen maze (no casa)
a47fa50 labo: shortcuts para Partes A y B en el TB4 real (tb4_0 / tb4_1 seamless)
c68b6c5 Merge labo evidence and replay workflow
2173b38 labo: add rviz replay video workflow
e696e02 labo: add evidence kit and safer obstacle recovery
da685f6 labo: sesion TB4 real Parte C - fixes nav + mapa + contexto
```

Que agrego Giner arriba del merge de evidencia:

- `a47fa50`: shortcuts para Parte A y Parte B sobre TB4 real, con soporte
  `tb4_0`/`tb4_1`, prechecks de topics y comandos de RViz.
- `9ac95f8`: ajustes para que los scripts y SLAM del TB4 asuman el laberinto
  (`maze`) y no la casa de simulacion.
- `c029498`: modo `--bag` en los scripts de TB4 para validar sin robot usando
  rosbag y `use_sim_time:=true`.

Confirmacion de que el kit de evidencia/replay sigue presente:

- `scripts/lab_record_all.sh`
- `scripts/lab_live_logger.py`
- `scripts/lab_make_report.py`
- `scripts/lab_record_rviz.sh`
- `scripts/lab_replay_rviz.sh`
- `scripts/lab_viz_markers.py`
- `scripts/smoke_chair_obstacle.sh`
- `rviz/lab_replay.rviz`
- `docs/decisiones/PARTE_C_EVIDENCIA_LABO.md`
- `docs/decisiones/LABO_COOK_20260702.md`

## Branches relevantes

- `final/ab-casa`: rama estable de Parte A+B en simulacion/casa/TurtleBot3. No
  se toca para el trabajo de laboratorio real.
- `labo`: rama real actual de laboratorio. Contiene mapa real, perfiles del TB4,
  fixes de topics/TF, kit de evidencia y shortcuts nuevos.
- `labo-cook-evidence-nav`: rama de trabajo donde se implemento el kit de
  evidencia, replay RViz y recovery mas seguro ante obstaculos finos. Ya fue
  mergeada en `labo`.
- `origin/toma-2`: aparece en documentacion como base conceptual/contrato de
  navegacion que consumio Parte C.
- `origin/feat/parte-c-robot-real`: rama previa de Parte C. No debe mergearse a
  ciegas; se usa solo como referencia si hace falta.

## Objetivo del TP final segun consignas

El TP se organiza en tres partes conectadas:

1. Parte A: simultaneous localization and mapping (SLAM), construccion de mapa y
   estimacion de pose.
2. Parte B: navegacion autonoma con localizacion probabilistica, planificacion,
   seguimiento de trayectoria, orientacion final, replanning y obstaculos no
   mapeados.
3. Parte C: autonomia completa en robot real, con busqueda de conos rojos,
   filtrado de distractores y analisis sim-to-real.

La rama elegida para el informe es Resultado V1 de la consigna: mapa grillado de
ocupacion. No se mezclan landmarks de LIDAR ni Graph SLAM visual salvo como
alternativas descartadas o futuras.

Puntos de consigna que conviene reforzar en el informe:

- el mapa final debe ser usable para planificacion;
- la navegacion no puede seguir un path ciegamente ante obstaculos no mapeados;
- el robot debe planificar un camino valido hacia el cono, incluso si lo ve a
  traves de una abertura, rejilla o hueco;
- la defensa debe apoyarse en videos, capturas, rosbags, mapas y metricas;
- las fallas parciales del robot real deben analizarse como brecha sim-to-real.

## Arquitectura general

Flujo de alto nivel:

```text
sensores / robot real / simulacion
-> SLAM / mapa
-> localizacion MCL
-> navegacion A*
-> seguimiento del camino
-> percepcion del cono
-> FSM de mision
-> evidencia, rosbag y replay RViz
```

Archivos principales:

- SLAM: `src/maze_slam/maze_slam/fastslam.py`,
  `src/maze_slam/maze_slam/fastslam_node.py`.
- Launch de SLAM real: `src/maze_slam/launch/slam_tb4_live.launch.py`.
- Localizacion: `src/maze_nav/maze_nav/localizer.py`.
- Navegacion: `src/maze_nav/maze_nav/navigator.py`.
- Launch de navegacion real: `src/maze_nav/launch/nav_tb4_live.launch.py`.
- Percepcion: `src/maze_perception/maze_perception/cone_detector_node.py`.
- Mision: `src/maze_mission/maze_mission/mission_node.py`.
- Perfil real: `config/parte_c/real.yaml`.
- Waypoints reales: `config/parte_c/waypoints_lab_tb4_0.yaml`.
- Mapa real: `maps/laberinto_lab_20260702.yaml` y
  `maps/laberinto_lab_20260702.pgm`.

## Parte A - SLAM / mapa

Paquete: `src/maze_slam`.

Implementacion:

- `fastslam.py`: algoritmo Grid-Based FastSLAM. Cada particula mantiene pose y
  mapa de ocupacion en log-odds. Se usa modelo de movimiento por odometria,
  scan matching local, likelihood field, actualizacion del mapa y resampling.
- `fastslam_node.py`: envoltorio ROS 2. Consume `LaserScan` y `Odometry`,
  publica `/map`, `/belief`, `/maze_slam/particles` y TF `map->odom`. Tambien
  escucha `/maze_slam/save_request_named` para guardar el mapa.
- `_kernels.py`: kernels vectorizados/JIT para scan matching e integracion de
  rayos cuando el entorno lo permite.

Launchs y scripts:

- `src/maze_slam/launch/slam_casa.launch.py`: Parte A en casa/simulacion.
- `src/maze_slam/launch/slam_tb4_live.launch.py`: Parte A en TB4 real o rosbag.
- `shs/mapear_tb4.sh`: shortcut para mapear sobre TB4 real.
- `shs/save_map.sh`: solicita al nodo que guarde `maps/<nombre>.pgm/.yaml`.

Mapa real disponible:

```text
maps/laberinto_lab_20260702.yaml
maps/laberinto_lab_20260702.pgm
```

Metadatos del mapa real:

```yaml
resolution: 0.03
origin: [-7.5, -7.5, 0.0]
size: 500 x 500
```

Topicos principales:

- Entrada TB4 real: `/<ns>/scan`, `/<ns>/odom`, `/<ns>/tf`, `/<ns>/tf_static`.
- Salida SLAM: `/map`, `/belief`, `/maze_slam/particles`, TF `map->odom`.

Limitaciones para el informe:

- El mapa real existe y esta versionado, pero la calidad visual final debe
  documentarse con captura/overlay para el informe.
- Si se usa el mapa de laboratorio de 2026-07-02, debe aclararse que se obtuvo
  sobre TB4 real y se reutiliza para Parte B/C.
- No afirmar cierre perfecto del laberinto ni metricas de error no medidas.

## Parte B - navegacion

Paquete: `src/maze_nav`.

Nodos:

- `map_publisher`: publica el mapa estatico en `/map`.
- `localizer`: Monte Carlo localization (MCL) sobre mapa de ocupacion.
- `navigator`: planificador A*, seguimiento tipo pure pursuit/control de heading,
  FSM de navegacion, recovery y telemetria.

### Localizacion MCL

Archivo: `src/maze_nav/maze_nav/localizer.py`.

Consume:

- `/map`
- `odom_topic` (`/calc_odom` en simulacion, `/<ns>/odom` en TB4 real)
- `scan_topic` (`/scan` o `/<ns>/scan`)
- `/initialpose`

Publica:

- `/amcl_pose`
- `/particlecloud`
- TF `map->odom` usando el frame detectado desde la odometria.

Parametros relevantes:

- `n_particles`
- `sigma_hit`
- `max_beams`
- `alpha1..alpha4`
- `update_min_d`
- `update_min_a`
- `init_xy_std`
- `init_yaw_std`
- `scan_x_offset`
- `scan_yaw_offset`

Para TB4 real, `nav_tb4_live.launch.py` usa `scan_yaw_offset:=1.5708` y
`scan_x_offset:=-0.04`.

### Planificacion y costmap

Archivo: `src/maze_nav/maze_nav/navigator.py`.

El costmap se construye con mapa estatico, celdas desconocidas y obstaculos
dinamicos. La inflacion bloquea celdas a distancia menor que
`robot_radius + inflation`.

Planificacion:

- A* 8-conectado.
- Heuristica Euclidiana.
- Penalizacion por cercania a obstaculos:

```text
prox = max(0, robot_radius + inflation + clearance_extra_m - distancia_a_obstaculo)
costo += prox * clearance_weight
```

Parametros relevantes:

- `robot_radius`
- `inflation`
- `safety_stop`
- `max_recovery_attempts`
- `clearance_extra_m`
- `clearance_weight`
- `recovery_hold_s`
- `recovery_backoff_s`
- `front_obstacle_mark_radius`

En el laberinto real se usa `robot_radius=0.14` e `inflation=0.06`, total
`0.20 m`, porque `0.26 m` sellaba pasillos angostos.

### Seguimiento del path

El seguimiento usa un indice monotono del path y un punto de lookahead para
evitar volver hacia puntos ya superados. Si el error angular es grande, el robot
rota antes de avanzar. Al final, el estado `ALIGNING` ajusta yaw final.

Publicaciones:

- `/cmd_vel`
- `/plan`
- `/nav_state`
- `/nav_debug`

Estados principales:

```text
IDLE -> PLANNING -> FOLLOWING -> ALIGNING -> REACHED
                    |
                    +-> RECOVERY -> FOLLOWING/IDLE
```

### Obstaculos no mapeados y caso silla/patas

El problema observado en laboratorio fue un loop ante silla/patas: el robot
avanzaba, detectaba algo, retrocedia, dejaba de verlo y volvia a intentar por el
mismo lugar.

Cambios defensivos en `navigator.py`:

- si el clearance frontal baja de `safety_stop`, entra en `RECOVERY` y frena;
- mantiene obstaculos dinamicos durante el goal actual;
- si el bloqueo se repite, marca una barrera dinamica delante del robot;
- si supera `max_recovery_attempts`, queda detenido con `cmd_vel=0`;
- publica la causa en `/nav_debug` (`blocked_max_recovery_attempts` o
  `blocked_no_recovery_plan`).

Smoke sintetico:

```bash
./scripts/smoke_chair_obstacle.sh
```

Resultado documentado:

```text
[smoke_chair_obstacle] OK
cmd_linear=[0.0, -0.05, 0.0, -0.05, 0.0]
dyn_obstacle_cells=50
final_debug.reason=blocked_max_recovery_attempts
```

## Parte C - robot real y cono

Paquetes: `src/maze_mission` y `src/maze_perception`.

### FSM de mision

Archivo: `src/maze_mission/maze_mission/mission_node.py`.

Rol: supervision. La FSM no publica `/cmd_vel`; emite goals validados en
`/goal_pose` y observa `/nav_state`.

Estados:

```text
INIT -> LOAD_MAP -> LOCALIZE -> SEARCH_CONE
  -> CONE_DETECTED -> ESTIMATE_CONE_GOAL -> PLAN_TO_CONE -> NAVIGATE_TO_CONE
  -> (AVOID_OBSTACLE -> REPLAN)* -> VERIFY_CONE -> DONE
  (FAILURE como aborto seguro)
```

Invariante importante:

- El unico metodo que publica `/goal_pose` es `_emit_goal()`.
- Antes de emitir, valida el goal contra mapa inflado.
- Si el cono estimado cae sobre obstaculo del mapa crudo, se rechaza. Este es el
  caso "cono visto a traves de pared/rejilla".

Validacion mockeada documentada:

```bash
bash scripts/smoke_mission.sh reachable
bash scripts/smoke_mission.sh wall
```

Resultados documentados:

- escenario `reachable`: `DONE`, 1 goal, 0 goals en pared;
- escenario `wall`: rechazo, 0 goals hacia pared;
- `all_goals_free = true`.

### Detector de cono

Archivo: `src/maze_perception/maze_perception/cone_detector_node.py`.

Funcionamiento:

- suscribe imagen de camara y `CameraInfo`;
- segmenta rojo en HSV;
- extrae blobs;
- publica detecciones JSON en `cone_detections`;
- opcionalmente publica `/cone_debug_image` y `/cone_mask`.

La deteccion produce bearing, centroide, area y confianza en imagen. No decide
goals ni control. La estimacion metrica se hace en mision con fusion LIDAR.

Validacion documentada:

- LIDAR-fusion validado contra rosbag `laberinto_conos`.
- Umbral HSV ajustado con saturacion alta para separar cono rojo de distractores
  naranja/madera.
- Debug online con `scripts/smoke_cone_detect.sh` y capturas en
  `results/parte_c/C2/`.

### Perfil real

Archivo: `config/parte_c/real.yaml`.

Puntos importantes:

- imagen: `/tb4_0/oakd/rgb/preview/image_raw`;
- camera info: `/tb4_0/oakd/rgb/preview/camera_info`;
- scan: `/tb4_0/scan`;
- pose: `/amcl_pose`;
- `lidar_yaw_offset: -1.5708` para fusion del cono;
- `inflation_radius_m: 0.20`;
- waypoints: `config/parte_c/waypoints_lab_tb4_0.yaml`;
- `publish_debug: true` para generar imagen/mask de evidencia.

## Sim-to-real

Problemas reales documentados:

- TB4 publica sensores bajo namespace (`/tb4_0` o `/tb4_1`).
- El TB4 escucha `/<ns>/cmd_vel`; publicar en `/cmd_vel` global no mueve el robot.
- El frame de odometria real es `odom`; el localizer ahora detecta el frame del
  mensaje para que TF `map->odom` cierre correctamente.
- RViz debe remapear `/tf` y `/tf_static` hacia `/<ns>/tf` y `/<ns>/tf_static`.
- El LIDAR del TB4 tiene montaje aproximado `+90 deg` y offset `-0.04 m`.
- Inflacion `0.26 m` sellaba pasillos; `0.20 m` habilita el laberinto pero deja
  menos margen fisico.
- La MCL actualiza con movimiento; si el robot queda quieto al inicio, puede
  requerir teleop/giro inicial para converger.
- `tb4_0` y `tb4_1` difieren en topics de camara y disponibilidad de
  `camera_info`.
- En la computadora de facultad puede variar la disponibilidad de `ffmpeg`,
  display grafico, overlays de ROS y paquetes Python.

Como reportarlo:

- Si el robot completa la mision, reportar corrida y evidencia.
- Si falla parcialmente, reportar logs, estado, `nav_debug`, mapa, path y causa
  observada. La consigna habilita evaluar la solidez metodologica y el analisis
  sim-to-real, no solo el exito perfecto.

## Evidencia y replay

Kit de evidencia:

- `scripts/lab_record_all.sh`: crea `results/labo_demo/<timestamp>/`, graba
  rosbag, ejecuta logger, guarda metadata y genera reporte al cortar.
- `scripts/lab_live_logger.py`: escribe `events.jsonl`, `poses.csv`,
  `goals.csv`, `states.csv`, `cmd_vel.csv`, `summary.json`, ultimo path y updates.
- `scripts/lab_make_report.py`: genera `summary.md`, `timeline.csv` y
  `map_overlay.png`; si no hay `matplotlib`, cae a `map_overlay.svg`.
- `scripts/lab_record_rviz.sh`: intenta grabar RViz con `ffmpeg`; si no existe,
  no rompe y deja instrucciones.
- `scripts/lab_replay_rviz.sh`: reproduce el rosbag de una corrida y abre RViz
  con `use_sim_time:=true`.
- `scripts/lab_viz_markers.py`: publica markers de trayectoria, goal, ultimo
  path, texto de estados/debug y rayo de deteccion del cono.
- `rviz/lab_replay.rviz`: configuracion de replay para video/captura.

Topicos que deben quedar en rosbag:

```text
/tb4_0/scan
/tb4_0/odom
/tb4_0/cmd_vel
/tb4_0/oakd/rgb/preview/image_raw
/tb4_0/oakd/rgb/preview/camera_info
/map
/amcl_pose
/particlecloud
/goal_pose
/nav_state
/nav_debug
/plan
/cone_detections
/cone_debug_image
/cone_mask
/mission_state
/tf
/tf_static
/tb4_0/tf
/tb4_0/tf_static
```

## Resultados confirmados

Confirmados por codigo/docs/smokes:

- Existe mapa real versionado: `maps/laberinto_lab_20260702.{yaml,pgm}`.
- Los launchs reales aceptan namespace `tb4_0`/`tb4_1`.
- `nav_tb4_live.launch.py` remapea `/cmd_vel` a `/<ns>/cmd_vel`.
- `localizer.py` autodetecta frame de odometria.
- `navigator.py` publica `/nav_debug`.
- La FSM rechaza goals de cono sobre pared cruda en el smoke `wall`.
- El smoke de silla sintetico termina en bloqueo seguro y `cmd_vel=0`.
- El kit de evidencia genera rosbag, logs, resumen y overlay post-run.

Validaciones documentadas antes de este borrador:

```text
Build temporal limpio: Summary: 5 packages finished
Tests: maze_perception 10 passed, maze_mission 40 passed, maze_nav 0 tests
colcon test-result: 50 tests, 0 errors, 0 failures, 0 skipped
smoke_mission reachable: PASS
smoke_mission wall: PASS
smoke_chair_obstacle: OK
```

Resultados NO confirmados todavia en robot real:

- Que el robot complete end-to-end la mision del cono en el laberinto real.
- Que el robot evite una silla real con exito, no solo que haga safe stop.
- Que la MCL inicial converja sin teleop/giro manual.
- Que los umbrales HSV sean optimos con la iluminacion de manana.
- Que `tb4_1` pueda reemplazar a `tb4_0` sin ajustar RViz/camera topics.

## Limitaciones conocidas

- El build normal sobre `build/` local puede fallar por artefactos generados
  viejos; las validaciones limpias usan bases temporales.
- El informe no debe subir rosbags ni videos pesados salvo decision explicita.
- El perfil real usa inflacion `0.20 m`, compromiso entre alcanzabilidad y margen
  fisico.
- El detector de color puede requerir retune por iluminacion.
- La FSM de Parte C esta validada con percepcion mockeada y parcialmente con bags,
  pero falta evidencia end-to-end real.

## Comandos

Build limpio temporal:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/tp_final_ws_log_report build --symlink-install \
  --build-base /tmp/tp_final_ws_build_report \
  --install-base /tmp/tp_final_ws_install_report
```

Tests:

```bash
source /opt/ros/humble/setup.bash
source /tmp/tp_final_ws_install_report/setup.bash
colcon --log-base /tmp/tp_final_ws_log_report_test test \
  --build-base /tmp/tp_final_ws_build_report \
  --install-base /tmp/tp_final_ws_install_report \
  --test-result-base /tmp/tp_final_ws_build_report \
  --packages-select maze_nav maze_mission maze_perception \
  --event-handlers console_direct+
colcon test-result --test-result-base /tmp/tp_final_ws_build_report --verbose
```

Mapeo TB4:

```bash
turtlebot_mode 0
ros2 service call /tb4_0/start_motor std_srvs/srv/Empty {}
./shs/mapear_tb4.sh --ns tb4_0
./shs/save_map.sh laberinto_lab_$(date +%Y%m%d)
```

Mapeo TB4 con bag:

```bash
./shs/bag.sh
./shs/mapear_tb4.sh --bag
```

Navegacion TB4:

```bash
./shs/navegar_tb4.sh --ns tb4_0 --map maps/laberinto_lab_20260702.yaml
```

Parte C real:

```bash
ros2 launch maze_nav nav_tb4_live.launch.py \
  map_yaml:=maps/laberinto_lab_20260702.yaml ns:=tb4_0

ros2 launch maze_mission mission.launch.py \
  params_file:=$(pwd)/config/parte_c/real.yaml

ros2 launch maze_perception cone_detector.launch.py \
  params_file:=$(pwd)/config/parte_c/real.yaml
```

Grabacion de evidencia:

```bash
./scripts/lab_record_all.sh tb4_0
```

RViz:

```bash
rviz2 -d src/maze_nav/rviz/nav.rviz --ros-args -p use_sim_time:=false \
  --remap /tf:=/tb4_0/tf \
  --remap /tf_static:=/tb4_0/tf_static \
  --remap /scan:=/tb4_0/scan
```

Replay RViz:

```bash
./scripts/lab_replay_rviz.sh results/labo_demo/<timestamp> tb4_0
```

Grabacion de pantalla:

```bash
./scripts/lab_record_rviz.sh results/labo_demo/<timestamp>
```

Postprocesamiento:

```bash
python3 scripts/lab_make_report.py results/labo_demo/<timestamp>
```

