# Contrato de interfaz: maze_nav (toma-2) consumido por la Parte C

> Estado: contrato observado del codigo en `toma-2` (rama base de la Parte C).
> La Parte C (`maze_mission`) consume maze_nav como CAJA NEGRA a traves de estos
> topicos. Si el equipo de Parte B cambia algo, actualizar este doc y volver a
> correr el smoke test de interfaz (C4).

## Por que existe este doc

`maze_nav` sigue recibiendo mejoras. Para que la Parte C no se rompa
silenciosamente, fijamos el contrato minimo que consumimos y NADA mas.
`maze_mission` **no importa codigo de maze_nav**: las primitivas de grilla se
vendorizaron en `maze_mission/occupancy.py`, asi que solo dependemos del contrato
de TOPICOS. Tras cada rebase sobre `origin/toma-2`, correr el smoke test de
interfaz: si falla, el contrato cambio.

## La pila de navegacion de toma-2 (3 nodos)

Se lanza con: `ros2 launch maze_nav nav.launch.py map_yaml:=maps/maze_slam.yaml`

| Nodo | Subscribe | Publica |
|---|---|---|
| `map_publisher` | param `map_yaml` (.pgm+.yaml) | `/map` (OccupancyGrid, latched) |
| `localizer` (MCL) | `/map`, `odom_topic` (`/calc_odom`), `scan_topic` (`/scan`), `/initialpose` | **`/amcl_pose`** (PoseWithCovarianceStamped), `/particlecloud` (PoseArray), TF map->odom |
| `navigator` | `/map`, `/amcl_pose`, `/goal_pose`, `scan_topic` | `/cmd_vel` (Twist), `/plan` (Path), `/nav_state` (String) |

## Lo que la mision PRODUCE hacia maze_nav

| Topico | Tipo | Notas |
|---|---|---|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Goal en frame `map` (x, y, yaw). Un goal nuevo PREEMPTA (el navigator pasa a PLANNING). Unico canal de comando; la mision nunca publica `cmd_vel`. |

## Lo que la mision OBSERVA de maze_nav

| Topico | Tipo | Uso en la mision |
|---|---|---|
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Pose del robot (MCL). Es la fuente de pose de la mision (`pose_topic`). |
| `/map` | `nav_msgs/OccupancyGrid` (latched) | Mapa estatico. La mision lo **infla por su cuenta** (occupancy.inflate_occupancy) para validar goals. |
| `/nav_state` | `std_msgs/String` | Estado del navigator: `IDLE`, `PLANNING`, `FOLLOWING`, `ALIGNING`, `REACHED`, `RECOVERY`. |
| `/plan` | `nav_msgs/Path` | Camino planificado (opcional, evidencia). |
| `/particlecloud` | `geometry_msgs/PoseArray` | Nube MCL (evidencia de localizacion en RViz). |

Semantica de `/nav_state` para la FSM de mision:
- **Exito de navegacion = `REACHED`.**
- **Goal inalcanzable**: el navigator loguea "sin celda navegable / no se encontro camino" y vuelve a **`IDLE`** sin pasar por `REACHED`. La mision detecta esto (PLANNING -> IDLE sin REACHED) como fallo del goal.
- **Obstaculo no mapeado**: `RECOVERY` (el navigator re-planifica solo).
- No re-implementar control: la mision solo emite goals y observa.

## Diferencias importantes vs. la otra rama (parte-b-nav)

- **NO existe `/nav_debug`** (no hay telemetria JSON). La mision usa `/nav_state` + `/amcl_pose` (+ `/plan`, `/particlecloud`).
- **NO existe `/global_costmap`**: el navigator infla internamente pero no lo publica. La mision replica el inflado sobre `/map` con el mismo criterio: obstaculo = pared (occ>=lethal) o desconocido (occ<0), bloqueado a < `robot_radius+inflation` (0.14+0.12 = **0.26 m**). Parametro de mision: `inflation_radius_m` (default 0.26).
- **Topicos casi todos hardcodeados** en la nav (`/map`, `/amcl_pose`, `/goal_pose`, `/cmd_vel`, `/plan`, `/nav_state`); solo `scan_topic`/`odom_topic` son parametros. Para el robot real (namespace `/tb4_0/`) el switch se hace por **remap o namespace al lanzar la nav**, no por parametro. (Deuda respecto de la regla "no hardcodear topics", del lado de maze_nav; se coordina con el equipo si molesta.)

## Parametros relevantes de maze_nav (toma-2)

- `navigator`: `robot_radius` (0.14), `inflation` (0.12), `v_max` (0.18), `w_max` (1.2), `goal_tol` (0.12), `yaw_tol` (0.15), `safety_stop` (0.22), `control_rate` (20), `scan_topic`. Para el robot real: bajar `v_max`/`w_max`.
- `localizer`: `n_particles` (400), `sigma_hit`, `scan_topic`, `odom_topic` (`/calc_odom`).
- `map_publisher`: `map_yaml`.

## Localizacion (decision de la Parte C sobre toma-2)

toma-2 trae **MCL propia** (`localizer` -> `/amcl_pose` + `/particlecloud`) sobre el
mapa estatico. Es "localizacion continua con filtros probabilisticos" (lo que pide
la consigna) sin necesidad de FastSLAM live. La mision usa `/amcl_pose` como pose.
El mapa del laberinto YA existe: `maps/maze_slam.yaml` (resolucion 0.03,
origin [-8.25, -8.25]); C3 se reduce a validar la localizacion contra el bag y
usar ese mapa.

## Validacion del contrato (C4)

Verificado en lazo cerrado con un mini-sim cinematico (`scripts/fake_diff_drive.py`:
integra `/cmd_vel` -> `/amcl_pose`, sin Gazebo) + map_publisher + navigator, sobre
el mapa del laberinto. `scripts/smoke_goal_nav.sh` publica un goal y confirma la
secuencia **PLANNING -> FOLLOWING -> ALIGNING -> REACHED** con la pose final a
0.12 m del goal. Es tambien el interface smoke test: correr tras cada rebase sobre
toma-2. No usa `/scan` (el navigator planifica y sigue sobre el mapa estatico; sin
scan no hace evitacion de obstaculos, pero el contrato de goal se valida igual).

## GAP sim-to-real: orientacion del LIDAR real (IMPORTANTE)

El `localizer` (MCL) y el navigator (`_register_obstacle_from_scan`) fueron
escritos para el TB3 simulado: asumen el scan ALINEADO con el frente del robot y
usan `scan_x_offset=-0.032` (base_scan del burger). Pero el TB4 real tiene el
RPLIDAR montado a **+90 deg** (`rplidar_link`, ver `PARTE_C_ESTIMACION_CONO.md`),
offset -0.04 m. El localizer calcula el endpoint del rayo como
`pose + r*dir(yaw + scan_angle)` (sin offset de rotacion), asi que la correccion
MCL quedaria rotada 90 deg contra el mapa -> NO localizaria sobre el bag real.
Fix (porteo Parte A/B a real): sumar `scan_yaw_offset` (+pi/2 real, 0 sim) al
angulo del rayo y usar -0.04 de offset. Cambio chico y retrocompatible en
`maze_nav` (default 0 = comportamiento actual), a COORDINAR con el equipo.
Alternativa sin tocar maze_nav: un shim que republica el scan con
`angle_min += pi/2` (`scripts/scan_reframe.py`).

**C3 validado (consistencia scan<->odom):** `scripts/scan_match_validate.py`
localiza globalmente el primer scan del bag y dead-reckona con odom sobre ~1.8 m.
Con offset **+90 el match scan<->mapa se mantiene en ~0.99**; con 0 (medio 0.79) y
-90 (medio 0.56) diverge a medida que el robot se traslada. Confirma que **+90 es
el offset correcto** y que la MCL localizara con el scan reencuadrado (consistente
con C1: base = scan + 90 deg). Evidencia:
`results/parte_c/C3/scan_odom_consistency.png`. Para el robot real (C6): usar el
shim, o agregar `scan_yaw_offset=+pi/2` al localizer (coordinar con el equipo).

## Dependencia abierta a confirmar con el equipo

El `localizer` subscribe `/calc_odom` (odometria calculada), no `/odom`. Los bags
traen `/tb4_0/odom`. Antes de C3, confirmar como se genera `/calc_odom` en A/B: si
`/tb4_0/odom` sirve directo, se remapea a `/calc_odom` en `ros2 bag play`; si hay
un nodo que la calcula, incluirlo. Ver `scripts/smoke_slam_bag.sh`.

## Convencion de rutas y mapas

Mapas en `maps/` (`casa_slam.*` Parte B, `maze_slam.*` Parte C). `waypoints_file`
se resuelve relativo al CWD; los scripts hacen `cd` a la raiz del repo, por eso los
perfiles usan rutas tipo `config/parte_c/...`.
