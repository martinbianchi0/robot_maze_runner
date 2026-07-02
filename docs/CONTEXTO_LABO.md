# Contexto sesión de laboratorio — Parte C (TB4 real)

Fecha: 2026-07-02. Branch: `labo`.

Resumen de qué se hizo en el labo con los TurtleBot4 reales y, sobre todo, **qué
no anduvo en la navegación / búsqueda del cono**, para retomar sin repetir el
diagnóstico.

---

## 1. Qué SÍ funcionó

- **Mapa (Parte A) en vivo sobre TB4 real**: se grabó un rosbag recorriendo el
  laberinto y se corrió `fastslam_node` en vivo → mapa guardado en
  `maps/laberinto_lab_20260702.{pgm,yaml}` (500×500 @ 0.03 m, origin (-7.5,-7.5)).
  Este es el mapa que usamos para Parte B/C.
- **Detección del cono rojo**: el `cone_detector` detecta y publica en
  `/cone_detections` correctamente (con la cámara prendida).
- **Nav stack levanta y planifica**: map_publisher + localizer (MCL) + navigator
  cargan el mapa, construyen costmap y el navigator llega a mandar `/cmd_vel`.
- **MCL converge** a ~0.09 m de spread **cuando el robot se mueve** (ver deadlock
  abajo).

## 2. Bugs encontrados y arreglados en esta branch

1. **`/cmd_vel` sin namespace** — el navigator publicaba en `/cmd_vel` pero el TB4
   escucha en `/<ns>/cmd_vel` → el robot NO se movía. **Fix**: remap
   `/cmd_vel -> /<ns>/cmd_vel` en `nav_tb4_live.launch.py`.

2. **TF `map->odom` desconectada** — `localizer.py` hardcodeaba
   `child_frame_id='calc_odom'` (frame de la sim). El TB4 usa `odom`. Sin esto la
   cadena TF no cerraba y RViz dropeaba el scan ("Message Filter dropping
   message"). **Fix**: `localizer.py` auto-detecta el frame del odom del mensaje.

3. **Todos los `/tf` al namespace del robot** — map_publisher/localizer/navigator
   y RViz remapean `/tf` y `/tf_static` a `/<ns>/tf(_static)`. Sin esto RViz no ve
   la corrección del SLAM/MCL.

## 3. Qué NO anduvo (lo importante para retomar)

### 3.1. Deadlock de localización (MCL solo actualiza en movimiento)

`localizer.py` (líneas ~173-180): el measurement update + resample **solo corren
cuando el robot se movió** (`update_min_d=0.05`, `update_min_a=0.05`). Con el
robot quieto en el estado LOCALIZE, la MCL nunca pesa el scan → el spread queda
clavado en ~0.43 m (el spread inicial) → nunca baja del umbral (0.25) → el
`mission_node` nunca sale de LOCALIZE → nunca busca ni va al cono.

Es un **deadlock**: no navega porque no localiza, no localiza porque no se mueve.

**Workaround usado en el labo**: mover el robot a mano con teleop para que la MCL
enganche:
```
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/tb4_0
```
Girar ~1 vuelta en el lugar → la MCL converge (~0.09) → pasa a SEARCH_CONE.
Cerrar el teleop después (pelea con el navigator por `/cmd_vel`).

**Pendiente / mejora**: que el estado LOCALIZE del `mission_node` comande un
giro-scan en el lugar automáticamente (la branch dice tener "giro-scan en
waypoints" pero el LOCALIZE inicial sigue pasivo). Sin eso, siempre hay que
teleopear al arranque.

### 3.2. Waypoints no matcheaban nuestro mapa → misión abortaba

Los waypoints de `config/parte_c/waypoints_laberinto.yaml` fueron generados sobre
`maps/maze_slam.yaml`, no sobre `laberinto_lab_20260702`. Sus coordenadas caían
fuera del free space de nuestro mapa → los 13 se rechazaban ("sin celda libre
cercana / posible cono tras pared") → "waypoints agotados" → **MISION FALLIDA**.

**Fix**: regenerados sobre nuestro mapa:
```
python3 scripts/gen_search_waypoints.py \
    --map maps/laberinto_lab_20260702.yaml --start 0.0 0.0 --inflation 0.20 \
    --out config/parte_c/waypoints_lab_tb4_0.yaml
```
(16 waypoints). `real.yaml` ya apunta a este archivo.

### 3.3. Inflación 0.26 sellaba el laberinto (causa raíz de "no navega")

Los pasillos del laberinto real son angostos. Con inflación 0.26 m (el default:
`robot_radius 0.14 + inflation 0.12` en el navigator) solo quedan **~5 m²**
alcanzables desde el arranque — el resto del laberinto (y el cono, si está más
allá) queda **inaccesible**, así que todo goal se rechaza.

Barrido con el generador (área libre alcanzable desde origen):
| inflación | área alcanzable |
|-----------|-----------------|
| 0.26 m    | 5.1 m² (solo el tercio superior) |
| 0.22 m    | 9.6 m² |
| **0.20 m**| **9.6-10 m² (cubre TODO el laberinto)** |
| 0.18 m    | 11.2 m² |

**Decisión**: bajar a **0.20 m** en los TRES lugares (deben coincidir o no sirve):
- `config/parte_c/real.yaml` → `inflation_radius_m: 0.20` (validación del cono/goals)
- `src/maze_nav/launch/nav_tb4_live.launch.py` → navigator `robot_radius 0.14 + inflation 0.06`
- generador de waypoints → `--inflation 0.20`

Nota sim-to-real: 0.20 con un TB4 de radio ~0.17 deja poco margen a las paredes.
Velocidades ya conservadoras (`v_max 0.12`). Si roza paredes, subir un poco y
regenerar waypoints; si no llega al cono, bajar.

### 3.4. Estado al terminar la sesión

Los cambios de 3.2 y 3.3 (waypoints nuevos + inflación 0.20 en los tres lados) se
hicieron pero **NO se probaron end-to-end** — nos fuimos antes. Falta verificar:
1. Con inflación 0.20, ¿el robot navega el laberinto completo hasta el cono?
2. ¿El goal del cono (detección → `ESTIMATE_CONE_GOAL`) cae en celda navegable y
   el robot se acerca? (antes se rechazaba por la inflación 0.26).

## 4. Quirks del hardware / red (TB4 del labo)

- **Discovery server**, no multicast. Conectarse con `turtlebot_mode <id>`
  (0=IP .87/dom0, 1=IP .70/dom1). Hace `sudo ip route` + super client. Si el
  `ros2 topic list` da 2 topics, no estás conectado al robot correcto.
- **LIDAR y cámara arrancan apagados**. Prenderlos por servicio:
  ```
  ros2 service call /<ns>/start_motor std_srvs/srv/Empty {}
  ros2 service call /<ns>/oakd/start_camera std_srvs/srv/Empty {}
  ```
  El `start_motor` responde OK aunque a veces tarda en girar; verificar con
  `ros2 topic hz /<ns>/scan` (~8 Hz cuando anda).
- **tb4_0** (IP .87): cámara estándar `/tb4_0/oakd/rgb/preview/image_raw` (con
  camera_info). Se quedó sin batería una vez en la sesión; revivió al recargar.
- **tb4_1** (IP .70): cámara con naming distinto `/tb4_1/color/preview/image`
  y **sin camera_info** → el detector usa `fallback_hfov_deg` (69°). Perfil en
  `config/parte_c/real_tb4_1.yaml`.
- **Skew de reloj robot vs laptop**: RViz tira "Lookup would require extrapolation
  into the future" y dropea scans. Es cosmético (la nav usa sus propios buffers),
  pero conviene sincronizar NTP para que RViz muestre limpio.
- El daemon de ROS queda stale al cambiar de `turtlebot_mode`; si `ros2 topic
  list` miente, `ros2 daemon stop && ros2 daemon start`.

## 4.5. Shortcuts A y B (tb4_0 / tb4_1 seamless)

Para las partes A (mapeo) y B (nav) sobre el TB4 real, sin escribir launch a mano:

```bash
export ROS_DOMAIN_ID=<el-que-usa-el-TB4>

# Parte A live: FastSLAM del laberinto real. Manejar por teleop.
./shs/mapear_tb4.sh --ns tb4_1
# Cuando el mapa este lindo, en otra terminal:
./shs/save_map.sh laberinto_lab_$(date +%Y%m%d)

# Parte B live: nav sobre el mapa recien hecho.
./shs/navegar_tb4.sh --ns tb4_1                # agarra el .yaml mas reciente
./shs/navegar_tb4.sh --ns tb4_1 --map maps/otro.yaml --v-max 0.10
```

Cambiar de robot es cambiar `--ns tb4_0` <-> `--ns tb4_1` (default tb4_0). Ambos
scripts:
- Corren `kill_all.sh` primero (evita nodos zombies).
- Chequean con `tb4_precheck` que el TB4 este publicando `/<ns>/scan` y
  `/<ns>/odom` antes de arrancar. Aborta con instrucciones si no ve nada
  (ROS_DOMAIN_ID, red, ns equivocada).
- Levantan RViz con `/tf` y `/tf_static` remapeados a `/<ns>/*` (igual que la
  launch, si no RViz dropea el scan).
- `navegar_tb4.sh` publica `cmd_vel=0` en el trap EXIT (Ctrl+C deja al TB4 quieto).

## 5. Cómo relanzar Parte C (tb4_0)

```bash
turtlebot_mode 0
ros2 service call /tb4_0/start_motor std_srvs/srv/Empty {}
ros2 service call /tb4_0/oakd/start_camera std_srvs/srv/Empty {}

# T1 nav:
ros2 launch maze_nav nav_tb4_live.launch.py \
    map_yaml:=maps/laberinto_lab_20260702.yaml ns:=tb4_0
# T2 mission:
ros2 launch maze_mission mission.launch.py \
    params_file:=$(pwd)/config/parte_c/real.yaml
# T3 cono:
ros2 launch maze_perception cone_detector.launch.py \
    params_file:=$(pwd)/config/parte_c/real.yaml
# T4 rviz (config con panel de cámara en scratchpad, o nav.rviz):
rviz2 -d src/maze_nav/rviz/nav.rviz --ros-args -p use_sim_time:=false \
    --remap /tf:=/tb4_0/tf --remap /tf_static:=/tb4_0/tf_static --remap /scan:=/tb4_0/scan
```
En RViz: `2D Pose Estimate`, y **teleopear para que la MCL converja** (ver 3.1).
Después el mission patrulla waypoints y va al cono al detectarlo.

## 6. Archivos tocados en esta branch

- `src/maze_nav/maze_nav/localizer.py` — auto-detección del frame de odom.
- `src/maze_nav/launch/nav_tb4_live.launch.py` — launch TB4 real parametrizado por
  `ns`, con remaps de `/tf` y `/cmd_vel`, inflación 0.20.
- `config/parte_c/real.yaml` — inflación 0.20 + waypoints nuevos (tb4_0).
- `config/parte_c/real_tb4_1.yaml` — perfil tb4_1 (cámara distinta, sin camera_info).
- `config/parte_c/waypoints_lab_tb4_0.yaml` — waypoints regenerados sobre nuestro mapa.
- `maps/laberinto_lab_20260702.{pgm,yaml}` — mapa de la Parte A del labo (forzado a
  trackear pese al .gitignore, para no perderlo).
