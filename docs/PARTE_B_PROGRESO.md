# Parte B — Navegación autónoma — Estado de avance

Última actualización: sesión del 2026-07-01 (madrugada).
Branch: `toma-2`.

## Qué es la Parte B

Navegación autónoma del TB3 en la casa, usando el mapa de la Parte A.
El robot debe: localizarse (filtro probabilístico), planear un camino a un objetivo, seguirlo, llegar a la posición **y** al ángulo final, re-planear si cambia el goal, esquivar obstáculos no mapeados, todo orquestado por una máquina de estados.
Entornos de prueba: `custom_casa.launch.py` y `custom_casa_obs.launch.py`.
La localización usa **solo** `/calc_odom` (ruidosa) + LIDAR + mapa. NUNCA `/odom` (ground truth) — eso sería trampa; el GT solo sirve para medir el error.

## Lo que ya está construido (paquete nuevo `src/maze_nav/`)

Arquitectura: 3 nodos desacoplados + utils.

- `maze_nav/nav_utils.py` — carga de mapas .pgm/.yaml (formato map_server), conversiones grilla↔mundo, modelo de movimiento por odometría (muestreo, Tabla 5.6). Sin ROS, testeable aislado.
- `maze_nav/map_publisher.py` — publica el mapa de la Parte A en `/map` (latched, transient_local). Param `map_yaml`.
- `maze_nav/localizer.py` — **MCL** (filtro de partículas). Predicción con `/calc_odom`, corrección con likelihood field (modelo de mezcla). Publica `/amcl_pose`, `/particlecloud` y TF `map→calc_odom`. Init con `/initialpose`.
- `maze_nav/navigator.py` — **A\*** (grilla inflada con margen de seguridad) + **pure-pursuit** + evasión de obstáculos no mapeados + **máquina de estados** (IDLE→PLANNING→FOLLOWING→ALIGNING→REACHED, más RECOVERY). Publica `/cmd_vel`, `/plan`, `/nav_state`. Objetivo por `/goal_pose`.
- `launch/nav.launch.py` — levanta los 3 nodos. Arg `map_yaml`.
- `rviz/nav.rviz` — con herramientas "2D Pose Estimate" (`/initialpose`) y "2D Goal Pose" (`/goal_pose`), muestra mapa, partículas, pose estimada, plan, scan.
- `shs/nav.sh` — corre build + nav + RViz. Elige el mapa (default `maps/casa_slam.yaml`, fallback al de la cátedra). Flag `--no-rviz`, o pasarle `/ruta/mapa.yaml`.
- `setup.py` + `setup.cfg` + `package.xml` — OJO: el `setup.cfg` es obligatorio (sin él los ejecutables no se instalan en `lib/maze_nav/` y `ros2 launch` falla con "libexec directory does not exist").

También:
- `maps/casa_slam.pgm` + `.yaml` — mapa de la casa generado con NUESTRO SLAM (exploración automática). **Está registrado bien** (el overlay scan/mapa alinea) pero es **ruidoso/incompleto** (la exploración auto de 95s no cubrió todo con paredes limpias).
- `shs/kill_all.sh` — se le agregaron `map_publisher`, `localizer`, `navigator`.

## Cómo probarlo (3 terminales)

```
T1: ./shs/casa.sh
T2: ./shs/nav.sh
En RViz: "2D Pose Estimate" en donde está el robot, después "2D Goal Pose" en el destino.
```

## Estado real / qué anda y qué no

- **Compila y corre** todo el pipeline (build OK, los 3 nodos levantan, publican).
- **El pipeline end-to-end funciona parcialmente**: en un test localizó, planeó y **manejó hacia el goal** (llegó a mitad de camino) antes de que un problema lo frenara.
- **Problema principal SIN resolver: la localización MCL es inestable** sobre el mapa auto-generado (ruidoso). El filtro trackea bien por momentos (err ~0.15 m) y de golpe salta a un modo simétrico equivocado (err 2–5 m). Colapsa entre modos al resamplear.
- Config actual del localizer: N=600, sigma_hit=0.35 (gentil), max_beams=30, resample si Neff<N/3, estimación por **cluster dominante con histéresis temporal** (seguir el modo cercano al estimado anterior). Última prueba de esto quedó sin verificar (el test de evaluación crasheaba por `/odom` intermitente, ver gotchas).

## Gotchas críticos descubiertos (NO volver a tropezar)

1. **Zombies de python** — `kill_all.sh` NO mata `python3 /tmp/xxx.py` (mis scripts de test). Procesos de tests anteriores quedaban vivos publicando `/cmd_vel` de fondo → el robot derivaba solo y `/calc_odom` fugaba (se veía como localización rota). SIEMPRE `pkill -9 -f "python3 /tmp/"` antes de cada test. Esto contaminó MUCHOS tests.
2. **Mapa de la cátedra mal registrado** — `src/turtlebot3_custom_simulation/worlds/map/map.yaml` NO alinea con el mundo real de la sim (el overlay scan/mapa no coincide). Por eso hay que usar NUESTRO mapa (`maps/casa_slam.yaml`), que sí está registrado al frame del robot.
3. **TF de la casa**: el static `map→calc_odom` de la cátedra NO arranca (le pusieron el mismo nombre de nodo a dos static_transform_publisher → colisión), así que nuestro localizer publica `map→calc_odom` sin conflicto. El `base_scan` cuelga de `odom→base_footprint` (Gazebo, ≈GT), NO de `calc_odom`. Por eso el control usa `/amcl_pose` (topic), no el TF del scan.
4. **`/odom` (GT) es intermitente** al arrancar Gazebo — a veces tarda >20s en publicar o solo publica tras mover el robot. Para tests que lo necesiten como referencia: dar un "nudge" (publicar cmd_vel un momento) y esperar. El localizer NO lo usa (usa `/calc_odom`).
5. **`setup.cfg` obligatorio** en paquetes ament_python (ver arriba).

## Pendiente (próximos pasos, en orden)

1. **Estabilizar la localización** — verificar si sigma=0.35 + histéresis alcanza. Si no: (a) generar un mapa mejor (manejar a mano cubriendo todo, guardar), (b) considerar inyección de partículas para recuperar, (c) revisar si conviene confiar más en el odom (que es decente, err~0.2 m) y usar el scan solo para refinar. Medir con `/odom` GT (con nudge).
2. **Verificar end-to-end completo**: llega a la posición y al ángulo final, re-planea con nuevo goal, y probar en `custom_casa_obs` (obstáculos).
3. **Evasión de obstáculos** no mapeados — está implementada (freno + registrar scan en costmap + re-planear) pero sin verificar.
4. **Tunear** velocidades / lookahead / tolerancias con el display real.
5. Documentar para la defensa.

## Archivos tocados esta sesión (además de crear maze_nav)

- `shs/kill_all.sh` (agregados nodos de nav)
- `maps/casa_slam.{pgm,yaml}` (generado)
- `src/maze_slam/launch/slam_casa.launch.py` (fix previo `sensor_yaw=0`, ya commiteado en `badfcfc`)
