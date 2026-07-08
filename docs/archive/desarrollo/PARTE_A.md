# Parte A — SLAM (Grid-Based FastSLAM)

## Qué hace

Construye un **mapa de ocupación** de la casa mientras el robot la explora, estimando en paralelo la pose del robot dentro de ese mapa.
Salida final: `maps/casa_slam.pgm` + `maps/casa_slam.yaml` — el mapa que consume la Parte B.

## Algoritmo

**Grid-Based FastSLAM** (Opción 1 de la consigna).
Filtro de partículas donde cada partícula lleva:

- Su propia pose `(x, y, θ)`.
- Su propio **occupancy grid** (log-odds) actualizado con el LIDAR desde esa pose.

Ciclo por scan:

1. **Predicción** — muestreo del modelo de movimiento por odometría (`/calc_odom`, la ruidosa; el `/odom` de Gazebo es GT y sería trampa).
2. **Scan-match local** — refina la pose de cada partícula alrededor de la predicción para maximizar el match con su mapa. Limitado en delta para evitar los saltos de 90° en salas simétricas (ver commit `ce8f426`).
3. **Peso** — likelihood del scan bajo el mapa de la partícula.
4. **Actualización del mapa** — inverse sensor model (log-odds) con los rayos del scan.
5. **Resample** por peso cuando Neff cae.

Al terminar, la mejor partícula se serializa como `.pgm` + `.yaml` de `map_server`.

## Estructura

- `src/maze_slam/maze_slam/fastslam.py` — el algoritmo (sin ROS, testeable aislado).
- `src/maze_slam/maze_slam/fastslam_node.py` — envoltorio ROS (suscribe `/scan` + `/calc_odom`, publica `/map` y TF `map→calc_odom`, expone `/maze_slam/save_request` para volcar el mapa a disco).
- `src/maze_slam/maze_slam/_kernels.py` — kernels vectorizados para el scan-match y la actualización de la grilla.
- `src/maze_slam/launch/slam_casa.launch.py` — sube el nodo con los params tuneados para la casa (sensor_yaw=0, ver commit `badfcfc`).

## Cómo correrlo

Tres terminales, todas dentro del container:

```
T1: ./shs/casa.sh              # Gazebo con la casa (TB3 burger)
T2: ./shs/slam_casa.sh         # nuestro SLAM + RViz con overlay scan/mapa
T3: ./shs/teleop.sh            # teclas w/a/s/d/x
```

Manejarlo por toda la casa (esquinas, cerrar loops).
Cuando el mapa se ve completo en RViz:

```
T4: ./shs/save_map.sh          # graba maps/casa_slam.{pgm,yaml}
```

Cortar con `Ctrl+C` los tres launchs y correr `./shs/kill_all.sh` (Gazebo y `robot_state_publisher` no mueren solos y contaminan `/calc_odom` en la próxima corrida).

## Gotchas descubiertos

- **Zombies de sim** — sin `kill_all.sh` quedan procesos vivos publicando `/calc_odom` duplicado; el SLAM recibe odometría contradictoria y el mapa sale roto.
- **Scan-match sin límite** produce saltos de 90° en salas cuadradas (aliasing rotacional del laberinto). Fix: capar el delta del refine (commit `ce8f426`).
- **`sensor_yaw` default del TB4** — heredaba 90° en el TB3; fix en `slam_casa.launch.py` (commit `badfcfc`).
- **Mapa de la cátedra mal registrado** — `worlds/map/map.yaml` no alinea con la sim; hay que usar nuestro `casa_slam.yaml`.
