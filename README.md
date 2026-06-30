# TP Final I-402 — Parte A: Grid-Based FastSLAM

SLAM propio (Opción 1 de la consigna) sobre el **rosbag del laberinto físico**
grabado con el TurtleBot4 en el lab (`maps/laberinto.zip`, 23 min, LIDAR + odom).

Implementa Grid-Based FastSLAM: cada partícula mantiene su propia pose y su propio
mapa de ocupación en log-odds, y se ponderan con un *likelihood field* construido
sobre ese mismo mapa.

## Entorno

- ROS 2 (Jazzy en el host de dev, Humble en el container del lab — los scripts
  autodetectan la distro).
- numpy 1.26 + scipy del sistema. **No instalar numpy>=2 en `~/.local`** (rompe
  scipy por ABI).
- **Numba (opcional pero recomendado):** acelera el SLAM ~45x (851ms -> 45ms por
  update), permitiendo la config de maxima calidad en vivo. Instalar con:
  ```bash
  pip install --user numba    # en el container: pip3 install numba
  ```
  Si numba no esta, el codigo cae a Python puro (anda igual, mas lento -> bajá
  `--rate` del bag).

## Cómo correrlo

Cada script en su propia terminal:

```bash
# 1) build (una vez, o tras cambios)
./shs/build.sh           # incremental
./shs/build.sh --clean   # forzar clean rebuild

# 2) en tres terminales:
./shs/bag.sh             # reproduce el rosbag del laberinto
./shs/slam.sh            # nodo de FastSLAM
./shs/rviz.sh            # visualización

# 3) cuando el mapa quedó lindo:
./shs/save_map.sh        # escribe maps/casa_slam.{pgm,yaml}
```

Iterá más rápido con `./shs/bag.sh --rate 2.0` o `--loop`.
Si algo queda colgado: `./shs/kill_all.sh`.

## Topics

El nodo publica:

| Topic                       | Tipo                          | Para qué                          |
|-----------------------------|-------------------------------|-----------------------------------|
| `/map`                      | `nav_msgs/OccupancyGrid`      | Mapa de la mejor partícula        |
| `/belief`                   | `geometry_msgs/PoseStamped`   | Pose corregida (mejor partícula)  |
| `/maze_slam/particles`      | `geometry_msgs/PoseArray`     | Todas las partículas, debug       |
| TF `map → tb4_0/odom`       | -                             | Corrección que aplica el SLAM     |

Y consume del bag:

| Topic           | Tipo                       |
|-----------------|----------------------------|
| `/tb4_0/scan`   | `sensor_msgs/LaserScan`    |
| `/tb4_0/odom`   | `nav_msgs/Odometry`        |
| `/tf` `/tf_static` (remap del bag) | TF de TB4         |

## Parámetros (en `src/maze_slam/launch/slam.launch.py`)

- `n_particles` (15) — más partículas = más fidelidad, más CPU.
- `map_size` (300) — celdas por lado. 300 × 0.05 m = 15 × 15 m.
- `resolution` (0.05) — metros por celda.
- `publish_rate` (4.0) — Hz de publicación de `/map` y TF.
- `odom_frame` (`tb4_0/odom`) — frame padre de la TF que publica el SLAM.

## Estructura

```
tpfinal/
├── docs/consignas/        ← PDFs de la cátedra
├── maps/
│   ├── laberinto.zip      ← rosbag original
│   ├── laberinto/         ← extraído por bag.sh la primera vez
│   └── casa_slam.{pgm,yaml}  ← entregable Parte A (después de save_map)
├── src/
│   ├── maze_slam/                       ← nuestro paquete (SLAM)
│   └── turtlebot3_custom_simulation/    ← paquete de la cátedra (se buildea pero
│                                          no se usa en Parte A; queda para B)
└── shs/                   ← scripts
```
