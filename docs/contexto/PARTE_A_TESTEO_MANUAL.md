# Parte A — Cómo testear `maze_slam` a mano

Implementación viva: `tpfinal/src/maze_slam/`.
Resultado del smoke test inicial: `tpfinal/results/parte_a/casa_map_smoke.{pgm,yaml}`.

---

## 0. Setup (una sola vez)

Todo corre dentro del container ROS 2 Humble.
Si no está corriendo:

```bash
cd ~/raid5/udesa/robotica
./run.sh                 # entra al container
```

Y dentro del container:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select maze_slam turtlebot3_custom_simulation --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## 1. Etapa 1 — Mapeo con pose conocida (gate de viabilidad)

Esto **no** usa partículas. Sirve para confirmar que el ray casting + log-odds
generan un mapa decente del entorno usando `/calc_odom` como pose conocida.
Si esta etapa ya sale mal, la 2 también va a salir mal.

**Terminal A — simulación + RViz:**

```bash
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
```

(En WSL, si `gzclient` falla, no importa: `gzserver` sigue publicando.)

**Terminal B — mapeo:**

```bash
ros2 launch maze_slam mapper.launch.py
```

RViz se abre con la config de `maze_slam`. Display ya configurados:
`Map (/map)`, `LaserScan (/scan)`, `TF`.

**Terminal C — teleop:**

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Qué mirar:**
1. El robot se mueve y empiezan a aparecer celdas blancas (libres) y negras (paredes) en RViz.
2. Las paredes coinciden con lo que se ve en Gazebo.
3. Después de recorrer un loop, el mapa "cierra".

**Criterio de pasa/no pasa de Etapa 1:**
- Mapa **reconocible** de la casa.
- Las paredes son **rectas** (no se duplican por giros).
- Las zonas libres son **continuas** (sin "manchas" en la mitad del piso).

Si las paredes salen torcidas o duplicadas, hay problema de frames o de cuándo
se actualiza la pose vs el scan — no avanzar a Etapa 2 hasta arreglarlo.

**Guardar el mapa para inspeccionar:**

```bash
ros2 run nav2_map_server map_saver_cli -f ~/casa_mapper
# genera casa_mapper.pgm + casa_mapper.yaml
```

---

## 2. Etapa 2 — Grid-Based FastSLAM con partículas

Acá sí corre el algoritmo completo: N partículas, cada una con pose y mapa
propio, pesado por likelihood field y resampleo.

**Mismo Terminal A:** simulación.

**Terminal B — SLAM:**

```bash
ros2 launch maze_slam fastslam.launch.py
```

RViz muestra ahora también:
- `/particles` (PoseArray verde): nube de hipótesis.
- `/belief` (flecha azul): pose de la mejor partícula.
- `/belief_path` (azul): trayectoria estimada por SLAM.
- `/real_path` (amarillo): trayectoria real de `/odom` (debug, no se usa en el algoritmo).

**Terminal C — manejar:**

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Qué mirar mientras manejás:**

1. **Predicción.** Cuando te movés, la nube de partículas se "abre" (la
   incertidumbre crece). Eso es esperado.
2. **Corrección.** Cuando hay obstáculos cercanos, las partículas se
   "concentran" sobre la mejor hipótesis (resampleo).
3. **`belief_path` vs `real_path`.** El amarillo es ground truth (no lo ve el
   algoritmo), el azul es lo que SLAM estima. **Idealmente** se mantienen cerca.
4. **`/calc_odom`** (no se publica como path por defecto, pero podés verlo con
   `ros2 topic echo`) debería ir **derivando** sin SLAM. El belief tiene que
   estar más cerca del real_path que del calc_odom puro.

**Log del nodo** (Terminal B):

```text
scans=42 updates=42 n_eff=12.3/20 belief=(+1.10,-0.42,+45°) conocidas=8120
```

- `n_eff < N/2` dispara resampling.
- `conocidas` es cuántas celdas del mapa de la mejor partícula tienen
  información (no -1).

---

## 3. Tuneo (parámetros del launch)

Todos los parámetros del nodo son `ros2 param`. Para probar otros valores:

```bash
ros2 launch maze_slam fastslam.launch.py
# o editar inline:
ros2 param set /grid_fastslam n_particles 40
ros2 param set /grid_fastslam sigma_hit 0.15
```

| Parámetro | Default | Qué hace |
|---|---|---|
| `n_particles` | 30 | Más = mejor estimación, peor performance |
| `map_size_m` | 16.0 | Lado del cuadrado en metros |
| `resolution` | 0.05 | m/celda |
| `max_range` | 3.5 | LIDAR max range usado |
| `beam_step` | 4 | Submuestreo de beams para pesar (1=todos) |
| `sigma_hit` | 0.07 | σ del likelihood field (m). Más chico = más estricto |
| `alpha1..4` | 0.3/0.05/0.2/0.05 | Ruido motion model (tuneado en sweep) |
| `min_d_trans` | 0.05 | m mínimos entre updates |
| `min_d_rot` | 0.05 | rad mínimos entre updates |

---

## 4. Guardar el resultado

Cuando el mapa te conforme:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/raid5/udesa/robotica/tpfinal/maps/sim/casa
```

Eso genera `casa.pgm` + `casa.yaml`, listos para Parte B (navegación).

---

## 5. Comparar contra ground truth

Para ver cuánto se "va" el SLAM vs odometría pura:

```bash
ros2 topic echo /odom        --once  # ground truth
ros2 topic echo /calc_odom   --once  # odometría ruidosa (input del SLAM)
ros2 topic echo /belief      --once  # estimación del SLAM
```

Lo deseado:
```
|belief - odom| < |calc_odom - odom|
```
Si no se cumple después de un rato de recorrido, el SLAM no está corrigiendo
nada — revisar `sigma_hit`, `n_particles`, y que `n_eff` baje al menos algunas
veces para que haya resampling.

---

## 6. Tests rápidos sin Gazebo

Para chequear el ray casting / log-odds sin levantar simulación:

```bash
cd ~/ros2_ws/src/robotica/tpfinal/src/maze_slam
python3 -c "
import sys; sys.path.insert(0, '.')
from maze_slam.utils import bresenham_line, update_map_from_scan, logodds_to_occupancy
import numpy as np
H = W = 200; res = 0.05; ox = oy = -5.0
L = np.zeros((H, W), dtype=np.float32)
n = 360
ang = np.linspace(-np.pi, np.pi, n, endpoint=False)
ranges = np.full(n, 2.0)
update_map_from_scan(L, 0.0, 0.0, 0.0, ranges, ang, 3.5, ox, oy, res)
print('libre:', (logodds_to_occupancy(L) < 50).sum(),
      'ocupado:', (logodds_to_occupancy(L) > 50).sum())
"
```

Debería imprimir algo como `libre: 5000+ ocupado: 200+`.

---

## 6.5 Cómo leer los stats del log

Después del tuneo (sweep de 4 configs, ver `tpfinal/results/parte_a/`),
los defaults son `N=30, sigma_hit=0.07, αs=(0.3, 0.05, 0.2, 0.05)`. Con eso
el algoritmo es un SLAM real, no mapeo con odometría ruidosa.

El log se ve así:
```text
scans=89 resamp=88 n_eff_pre=1.0/30 spread_pre=2.8cm
belief=(-1.76,-0.48,-129°) odom=(-1.19,+0.30) occ=545 free=16457
```

| Campo | Qué significa |
|---|---|
| `n_eff_pre` | Effective sample size **antes** de resamplear. Si baja a 1-5 = discriminación fuerte (lo bueno). Si queda en N = pesos uniformes (no está haciendo SLAM). |
| `spread_pre` | Std de las posiciones de las partículas **antes** de resamplear, en cm. Mostraría el ancho del "abanico" de hipótesis. Típico 2-5 cm en run sano. |
| `resamp / scans` | Si ratio ≈ 1.0, resamplea casi cada scan (correcto cuando n_eff_pre es bajo). |
| `belief vs odom` | Diferencia entre la pose estimada por SLAM y `/odom` (ground truth). Idealmente, sobre un recorrido largo, el belief no debería derivar tanto como `/calc_odom`. |

Si `n_eff_pre` se queda en N (no resamplea):
- Subir `alpha1` y `alpha3` (más spread).
- Bajar `sigma_hit` (más estricto).

---

## 7. Problemas conocidos / qué mirar si algo falla

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| Mapa vacío (todo gris) | Nadie publica en `/calc_odom` o `/scan` | `ros2 topic hz /scan` y `/calc_odom` |
| Paredes duplicadas | Drift acumulado de `/calc_odom` sin corrección | Subir `n_particles`, bajar `sigma_hit` |
| Mapa muy chico/cortado | `map_size_m` corto, o robot saliendo del mapa | Subir a 20-24 m |
| Performance baja (`updates` no avanza) | Demasiados beams o partículas | Subir `beam_step` a 8, bajar a `n_particles=10` |
| Belief se "salta" de golpe | Resampleo agresivo | Subir `sigma_hit` para suavizar pesos |

---

## 8. Estado actual

- ✅ Etapa 0: scaffolding (`tpfinal/src/maze_slam`).
- ✅ Etapa 1: `occupancy_mapper` funcional, `/map` publicado en TRANSIENT_LOCAL.
- ✅ Etapa 2: `grid_fastslam` funcional end-to-end con resampling activo.
  Mapa tuneado en `results/parte_a/casa_map_tuned.pgm` (538 occ + 13603 free,
  bbox 8.45 × 7.85 m). `n_eff_pre` cae a 1-5/30, spread real ~3 cm, resample
  cada scan → SLAM discrimina hipótesis.
- ✅ Tuning hecho vía sweep de 4 configs (ver §6.5). Defaults: N=30,
  sigma_hit=0.07, αs=(0.3, 0.05, 0.2, 0.05).
- ⏳ Etapa 4 (opcional): no se necesitó GPU (N=30 corre en CPU sin sufrir).
- ⏳ Etapa 5: falta recorrido largo + evidencia "linda" para informe.
