# Parte B — Navegación autónoma

## Qué hace

Con el mapa de la Parte A (`maps/casa_slam.yaml`), el robot:

1. Se localiza dentro del mapa desde una pose inicial aproximada.
2. Planifica un camino al objetivo que le indica el usuario.
3. Sigue ese camino y llega a la posición **y** al ángulo pedidos.
4. Re-planifica si se le da un nuevo objetivo (aunque esté a mitad de camino).
5. Esquiva obstáculos que no estaban en el mapa.
6. Todo orquestado por una máquina de estados.

Cubre los ítems 1.1 a 1.11 de la consigna.

## Arquitectura

Tres nodos ROS 2 desacoplados en el paquete `src/maze_nav/`:

| Nodo | Rol | Tópicos que consume | Tópicos que publica |
|---|---|---|---|
| `map_publisher` | Publica el mapa estático de Parte A. | — | `/map` (latched) |
| `localizer` | MCL (filtro de partículas). | `/map`, `/calc_odom`, `/scan`, `/initialpose` | `/amcl_pose`, `/particlecloud`, TF `map→calc_odom` |
| `navigator` | A\* + pure-pursuit + FSM. | `/map`, `/amcl_pose`, `/goal_pose`, `/scan` | `/cmd_vel`, `/plan`, `/nav_state` |

Utilities compartidas en `maze_nav/nav_utils.py` (conversiones grilla↔mundo, modelo de movimiento, wrapping de ángulos).

## Algoritmos

### Localización — MCL con Augmented resampling

`maze_nav/localizer.py`.

- **Predicción**: modelo de movimiento por odometría sobre `/calc_odom` (la ruidosa; nunca `/odom` GT).
- **Corrección**: **likelihood field** (endpoint model) sobre el mapa. Modelo de mezcla `0.8·gaussiana + 0.2·uniforme` — el término uniforme acota la penalización por rayo y evita el colapso a hipótesis simétricas.
- **Offset del LIDAR**: el frame `base_scan` está a `-3.2 cm` en X respecto de `base_footprint` (TB3 burger). Compensado al proyectar los endpoints; ignorarlo shifteaba el scan ~0.6 celdas y el filtro nunca alineaba.
- **Resample**: sistemático + roughening (jitter chico); se dispara sólo si `Neff < N/5` (menos agresivo que el `N/3` clásico).
- **Augmented MCL** (Probabilistic Robotics 8.3): traqueo de `w_slow` y `w_fast`; cuando `w_fast/w_slow < 0.95` se inyectan hasta 5 % de partículas en celdas libres al azar → recuperación automática si el filtro se pierde.
- **Reporte por moda**: la estimación publicada es la media pesada del **top 20 %** de partículas por peso, no la media pesada global. Robusto contra distribuciones multimodales en yaw.
- **Smoothing de yaw**: máximo 0.35 rad de salto entre publicaciones consecutivas — el controlador no aguanta thrashing.
- **TF `map→calc_odom`**: publicado por el localizer (corrige la deriva de `/calc_odom` con la pose del filtro).

### Planning — A\* en grilla inflada

`maze_nav/navigator.py::plan`.

- **Costmap** = `distance_transform_edt` sobre `paredes ∪ celdas desconocidas ∪ obstáculos dinámicos`. Se marca **blocked** todo lo que esté a menos de `robot_radius + inflation` de un obstáculo.
- **A\*** 8-conectado, heurística Euclidiana. Coste del arco = paso + penalización por proximidad a obstáculos → empuja el camino al centro del pasillo.
- Si el `start` o el `goal` cae en una celda bloqueada (posible por error de MCL), se busca la celda libre más cercana.

### Control — Pure Pursuit

`maze_nav/navigator.py::_follow`.

- Índice `path_idx` monótono: en cada tick busca el punto más cercano al robot **hacia adelante** en el path, avanza el índice, y desde ahí toma el primer punto a `lookahead ≥ 0.35 m`.
- Esto evita el bug del "orbit": si el robot se desviaba lateralmente por MCL flojo, la búsqueda naive lo mandaba al `path[0]` (detrás) y giraba 180°.
- Si `|error_angular| > 0.8 rad` → sólo rota; si no → rota + avanza (velocidad lineal proporcional a `1 - |err|`).
- Alineación final con **piso de 0.3 rad/s** para no atascarse en el ruido residual de MCL.

### Detección y evasión de obstáculos no mapeados

`maze_nav/navigator.py::_register_obstacle_from_scan`.

- **Freno de emergencia**: si `_forward_clearance < safety_stop (0.22 m)` en el arco frontal (±30°) → intenta registrar el hit.
- Sólo cuenta como **obstáculo nuevo** un hit del LIDAR que caiga en una celda marcada como *libre* en el mapa estático **Y** a más de 2 celdas de cualquier pared conocida (filtro anti-error-MCL — sin esto, el error residual de localización se registraba como obstáculo fantasma).
- Si hubo hits nuevos → los agrega al costmap, re-infla, transita a `RECOVERY`, re-planifica.
- Si el freno se disparó pero no hay obstáculos nuevos → es una pared ya mapeada + error de MCL: baja la velocidad pero no entra en `RECOVERY` (evita el loop `FOLLOWING↔RECOVERY`).

### FSM

```
IDLE ── /goal_pose ─► PLANNING ── plan OK ─► FOLLOWING ── |dist| < tol ─► ALIGNING ── |yaw err| < tol ─► REACHED
                                                       │
                                                       └─ obstáculo nuevo ─► RECOVERY ── re-plan OK ─► FOLLOWING
```

Publicado en `/nav_state` (útil para debug).

## Cómo correrlo

Dos terminales, dentro del container:

```
T1: ./shs/casa.sh              # casa vacía
    ./shs/casa.sh obs          # casa con obstáculos no mapeados
    ./shs/casa.sh obs2         # casa con rutas cerradas (opcional)
T2: ./shs/nav.sh               # stack de nav + RViz
```

En RViz:

1. **2D Pose Estimate** → click + drag en la pose real del robot (a ojo, ±30 cm y ±15° está bien; MCL refina solo).
2. **2D Goal Pose** → click + drag en el destino. El drag define el yaw final.
3. Robot planifica → sigue → alinea → `Objetivo alcanzado`.
4. Podés tirar otro goal en cualquier momento (incluso a mitad de camino): re-planifica desde donde esté.

## Cómo probar cada requisito de la consigna

| Consigna | Cómo se prueba |
|---|---|
| 1.1 Localización | El filtro converge al fijar `2D Pose Estimate`. |
| 1.2 Init pose | Herramienta `2D Pose Estimate` → `/initialpose`. |
| 1.3 Objetivo | Herramienta `2D Goal Pose` → `/goal_pose`. |
| 1.4 Localización on-the-move | La nube de partículas trackea al robot mientras se mueve. |
| 1.5 Planning | Ver la línea rosa (`/plan`) en RViz — no toca paredes. |
| 1.6 Path following | El robot sigue la línea rosa suavemente. |
| 1.7 Ángulo final | El drag del goal define el yaw; el robot gira in-place al llegar. |
| 1.8 Repeticiones | Tirar un segundo goal, sea después de llegar o durante el camino. |
| 1.9 Obstáculos | `./shs/casa.sh obs` — el robot detecta, agrega al costmap, re-planifica. |
| 1.11 FSM | `ros2 topic echo /nav_state` para ver transiciones. |

## Parámetros clave (todos en `nav.launch.py` / declare_parameter)

**Localizer**:

- `n_particles=400`, `sigma_hit=0.35 m`, `max_beams=60`.
- `alpha1..4=0.05` (ruido del motion model).
- `resample_neff_ratio=0.20`.
- `alpha_slow=0.001`, `alpha_fast=0.1` (Augmented MCL).
- `init_xy_std=0.30 m`, `init_yaw_std=0.30 rad`.
- `scan_x_offset=-0.032 m` (TB3 burger).

**Navigator**:

- `robot_radius=0.14 m`, `inflation=0.12 m`.
- `lookahead=0.35 m`, `v_max=0.18 m/s`, `w_max=1.2 rad/s`.
- `goal_tol=0.12 m`, `yaw_tol=0.15 rad`.
- `safety_stop=0.22 m`, `control_rate=20 Hz`.

## Gotchas descubiertos (no volver a tropezar)

1. **Offset del LIDAR** — `base_scan` a `-3.2 cm` de `base_footprint`. Ignorarlo shiftea el scan 0.6 celdas y el MCL nunca alinea.
2. **Bug del orbit en pure-pursuit** — sin `path_idx` monótono, el algoritmo naive elige `path[0]` cuando el robot se desvía lateralmente → gira 180° y va marcha atrás.
3. **Loop FOLLOWING↔RECOVERY** — sin filtrar los hits del LIDAR contra el mapa, cualquier pared ya conocida disparaba freno + backup.
4. **Yaw thrashing** — sin smoothing de yaw y sin reporte por moda, distribuciones multimodales del filtro pasaban al controlador y el robot orbita en el lugar.
5. **Alineación final infinita** — sin piso de velocidad angular, el proporcional cae bajo el ruido de MCL y no converge.
6. **Zombies** — `python3 /tmp/xxx.py` (scripts de test) NO los mata `kill_all.sh`; procesos viejos publicando `/cmd_vel` fantasma. Antes de cada test: `pkill -9 -f "python3 /tmp/"`.
7. **`setup.cfg` obligatorio** en paquetes ament_python — sin él los ejecutables no se instalan en `lib/maze_nav/` y `ros2 launch` falla con "libexec directory does not exist".
8. **Typo del launch de obstáculos** — `custom_casa_obs.launch.py` apuntaba a `casa_obs.world`, el archivo real es `casa_o.world` (corregido en `9b7bea4`).
