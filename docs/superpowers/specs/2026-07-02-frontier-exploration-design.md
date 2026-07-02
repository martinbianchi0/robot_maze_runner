# Diseño: Exploración por fronteras para laberinto desconocido

> Estado: diseño aprobado (brainstorming). Fecha: 2026-07-02.
> Rama: `feat/parte-c-robot-real`.

## Problema

La navegación de búsqueda (Parte B/C) está hardcodeada a recorrer una lista fija de
waypoints (`search_waypoints.py` + `config/parte_c/waypoints_*.yaml`), cargada en el
estado `SEARCH_CONE` de `mission_node`. Eso asume que el laberinto es idéntico al del
rosbag pre-grabado, lo cual es falso: el laberinto real que dé la cátedra puede ser
desconocido. Los waypoints fijos no cubren un mapa que no se conoce de antemano.

La consigna de Parte C pide explícitamente que el robot **explore** el laberinto
(`docs/consignas/FLUJO_TP_FINAL.md`, §10.1/§10.3). Este diseño reemplaza los waypoints
fijos por **exploración autónoma por fronteras** sobre un mapa construido online por SLAM.

## Fundamento teórico

Derivado de tres teóricas (`TP Final/Teorica/`):

- **Fronteras** (`20-exploracion_multirobot`, p.4): objetivo = borde entre espacio libre
  y desconocido. Con 1 robot, la asignación multi-robot (húngaro, mercado, descuento por
  visibilidad `U←U·(1−P(d))`) NO aplica; `argmax[U−V]` colapsa a elegir la mejor frontera.
- **Utilidad = información − costo** (`21-exploracion_basada_en_informacion`, p.11, p.26):
  regla de decisión `a* = argmax_a [ I(X,M;Zᵃ) − cost(a) ]`. Aproximamos `I` (info-gain,
  que rigurosamente requiere ray-casting por partícula, p.21-25, DESCARTADO por costo) con
  el tamaño del cluster de frontera. Entropía de la occupancy grid como métrica de progreso
  opcional: `H(M) = -Σ[p·log p + (1-p)·log(1-p)]` (p.18).
- **Control** (`22-control_de_robots`, p.28/35/40): las leyes go-to-point / pure-pursuit /
  go-to-pose ya están implementadas en el `navigator` de toma-2; el explorador NO hace
  control, solo emite goals. Ninguna teórica cubre evasión reactiva (campos potenciales /
  VFH / DWA): se delega al planner, como hoy.

## Alcance

Cubre:
1. **Plomería mínima del SLAM**: `fastslam_node` como fuente de `/map` vivo y pose por TF,
   reemplazando `map_publisher` (mapa estático) + `localizer` (MCL contra mapa estático).
2. **Explorador de fronteras**: módulo puro `frontier.py` + integración en la FSM.
3. **Política de "desconocido"** en `occupancy.py`.

Fuera de alcance (validación propia): porteo +90° del scan real y corrida en el robot
físico (C6); info-gain riguroso con ray-casting; toda la coordinación multi-robot.

## Arquitectura

### Cambio de fuentes (plomería)

- Se levanta **`fastslam_node`** en lugar de `map_publisher` + `localizer`. Ya publica
  `/map` (`OccupancyGrid` que crece) y la TF `map→odom` en vivo.
- La **pose** de la misión pasa de `/amcl_pose` a leerse de la TF `map→base_link`
  (corrección del SLAM ∘ odometría). Cambia la fuente del callback de pose, no la lógica.
- El **`navigator`** de toma-2 queda igual (`/map` + `/goal_pose` → `/cmd_vel` +
  `/nav_state`), pero su suscripción a `/map` debe ser dinámica (QoS no-latched), no
  tomar el mapa una sola vez.
- Aplica el fix documentado del **scan a +90°** del TB4 real (`scripts/scan_reframe.py` o
  `scan_yaw_offset` en el localizer/SLAM), necesario para que el SLAM real localice.

### Flujo de datos (lazo de exploración)

```
fastslam_node ──/map (crece)──┐
              ──TF map→base────┤
                               ▼
                        mission_node (SEARCH_CONE)
                          └─ frontier.select_frontier_goal(/map, pose)
                          └─ valida goal con occupancy.py
                          └─ _emit_goal → /goal_pose
                               ▼
                          navigator → /cmd_vel, /nav_state=REACHED
                               ▲
        cono en /detections ──► preempta a CONE_DETECTED
```

El explorador NO toca `/cmd_vel`: emite goals de frontera como hoy emite waypoints. La
misión sigue siendo el **único emisor de `/goal_pose`** (por eso el módulo vive dentro de
`mission_node`, no en un nodo aparte que competiría por el tópico).

## Componentes

### `frontier.py` (módulo puro, sin ROS)

Recibe la grilla como `np.ndarray` + `GridSpec` + pose; devuelve un goal. Reutiliza las
primitivas de `occupancy.py` (`world_to_grid`, `grid_to_world`, `in_bounds`, BFS).

Cuatro pasos internos (funciones separadas, testeables aisladas):

1. **Detección**: celda frontera = **libre** (`0 ≤ occ < lethal`) con ≥1 vecino 8-conexo
   **desconocido** (`occ < 0`). Vectorizado con numpy (shifts + máscaras).
2. **Clustering**: connected-components 8-conexo (`scipy.ndimage.label`). Descartar
   clusters con < `min_frontier_cells` celdas. Punto representativo por cluster: centroide,
   o la celda del cluster más cercana al robot (para que el goal caiga en celda navegable).
3. **Scoring — utilidad = ganancia − α·costo**:
   - **Costo** = longitud de camino real desde la celda del robot al candidato, vía BFS/
     Dijkstra sobre celdas libres del mapa **inflado** (respeta paredes; no euclídeo). El
     BFS se corre UNA vez desde el robot → costo a todos los candidatos en una pasada.
   - **Ganancia** = tamaño del cluster (proxy de info-gain; frentes grandes abren más área).
     Opcional: afinar contando desconocidos en un radio del centroide.
   - `utilidad = ganancia − α·costo`; `argmax`. Candidatos sin camino → descartados.
4. **Terminación**: sin clusters válidos/alcanzables → `None` (mapa cerrado).

Interfaz única que consume la misión:

```python
@dataclass(frozen=True)
class FrontierGoal:
    x: float
    y: float
    yaw: float      # rumbo del robot hacia la frontera (llega "mirando" lo desconocido)
    utility: float
    cost: float
    gain: float

def select_frontier_goal(
    grid, spec, robot_xy,
    *, lethal=50, inflation_cells, min_frontier_cells, alpha,
) -> FrontierGoal | None:
    ...
```

`alpha`, `min_frontier_cells`, `inflation_cells` se declaran como parámetros en
`mission_config.py`.

### Cambio en `occupancy.py` (política de "desconocido")

Hoy `inflate_occupancy` trata desconocido (`occ < 0`) SIEMPRE como obstáculo. Correcto
para validar el goal del cono; impide usar el mapa para exploración de forma flexible. Se
separan los dos usos con un flag, sin cambiar el comportamiento por defecto:

```python
def inflate_occupancy(grid, radius_cells, *, unknown_as_obstacle=True, lethal_threshold=50):
    # default True = comportamiento actual (validación del goal del cono no cambia)
```

Nota de diseño: con fronteras, el goal SIEMPRE cae en celda conocida-libre (el borde), así
que no hace falta "planificar a través de lo desconocido". Vas al borde → el mapa crece →
aparece el borde siguiente. El único cambio real en `occupancy.py` es el flag para no
colapsar los dos criterios. `is_cell_free` ya tiene `allow_unknown`; se mantiene.

### Cambio en `mission_node.py` (estado `SEARCH_CONE`)

Reemplazar el bloque de `WaypointRoute` (líneas ~304-315) por:

```
SEARCH_CONE (tick):
  - si hay cono estable en /detections → CONE_DETECTED          (igual que hoy, preempción)
  - si no hay goal de frontera en curso:
        goal = frontier.select_frontier_goal(raw_grid, spec, pose, ...)
        si goal is None → mapa cerrado sin cono → FAILURE        ("laberinto explorado, sin cono")
        _emit_goal(goal.x, goal.y, goal.yaw); wp_sent = True
  - si hay goal en curso y nav_state == REACHED:
        wp_sent = False   → el próximo tick recalcula fronteras sobre el mapa ya crecido
  - (opcional) giro-scan al llegar, reutilizando la lógica de scan existente
```

Se elimina `search_waypoints.py` y `_load_route()`; el parámetro `waypoints_file` queda
deprecado. El resto de la FSM (CONE_DETECTED, ESTIMATE_CONE_GOAL, PLAN_TO_CONE, NAVIGATE,
REPLAN, VERIFY, DONE) queda **intacto**: solo cambia cómo se generan los goals de búsqueda.

## Manejo de errores / casos borde

- **Sin mapa todavía** (SLAM arrancando): `raw_grid is None` → esperar (ya hay gracia
  inicial de 1.5 s).
- **Frontera inalcanzable** (BFS no llega): se descarta en el scoring; si caen todas,
  `None` → FAILURE.
- **Goal rechazado por validación** (cae sobre inflado): tomar el siguiente mejor candidato
  del ranking; si no queda ninguno, recalcular al próximo tick.
- **Navigator en RECOVERY / IDLE sin REACHED**: mismo manejo que hoy (la FSM detecta
  PLANNING→IDLE como fallo de goal) → recalcular frontera.
- **Robot atascado** (mismo frontier goal N veces sin avanzar): contador de reintentos por
  frontera; al agotarse, blacklist temporal de esa celda y siguiente candidato.

## Testing

### Unit tests (Python puro, `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`)

Sobre `frontier.py` con grillas sintéticas:

- **Detección**: borde libre↔desconocido conocido → marca exactamente esas celdas;
  grilla toda-conocida → cero fronteras.
- **Clustering**: dos frentes separados → dos clusters; frente de 1-2 celdas con
  `min_frontier_cells=5` → descartado.
- **Costo (BFS)**: laberinto en L → costo de camino a frontera detrás de pared > euclídeo;
  frontera amurallada sin camino → inalcanzable (descartada).
- **Scoring/argmax**: frontera grande-lejana vs chica-cercana → `alpha` inclina la
  elección como se espera (barrido de `alpha`).
- **Terminación**: mapa sin desconocidos → `None`.
- **Conversión grid↔world**: goal en mundo cae dentro de la celda del centroide
  (round-trip con `resolution`/`origin`).

### Integración en Gazebo (lazo cerrado)

Smoke tipo `scripts/smoke_*.sh`: Gazebo (world de laberinto) + `fastslam_node` +
`navigator` + `mission_node`, sin cono. Éxito: el área conocida de `/map` crece
monótonamente, el robot emite goals de frontera distintos, y termina en FAILURE con nota
"laberinto explorado sin cono" (o DONE si el world tiene cono). Capturas de RViz (mapa +
`/goal_pose` + `/plan`) para el informe. Corre en `rosenv` con el workaround sandbox-conda
documentado en el entorno de Parte C.

## Resumen de decisiones

- **Alcance**: explorador de fronteras + plomería mínima (fastslam_node → `/map` vivo +
  pose por TF).
- **Selección**: utilidad = ganancia (tamaño de cluster) − α·costo (BFS sobre inflado),
  argmax.
- **Ubicación**: módulo puro `frontier.py` llamado desde `SEARCH_CONE`; la misión sigue
  siendo el único emisor de `/goal_pose`.
- **`occupancy.py`**: flag `unknown_as_obstacle` para separar validación-del-cono de
  costo-de-exploración.
- **Validación**: unit tests con grillas sintéticas + smoke en Gazebo en lazo cerrado.
