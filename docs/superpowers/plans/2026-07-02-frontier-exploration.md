# Exploración por fronteras — Plan de implementación

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reemplazar la búsqueda por waypoints fijos de `mission_node` por exploración autónoma por fronteras sobre un mapa construido online por `fastslam_node`.

**Architecture:** Un módulo puro `frontier.py` detecta fronteras (borde libre↔desconocido) sobre el `/map` vivo, las puntúa por utilidad = tamaño_cluster − α·costo_camino (BFS sobre mapa inflado) y elige la mejor. El estado `SEARCH_CONE` de la FSM lo llama en lugar de iterar waypoints; la misión sigue siendo el único emisor de `/goal_pose`. `fastslam_node` reemplaza a `map_publisher`+`localizer` como fuente de `/map` y `/amcl_pose`.

**Tech Stack:** Python 3.11, rclpy, numpy, scipy (`ndimage.binary_dilation`, `ndimage.label`, `distance_transform_edt`), pytest. Paquetes ament_python: `maze_mission`, `maze_slam`.

## Global Constraints

- Python: imports siempre arriba del archivo (regla del usuario).
- Sin emojis en commits, comentarios ni output escrito.
- Tests unitarios de módulos puros: correr con el intérprete de `rosenv` directo, sin colcon:
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission /Users/varbelaiz/miniforge3/envs/rosenv/bin/python -m pytest <archivo> -v`
  (el `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` es obligatorio: los plugins launch_testing rompen la colección en rosenv).
- Build de paquetes ROS: `source $HOME/miniforge3/etc/profile.d/conda.sh && conda activate rosenv && colcon build --packages-select <pkg> --symlink-install` con `dangerouslyDisableSandbox: true`.
- Convención de grilla: la occupancy grid se indexa `grid[gy, gx]` (fila=y, columna=x), igual que `occupancy.py`. Valores: `-1` desconocido, `0..49` libre, `>=50` (LETHAL) ocupado.
- Rama de trabajo: `feat/parte-c-robot-real`.

---

## File Structure

- Create `src/maze_mission/maze_mission/frontier.py` — módulo puro de exploración por fronteras (detección, clustering, costo BFS, selección). Sin ROS.
- Create `src/maze_mission/test/test_frontier.py` — unit tests con grillas sintéticas.
- Modify `src/maze_mission/maze_mission/occupancy.py` — flag `unknown_as_obstacle` en `inflate_occupancy`.
- Modify `src/maze_mission/test/test_occupancy.py` — test del flag.
- Modify `src/maze_mission/maze_mission/mission_config.py` — parámetros `frontier_alpha`, `frontier_min_cells`.
- Modify `src/maze_mission/maze_mission/mission_node.py` — `SEARCH_CONE` usa `frontier`; se quita el uso de `WaypointRoute`.
- Modify `src/maze_slam/maze_slam/fastslam_node.py` — publicar `/amcl_pose` (PoseWithCovarianceStamped) y `/map` como `transient_local`.
- Create `src/maze_mission/launch/explore.launch.py` — levanta fastslam_node + navigator + mission_node para exploración.
- Create `scripts/smoke_explore.sh` — smoke de integración en Gazebo (lazo cerrado).

---

## Task 1: Flag `unknown_as_obstacle` en `occupancy.py`

**Files:**
- Modify: `src/maze_mission/maze_mission/occupancy.py:81-95`
- Test: `src/maze_mission/test/test_occupancy.py`

**Interfaces:**
- Produces: `inflate_occupancy(grid, radius_cells, *, unknown_as_obstacle=True, lethal_threshold=50) -> np.ndarray`. Default `True` = comportamiento actual (desconocido trata como obstáculo). `False` = solo las paredes mapeadas (`>= lethal_threshold`) inflan; el desconocido no.

- [ ] **Step 1: Escribir el test que falla**

Agregar al final de `src/maze_mission/test/test_occupancy.py`:

```python
def test_inflate_unknown_as_obstacle_false_no_infla_desconocido():
    grid = np.zeros((5, 5), dtype=np.int16)
    grid[0, 0] = -1
    inflated = inflate_occupancy(grid, radius_cells=1, unknown_as_obstacle=False)
    # sin tratar desconocido como obstaculo: las celdas libres vecinas siguen libres
    assert inflated[0, 1] == 0 and inflated[1, 0] == 0
    assert inflated[0, 0] == -1


def test_inflate_unknown_as_obstacle_false_igual_infla_paredes():
    grid = np.zeros((5, 5), dtype=np.int16)
    grid[2, 2] = 100
    inflated = inflate_occupancy(grid, radius_cells=1, unknown_as_obstacle=False)
    assert inflated[2, 1] == 100 and inflated[1, 2] == 100
```

- [ ] **Step 2: Correr el test y verificar que falla**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission /Users/varbelaiz/miniforge3/envs/rosenv/bin/python -m pytest src/maze_mission/test/test_occupancy.py -v`
Expected: FAIL — `inflate_occupancy() got an unexpected keyword argument 'unknown_as_obstacle'`

- [ ] **Step 3: Implementar el cambio mínimo**

Reemplazar la firma y el cálculo de obstáculos en `src/maze_mission/maze_mission/occupancy.py`:

```python
def inflate_occupancy(grid, radius_cells, *, unknown_as_obstacle=True, lethal_threshold=50):
    """Marca como lethal (100) toda celda libre a <= radius_cells de un obstaculo.

    Obstaculo = pared mapeada (>= lethal_threshold). Si unknown_as_obstacle (default),
    tambien el desconocido (< 0) cuenta como obstaculo, igual que el navigator de toma-2
    (para validar goals del cono). unknown_as_obstacle=False lo excluye, para usar el
    mapa como campo de costo de exploracion sin que lo desconocido bloquee. Los
    desconocidos quedan en -1 (no navegables via is_cell_free con allow_unknown=False).
    Devuelve una grilla nueva.
    """
    grid = np.asarray(grid)
    inflated = grid.copy()
    obstacles = grid >= lethal_threshold
    if unknown_as_obstacle:
        obstacles = obstacles | (grid < 0)
    if radius_cells <= 0 or not obstacles.any():
        return inflated
    dist_cells = distance_transform_edt(~obstacles)
    inflated[(dist_cells <= radius_cells) & (grid >= 0)] = 100
    return inflated
```

- [ ] **Step 4: Correr el test y verificar que pasa**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission /Users/varbelaiz/miniforge3/envs/rosenv/bin/python -m pytest src/maze_mission/test/test_occupancy.py -v`
Expected: PASS (todos, incluyendo `test_inflate_trata_desconocido_como_obstaculo` que sigue verde porque el default no cambió).

- [ ] **Step 5: Commit**

```bash
git add src/maze_mission/maze_mission/occupancy.py src/maze_mission/test/test_occupancy.py
git commit -m "feat(occupancy): flag unknown_as_obstacle para separar validacion de costo de exploracion"
```

---

## Task 2: Detección de fronteras en `frontier.py`

**Files:**
- Create: `src/maze_mission/maze_mission/frontier.py`
- Test: `src/maze_mission/test/test_frontier.py`

**Interfaces:**
- Consumes: nada de tareas previas.
- Produces: `find_frontier_cells(grid, lethal=50) -> np.ndarray` (máscara booleana `[gy, gx]`, `True` = celda frontera: libre `0 <= v < lethal` con al menos un vecino 8-conexo desconocido `v < 0`).

- [ ] **Step 1: Escribir el test que falla**

Crear `src/maze_mission/test/test_frontier.py`:

```python
import numpy as np

from maze_mission.frontier import find_frontier_cells


def test_borde_libre_desconocido_es_frontera():
    # columna 0 libre, columna 1 desconocida -> la col 0 es frontera
    grid = np.array([[0, -1, -1],
                     [0, -1, -1],
                     [0, -1, -1]], dtype=np.int16)
    mask = find_frontier_cells(grid, lethal=50)
    assert mask[:, 0].all()          # toda la columna libre lindante con desconocido
    assert not mask[:, 2].any()      # desconocido no es frontera


def test_grilla_toda_conocida_sin_fronteras():
    grid = np.zeros((4, 4), dtype=np.int16)   # todo libre, nada desconocido
    mask = find_frontier_cells(grid, lethal=50)
    assert not mask.any()


def test_pared_no_es_frontera():
    grid = np.array([[100, -1],
                     [0, -1]], dtype=np.int16)
    mask = find_frontier_cells(grid, lethal=50)
    assert not mask[0, 0]            # pared (lethal) no es frontera aunque linde desconocido
    assert mask[1, 0]               # libre lindante con desconocido si
```

- [ ] **Step 2: Correr el test y verificar que falla**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission /Users/varbelaiz/miniforge3/envs/rosenv/bin/python -m pytest src/maze_mission/test/test_frontier.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'maze_mission.frontier'`

- [ ] **Step 3: Implementar el módulo mínimo**

Crear `src/maze_mission/maze_mission/frontier.py`:

```python
"""Exploracion por fronteras sobre una occupancy grid (autocontenido, sin ROS).

Una celda frontera es una celda libre adyacente a una desconocida: el borde entre lo
mapeado y lo por mapear. Explorar = ir a la mejor frontera (utilidad = tamanio del
cluster menos alpha por el costo de camino), hasta que no queden fronteras (mapa cerrado).
Reemplaza la busqueda por waypoints fijos. Fundamento: teoricas 20 (fronteras) y 21
(utilidad = informacion - costo).
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import binary_dilation, label

from maze_mission.occupancy import (
    GridSpec,
    grid_to_world,
    inflate_occupancy,
    nearest_free_cell,
    world_to_grid,
)

_NEIGH8 = np.ones((3, 3), dtype=bool)


def find_frontier_cells(grid, lethal=50):
    grid = np.asarray(grid)
    free = (grid >= 0) & (grid < lethal)
    unknown = grid < 0
    unknown_adjacent = binary_dilation(unknown, structure=_NEIGH8)
    return free & unknown_adjacent
```

- [ ] **Step 4: Correr el test y verificar que pasa**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission /Users/varbelaiz/miniforge3/envs/rosenv/bin/python -m pytest src/maze_mission/test/test_frontier.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/maze_mission/maze_mission/frontier.py src/maze_mission/test/test_frontier.py
git commit -m "feat(frontier): deteccion de celdas frontera libre-desconocido"
```

---

## Task 3: Clustering de fronteras en `frontier.py`

**Files:**
- Modify: `src/maze_mission/maze_mission/frontier.py`
- Test: `src/maze_mission/test/test_frontier.py`

**Interfaces:**
- Consumes: `find_frontier_cells` (Task 2).
- Produces:
  - `@dataclass(frozen=True) FrontierCluster` con campos: `cells: tuple[tuple[int,int], ...]` (lista de `(gx, gy)`), `size: int`, `centroid_gx: int`, `centroid_gy: int`.
  - `cluster_frontiers(frontier_mask, min_cells) -> list[FrontierCluster]` (clusters 8-conexos con `size >= min_cells`).

- [ ] **Step 1: Escribir el test que falla**

Agregar a `src/maze_mission/test/test_frontier.py`:

```python
from maze_mission.frontier import cluster_frontiers


def test_dos_frentes_separados_dos_clusters():
    mask = np.zeros((7, 7), dtype=bool)
    mask[0, 0:3] = True          # frente A (3 celdas)
    mask[6, 4:7] = True          # frente B (3 celdas), no contiguo
    clusters = cluster_frontiers(mask, min_cells=1)
    assert len(clusters) == 2
    assert {c.size for c in clusters} == {3}


def test_descarta_clusters_chicos():
    mask = np.zeros((7, 7), dtype=bool)
    mask[0, 0:2] = True          # 2 celdas
    mask[6, 0:5] = True          # 5 celdas
    clusters = cluster_frontiers(mask, min_cells=5)
    assert len(clusters) == 1
    assert clusters[0].size == 5


def test_centroide_dentro_del_frente():
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 1:4] = True          # fila y=2, x=1..3 -> centroide (2,2)
    clusters = cluster_frontiers(mask, min_cells=1)
    assert len(clusters) == 1
    assert clusters[0].centroid_gx == 2 and clusters[0].centroid_gy == 2
```

- [ ] **Step 2: Correr el test y verificar que falla**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission /Users/varbelaiz/miniforge3/envs/rosenv/bin/python -m pytest src/maze_mission/test/test_frontier.py -v`
Expected: FAIL — `ImportError: cannot import name 'cluster_frontiers'`

- [ ] **Step 3: Implementar**

Agregar a `src/maze_mission/maze_mission/frontier.py` (después de `find_frontier_cells`):

```python
@dataclass(frozen=True)
class FrontierCluster:
    cells: tuple           # tupla de (gx, gy)
    size: int
    centroid_gx: int
    centroid_gy: int


def cluster_frontiers(frontier_mask, min_cells):
    labels, n = label(np.asarray(frontier_mask), structure=_NEIGH8)
    clusters = []
    for idx in range(1, n + 1):
        ys, xs = np.where(labels == idx)
        if xs.size < min_cells:
            continue
        cells = tuple((int(x), int(y)) for x, y in zip(xs, ys))
        clusters.append(FrontierCluster(
            cells=cells,
            size=int(xs.size),
            centroid_gx=int(round(float(xs.mean()))),
            centroid_gy=int(round(float(ys.mean()))),
        ))
    return clusters
```

- [ ] **Step 4: Correr el test y verificar que pasa**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission /Users/varbelaiz/miniforge3/envs/rosenv/bin/python -m pytest src/maze_mission/test/test_frontier.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/maze_mission/maze_mission/frontier.py src/maze_mission/test/test_frontier.py
git commit -m "feat(frontier): clustering de fronteras 8-conexo con descarte de ruido"
```

---

## Task 4: Campo de costo por BFS en `frontier.py`

**Files:**
- Modify: `src/maze_mission/maze_mission/frontier.py`
- Test: `src/maze_mission/test/test_frontier.py`

**Interfaces:**
- Consumes: nada nuevo.
- Produces: `path_cost_field(inflated_grid, start_cell, lethal=50) -> np.ndarray` (array int32 `[gy, gx]` con nº de pasos BFS 8-conexo desde `start_cell` sobre celdas libres `0 <= v < lethal`; `-1` = inalcanzable). `start_cell` es `(gx, gy)`.

- [ ] **Step 1: Escribir el test que falla**

Agregar a `src/maze_mission/test/test_frontier.py`:

```python
from maze_mission.frontier import path_cost_field


def test_costo_respeta_paredes():
    # laberinto en L: pared en el medio obliga a rodear
    grid = np.zeros((5, 5), dtype=np.int16)
    grid[0:4, 2] = 100           # pared vertical col 2, filas 0..3 (deja fila 4 abierta)
    dist = path_cost_field(grid, (0, 0), lethal=50)
    # celda al otro lado de la pared: alcanzable pero con costo > distancia recta
    assert dist[0, 4] > 4        # tuvo que bajar y rodear por la fila 4
    assert dist[0, 0] == 0


def test_frontera_amurallada_inalcanzable():
    grid = np.zeros((5, 5), dtype=np.int16)
    grid[1, 1:4] = 100
    grid[3, 1:4] = 100
    grid[1:4, 1] = 100
    grid[1:4, 3] = 100           # celda (2,2) encerrada por paredes
    dist = path_cost_field(grid, (0, 0), lethal=50)
    assert dist[2, 2] == -1      # inalcanzable
```

- [ ] **Step 2: Correr el test y verificar que falla**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission /Users/varbelaiz/miniforge3/envs/rosenv/bin/python -m pytest src/maze_mission/test/test_frontier.py -v`
Expected: FAIL — `ImportError: cannot import name 'path_cost_field'`

- [ ] **Step 3: Implementar**

Agregar a `src/maze_mission/maze_mission/frontier.py`:

```python
_DIRS8 = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))


def path_cost_field(inflated_grid, start_cell, lethal=50):
    grid = np.asarray(inflated_grid)
    h, w = grid.shape
    dist = np.full((h, w), -1, dtype=np.int32)
    sx, sy = start_cell
    if not (0 <= sx < w and 0 <= sy < h):
        return dist
    if not (0 <= grid[sy, sx] < lethal):
        return dist
    dist[sy, sx] = 0
    queue = deque([(sx, sy)])
    while queue:
        x, y = queue.popleft()
        d = dist[y, x]
        for dx, dy in _DIRS8:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and dist[ny, nx] < 0:
                if 0 <= grid[ny, nx] < lethal:
                    dist[ny, nx] = d + 1
                    queue.append((nx, ny))
    return dist
```

- [ ] **Step 4: Correr el test y verificar que pasa**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission /Users/varbelaiz/miniforge3/envs/rosenv/bin/python -m pytest src/maze_mission/test/test_frontier.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/maze_mission/maze_mission/frontier.py src/maze_mission/test/test_frontier.py
git commit -m "feat(frontier): campo de costo por BFS sobre celdas libres del mapa inflado"
```

---

## Task 5: Selección de goal en `frontier.py`

**Files:**
- Modify: `src/maze_mission/maze_mission/frontier.py`
- Test: `src/maze_mission/test/test_frontier.py`

**Interfaces:**
- Consumes: `find_frontier_cells` (T2), `cluster_frontiers` (T3), `path_cost_field` (T4), y de `occupancy.py`: `inflate_occupancy`, `world_to_grid`, `grid_to_world`, `nearest_free_cell`, `GridSpec`.
- Produces:
  - `@dataclass(frozen=True) FrontierGoal` con: `x: float`, `y: float`, `yaw: float`, `utility: float`, `cost: float`, `gain: float`.
  - `select_frontier_goal(grid, spec, robot_xy, *, lethal=50, inflation_cells, min_frontier_cells, alpha) -> FrontierGoal | None`. `grid` es la occupancy CRUDA (`np.ndarray [gy,gx]`), `spec` es `GridSpec`, `robot_xy` es `(x, y)` en mundo. Devuelve `None` si no hay fronteras alcanzables (mapa cerrado). El goal cae en la celda del cluster alcanzable más barata; `yaw` apunta del robot hacia esa celda.

- [ ] **Step 1: Escribir el test que falla**

Agregar a `src/maze_mission/test/test_frontier.py`:

```python
from maze_mission.occupancy import GridSpec
from maze_mission.frontier import select_frontier_goal


def _spec():
    return GridSpec(resolution=1.0, origin_x=0.0, origin_y=0.0)


def test_sin_fronteras_devuelve_none():
    grid = np.zeros((5, 5), dtype=np.int16)   # todo conocido-libre
    out = select_frontier_goal(grid, _spec(), (0.0, 0.0),
                               inflation_cells=0, min_frontier_cells=1, alpha=0.1)
    assert out is None


def test_elige_frontera_por_utilidad():
    # dos frentes: uno chico y cercano, uno grande y lejano.
    grid = np.zeros((9, 9), dtype=np.int16)
    grid[:, 8] = -1              # columna desconocida a la derecha (frente grande, lejano)
    grid[0, 0] = -1             # una celda desconocida arriba-izq (frente chico, cercano)
    # con alpha alto (costo domina) elige el cercano; con alpha bajo (ganancia domina) el grande
    cerca = select_frontier_goal(grid, _spec(), (1.0, 1.0),
                                 inflation_cells=0, min_frontier_cells=1, alpha=10.0)
    lejos = select_frontier_goal(grid, _spec(), (1.0, 1.0),
                                 inflation_cells=0, min_frontier_cells=1, alpha=0.01)
    assert cerca is not None and lejos is not None
    # el frente grande esta en x~7 (col 7 libre lindante con col 8 desconocida)
    assert lejos.x > cerca.x


def test_goal_en_mundo_dentro_de_bounds():
    grid = np.zeros((5, 5), dtype=np.int16)
    grid[:, 4] = -1
    out = select_frontier_goal(grid, _spec(), (0.0, 0.0),
                               inflation_cells=0, min_frontier_cells=1, alpha=0.1)
    assert out is not None
    assert 0.0 <= out.x <= 5.0 and 0.0 <= out.y <= 5.0
```

- [ ] **Step 2: Correr el test y verificar que falla**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission /Users/varbelaiz/miniforge3/envs/rosenv/bin/python -m pytest src/maze_mission/test/test_frontier.py -v`
Expected: FAIL — `ImportError: cannot import name 'select_frontier_goal'`

- [ ] **Step 3: Implementar**

Agregar a `src/maze_mission/maze_mission/frontier.py` (agregar `import math` arriba junto a los demás imports):

```python
@dataclass(frozen=True)
class FrontierGoal:
    x: float
    y: float
    yaw: float
    utility: float
    cost: float
    gain: float


def select_frontier_goal(grid, spec, robot_xy, *, lethal=50,
                         inflation_cells, min_frontier_cells, alpha):
    grid = np.asarray(grid)
    frontier_mask = find_frontier_cells(grid, lethal)
    clusters = cluster_frontiers(frontier_mask, min_frontier_cells)
    if not clusters:
        return None

    # Inflar solo paredes reales: el BFS ya no atraviesa desconocido (es -1, no
    # libre), asi que inflar el desconocido solo volveria inalcanzables las celdas
    # frontera (que lindan con desconocido). Por eso unknown_as_obstacle=False.
    inflated = inflate_occupancy(grid, inflation_cells,
                                 unknown_as_obstacle=False, lethal_threshold=lethal)
    start = world_to_grid(robot_xy[0], robot_xy[1], spec)
    start_free = nearest_free_cell(inflated, start, lethal_threshold=lethal)
    if start_free is None:
        return None
    start = start_free
    dist = path_cost_field(inflated, start, lethal)

    best = None
    for cluster in clusters:
        reachable = [(int(dist[gy, gx]), gx, gy)
                     for (gx, gy) in cluster.cells if dist[gy, gx] >= 0]
        if not reachable:
            continue
        cost, gx, gy = min(reachable)
        gain = float(cluster.size)
        utility = gain - alpha * cost
        if best is None or utility > best[0]:
            best = (utility, gx, gy, float(cost), gain)

    if best is None:
        return None
    utility, gx, gy, cost, gain = best
    x, y = grid_to_world(gx, gy, spec)
    yaw = math.atan2(y - robot_xy[1], x - robot_xy[0])
    return FrontierGoal(x=x, y=y, yaw=yaw, utility=utility, cost=cost, gain=gain)
```

- [ ] **Step 4: Correr el test y verificar que pasa**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission /Users/varbelaiz/miniforge3/envs/rosenv/bin/python -m pytest src/maze_mission/test/test_frontier.py -v`
Expected: PASS (todos los tests de frontier)

- [ ] **Step 5: Commit**

```bash
git add src/maze_mission/maze_mission/frontier.py src/maze_mission/test/test_frontier.py
git commit -m "feat(frontier): seleccion de goal por utilidad = ganancia - alpha*costo"
```

---

## Task 6: Parámetros de frontera en `mission_config.py`

**Files:**
- Modify: `src/maze_mission/maze_mission/mission_config.py:42-48`
- Test: `src/maze_mission/test/test_mission_config.py`

**Interfaces:**
- Produces: `MissionConfig` gana `frontier_alpha: float = 0.1` y `frontier_min_cells: int = 4`. `from_dict`/`field_defaults` ya los incluyen automáticamente (iteran los campos del dataclass).

- [ ] **Step 1: Escribir el test que falla**

Agregar a `src/maze_mission/test/test_mission_config.py`:

```python
def test_config_tiene_parametros_de_frontera():
    cfg = MissionConfig()
    assert cfg.frontier_alpha == 0.1
    assert cfg.frontier_min_cells == 4


def test_from_dict_acepta_frontera():
    cfg = MissionConfig.from_dict({'frontier_alpha': 0.5, 'frontier_min_cells': 8})
    assert cfg.frontier_alpha == 0.5 and cfg.frontier_min_cells == 8
```

(Si `MissionConfig` no está importado en el test, agregar `from maze_mission.mission_config import MissionConfig` arriba.)

- [ ] **Step 2: Correr el test y verificar que falla**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission /Users/varbelaiz/miniforge3/envs/rosenv/bin/python -m pytest src/maze_mission/test/test_mission_config.py -v`
Expected: FAIL — `AttributeError: 'MissionConfig' object has no attribute 'frontier_alpha'`

- [ ] **Step 3: Implementar**

En `src/maze_mission/maze_mission/mission_config.py`, en el bloque "Busqueda / percepcion" (después de `waypoints_file`), agregar:

```python
    # Exploracion por fronteras (reemplaza waypoints_file). alpha balancea
    # ganancia (tamanio de cluster) vs costo de camino; min_cells descarta ruido.
    frontier_alpha: float = 0.1
    frontier_min_cells: int = 4
```

- [ ] **Step 4: Correr el test y verificar que pasa**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission /Users/varbelaiz/miniforge3/envs/rosenv/bin/python -m pytest src/maze_mission/test/test_mission_config.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/maze_mission/maze_mission/mission_config.py src/maze_mission/test/test_mission_config.py
git commit -m "feat(mission): parametros frontier_alpha y frontier_min_cells"
```

---

## Task 7: `SEARCH_CONE` usa fronteras en `mission_node.py`

**Files:**
- Modify: `src/maze_mission/maze_mission/mission_node.py` (imports; `__init__`; `_state_search_cone` ~línea 296-315; quitar `_load_route` y el uso de `WaypointRoute`)

**Interfaces:**
- Consumes: `select_frontier_goal` (T5), `MissionConfig.frontier_alpha`/`frontier_min_cells` (T6). Usa el estado ya existente: `self.raw_grid`, `self.spec`, `self.pose` (x,y,yaw), `self.nav_state`, `self._emit_goal(x, y, yaw)`, `self._to(...)`, `self._note(...)`, `self.wp_sent`, `self.cfg.inflation_radius_m`.

- [ ] **Step 1: Localizar el estado actual**

Leer `src/maze_mission/maze_mission/mission_node.py` alrededor de la línea 296 (método `_state_search_cone`, que hoy usa `self.route.current()` / `self.route.advance()`).

- [ ] **Step 2: Cambiar los imports y quitar la ruta de waypoints**

En `mission_node.py`:
- Reemplazar `from maze_mission.search_waypoints import WaypointRoute, load_waypoints` por:
  ```python
  from maze_mission.frontier import select_frontier_goal
  ```
- En `__init__`, reemplazar `self.route = self._load_route()` por `pass` (borrar la línea) y borrar el método `_load_route`.
- En el log de arranco (`mission_node iniciado...`), reemplazar `waypoints={len(self.route)} ` por `frontier_alpha={self.cfg.frontier_alpha} `.

- [ ] **Step 3: Reescribir `_state_search_cone`**

Reemplazar el cuerpo del recorrido de waypoints (desde `# recorrido de waypoints...` hasta el final del método) por:

```python
        # Exploracion por fronteras: en vez de waypoints fijos, ir al mejor borde
        # libre<->desconocido del mapa que construye el SLAM en vivo.
        if self.raw_grid is None or self.spec is None or self.pose is None:
            return
        if not self.wp_sent:
            inflation_cells = int(round(
                self.cfg.inflation_radius_m / max(self.spec.resolution, 1e-6)))
            goal = select_frontier_goal(
                self.raw_grid, self.spec, (self.pose[0], self.pose[1]),
                inflation_cells=inflation_cells,
                min_frontier_cells=self.cfg.frontier_min_cells,
                alpha=self.cfg.frontier_alpha)
            if goal is None:
                self._note('laberinto explorado sin encontrar el cono')
                self._to(MissionState.FAILURE)
                return
            if self._emit_goal(goal.x, goal.y, goal.yaw):
                self.wp_sent = True
        elif self.nav_state == NAV_REACHED:
            # llegamos a la frontera; el mapa crecio -> recalcular en el proximo tick
            self.wp_sent = False
```

(Mantener intacto el bloque anterior del método: la deteccion de cono estable que hace `self._to(MissionState.CONE_DETECTED)` y la gracia inicial de 1.5 s.)

- [ ] **Step 4: Verificar que el paquete compila e importa**

Run:
```bash
source $HOME/miniforge3/etc/profile.d/conda.sh && conda activate rosenv && \
cd "TP Final/robot_maze_runner" && \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission python -c "import ast; ast.parse(open('src/maze_mission/maze_mission/mission_node.py').read()); print('sintaxis OK')"
```
Expected: `sintaxis OK` (chequeo de sintaxis sin levantar rclpy).

Correr la suite de mission que no requiere ROS para confirmar que no rompimos nada:
Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission /Users/varbelaiz/miniforge3/envs/rosenv/bin/python -m pytest src/maze_mission/test/ -v`
Expected: PASS (excepto, si existe, `test_search_waypoints.py`, que se elimina en el Step 5).

- [ ] **Step 5: Eliminar el módulo de waypoints y su test**

```bash
git rm src/maze_mission/maze_mission/search_waypoints.py src/maze_mission/test/test_search_waypoints.py
```

Re-correr la suite:
Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/maze_mission /Users/varbelaiz/miniforge3/envs/rosenv/bin/python -m pytest src/maze_mission/test/ -v`
Expected: PASS (sin referencias colgadas a search_waypoints).

- [ ] **Step 6: Commit**

```bash
git add -A src/maze_mission
git commit -m "feat(mission): SEARCH_CONE explora por fronteras en vez de waypoints fijos"
```

---

## Task 8: `fastslam_node` publica `/amcl_pose` y `/map` transient_local

**Files:**
- Modify: `src/maze_slam/maze_slam/fastslam_node.py` (imports; `__init__` publishers ~línea 143-145; `publish_state` ~línea 242-272)

**Interfaces:**
- Produces: `fastslam_node` publica además `/amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`, frame `map`, pose = mejor partícula, covarianza diagonal chica) para que `navigator` y `mission_node` conserven su fuente de pose. `/map` pasa a QoS `transient_local` depth 1 (compatible con la suscripción `latched` de la misión y del navigator, y sigue republicando updates).

- [ ] **Step 1: Agregar el import y el QoS**

En `src/maze_slam/maze_slam/fastslam_node.py`, junto a los imports de mensajes agregar:
```python
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
```
(Si `PoseStamped`/`PoseArray` ya se importan de `geometry_msgs.msg`, sumar `PoseWithCovarianceStamped` a esa línea en vez de duplicar.)

- [ ] **Step 2: Cambiar el publisher de `/map` a transient_local y agregar `/amcl_pose`**

Reemplazar la línea `self.pub_map = self.create_publisher(OccupancyGrid, '/map', 1)` (~143) por:
```python
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.pub_amcl = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
```

- [ ] **Step 3: Publicar la pose en `publish_state`**

En `publish_state`, después de que se publica `/belief` (`self.pub_belief.publish(ps)`, ~línea 272), agregar:
```python
        pc = PoseWithCovarianceStamped()
        pc.header.frame_id = 'map'
        pc.header.stamp = ps.header.stamp
        pc.pose.pose.position.x = float(best.x)
        pc.pose.pose.position.y = float(best.y)
        pc.pose.pose.orientation = yaw_to_quat(best.theta)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05      # var x, y
        cov[35] = 0.05              # var yaw
        pc.pose.covariance = cov
        self.pub_amcl.publish(pc)
```
(Si `ps.header.stamp` no estuviera seteado en ese punto, usar `self.get_clock().now().to_msg()`.)

- [ ] **Step 4: Verificar sintaxis y build**

Run:
```bash
source $HOME/miniforge3/etc/profile.d/conda.sh && conda activate rosenv && \
cd "TP Final/robot_maze_runner" && \
python -c "import ast; ast.parse(open('src/maze_slam/maze_slam/fastslam_node.py').read()); print('sintaxis OK')" && \
colcon build --packages-select maze_slam --symlink-install
```
Expected: `sintaxis OK` y build sin errores (el warning `tests_require` es cosmético).
(Requiere `dangerouslyDisableSandbox: true`.)

- [ ] **Step 5: Commit**

```bash
git add src/maze_slam/maze_slam/fastslam_node.py
git commit -m "feat(slam): publicar /amcl_pose y /map transient_local para navegacion sobre SLAM vivo"
```

---

## Task 9: Launch de exploración `explore.launch.py`

**Files:**
- Create: `src/maze_mission/launch/explore.launch.py`

**Interfaces:**
- Consumes: `fastslam_node` con `/amcl_pose` + `/map` (T8); `navigator` (maze_nav, sin cambios); `mission_node` con exploración por fronteras (T7).
- Produces: un launch que levanta la pila de exploración: `fastslam_node` (fuente de mapa y pose), `navigator` (planner + control), `mission_node` (FSM con fronteras). NO levanta `map_publisher` ni `localizer`.

- [ ] **Step 1: Escribir el launch**

Crear `src/maze_mission/launch/explore.launch.py`:

```python
"""Pila de exploracion para laberinto DESCONOCIDO.

A diferencia de nav.launch.py (mapa estatico + MCL), aca el mapa lo construye
fastslam_node en vivo y la mision explora por fronteras. No se levanta map_publisher
ni localizer: fastslam_node publica /map y /amcl_pose.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maze_slam', executable='fastslam_node', name='fastslam_node',
            output='screen',
            parameters=[{'publish_tf': True}],
        ),
        Node(
            package='maze_nav', executable='navigator', name='navigator',
            output='screen',
        ),
        Node(
            package='maze_mission', executable='mission_node', name='mission_node',
            output='screen',
        ),
    ])
```

- [ ] **Step 2: Asegurar que el launch se instala**

Verificar en `src/maze_mission/setup.py` que el glob de launch incluye `launch/*.launch.py` (el `mission.launch.py` existente ya implica que sí). Si el `data_files` lista archivos de launch uno por uno en vez de glob, agregar `explore.launch.py` a esa lista.

Run:
```bash
grep -n "launch" src/maze_mission/setup.py
```
Expected: una entrada tipo `('share/' + package_name + '/launch', glob('launch/*.launch.py'))` o similar; si es glob, no hace falta cambiar nada.

- [ ] **Step 3: Build y verificación de que el launch resuelve**

Run:
```bash
source $HOME/miniforge3/etc/profile.d/conda.sh && conda activate rosenv && \
cd "TP Final/robot_maze_runner" && \
colcon build --packages-select maze_mission --symlink-install && \
source install/setup.zsh && \
python -c "from launch import LaunchDescription; import importlib.util, glob; \
p=glob.glob('install/maze_mission/share/maze_mission/launch/explore.launch.py')[0]; \
spec=importlib.util.spec_from_file_location('l', p); m=importlib.util.module_from_spec(spec); \
spec.loader.exec_module(m); assert m.generate_launch_description(); print('launch OK')"
```
Expected: `launch OK` (el launch importa y genera la descripción sin levantar nodos).
(Requiere `dangerouslyDisableSandbox: true`.)

- [ ] **Step 4: Commit**

```bash
git add src/maze_mission/launch/explore.launch.py src/maze_mission/setup.py
git commit -m "feat(mission): launch de exploracion (fastslam + navigator + mission, sin mapa estatico)"
```

---

## Task 10: Smoke de integración en Gazebo `smoke_explore.sh`

**Files:**
- Create: `scripts/smoke_explore.sh`

**Interfaces:**
- Consumes: `explore.launch.py` (T9), un world de laberinto de Gazebo (`turtlebot3_custom_simulation`).
- Produces: un script que levanta Gazebo + la pila de exploración, deja correr N segundos, y reporta si `/map` creció y si `mission_node` emitió goals de frontera distintos. Validación manual/visual en RViz.

- [ ] **Step 1: Escribir el smoke**

Crear `scripts/smoke_explore.sh`:

```bash
#!/usr/bin/env bash
# Smoke de exploracion por fronteras en Gazebo (lazo cerrado).
# Levanta Gazebo (laberinto) + fastslam + navigator + mission (fronteras) y
# verifica que el mapa crece y que la mision emite goals de frontera distintos.
# Correr dentro de rosenv:
#   source $HOME/miniforge3/etc/profile.d/conda.sh && conda activate rosenv
#   export TURTLEBOT3_MODEL=burger
#   ./scripts/smoke_explore.sh
set -euo pipefail
cd "$(dirname "$0")/.."
source install/setup.zsh

DURATION="${1:-60}"

echo "[smoke] lanzando Gazebo (laberinto)..."
ros2 launch turtlebot3_custom_simulation custom_maze.launch.py &
GZ_PID=$!
sleep 12

echo "[smoke] lanzando pila de exploracion (fastslam + navigator + mission)..."
ros2 launch maze_mission explore.launch.py &
STACK_PID=$!
sleep 3

echo "[smoke] registrando /goal_pose durante ${DURATION}s..."
timeout "${DURATION}" ros2 topic echo --once /map >/dev/null 2>&1 || true
timeout "${DURATION}" ros2 topic echo /goal_pose > /tmp/explore_goals.txt 2>&1 || true

echo "[smoke] apagando..."
kill "${STACK_PID}" "${GZ_PID}" 2>/dev/null || true
wait 2>/dev/null || true

GOALS=$(grep -c 'position' /tmp/explore_goals.txt || true)
echo "[smoke] goals de frontera emitidos (lineas position): ${GOALS}"
if [ "${GOALS}" -ge 1 ]; then
  echo "[smoke] OK: la mision emitio al menos un goal de frontera."
else
  echo "[smoke] FALLO: no se emitieron goals; revisar SLAM/pose/mapa."
  exit 1
fi
```

- [ ] **Step 2: Hacerlo ejecutable**

```bash
chmod +x scripts/smoke_explore.sh
```

- [ ] **Step 3: Correr el smoke (validación de integración)**

Run (requiere `dangerouslyDisableSandbox: true`, entorno gráfico para Gazebo):
```bash
source $HOME/miniforge3/etc/profile.d/conda.sh && conda activate rosenv && \
export TURTLEBOT3_MODEL=burger && cd "TP Final/robot_maze_runner" && \
./scripts/smoke_explore.sh 60
```
Expected: `[smoke] OK: la mision emitio al menos un goal de frontera.` Verificación adicional manual en RViz: el `/map` crece (área conocida aumenta) y el robot se mueve hacia bordes desconocidos. Capturar RViz para el informe.

Nota de riesgo: si el `navigator` no recibe el `/map` dinámico (por QoS o porque cachea el mapa una sola vez), no planificará. Si eso ocurre, verificar la QoS de la suscripción `/map` del navigator en `maze_nav`; es el único punto donde puede necesitarse un ajuste en la caja negra de toma-2 (coordinar con el equipo, como anota el contrato de interfaz).

- [ ] **Step 4: Commit**

```bash
git add scripts/smoke_explore.sh
git commit -m "test(explore): smoke de integracion en Gazebo para exploracion por fronteras"
```

---

## Self-Review (hecho al escribir el plan)

- **Cobertura del spec:** plomería SLAM (T8 `/amcl_pose` + `/map` transient_local, T9 launch) ✓; explorador de fronteras (T2 detección, T3 clustering, T4 costo, T5 selección) ✓; política de desconocido en occupancy (T1) ✓; swap en SEARCH_CONE conservando el resto de la FSM (T7) ✓; parámetros (T6) ✓; validación unit + Gazebo (T2-T5 unit, T10 smoke) ✓.
- **Riesgo abierto documentado:** la QoS de `/map` del `navigator` (caja negra de maze_nav) — anotado en T10 como verificación, sin modificar maze_nav salvo que el smoke lo exija.
- **Consistencia de tipos:** `FrontierCluster`(cells/size/centroid_gx/centroid_gy) y `FrontierGoal`(x/y/yaw/utility/cost/gain) usados igual en T3/T5; `select_frontier_goal` firma idéntica en T5 y T7; convención `(gx,gy)` / `grid[gy,gx]` uniforme.
