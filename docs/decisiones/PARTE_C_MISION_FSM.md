# Parte C - Maquina de estados de mision (C5, validada con cono mockeado)

> Estado: FSM implementada en `maze_mission/mission_node.py` y validada end-to-end
> con percepcion MOCKEADA (etapa C5). Junta percepcion (M1) + estimacion
> LIDAR-fusion (M1) + validacion geometrica (M0) + navegacion (M2).

## Rol

Capa de SUPERVISION sobre la pila de navegacion de toma-2: coordina percepcion y
navegacion para encontrar y alcanzar el unico cono rojo. **No hace control de bajo
nivel**: emite goals por `/goal_pose` y observa `/nav_state`; el navigator resuelve
A* + pure-pursuit + recovery.

## Estados (13)

```
INIT -> LOAD_MAP -> LOCALIZE -> SEARCH_CONE
  -> CONE_DETECTED -> ESTIMATE_CONE_GOAL -> PLAN_TO_CONE -> NAVIGATE_TO_CONE
  -> (AVOID_OBSTACLE -> REPLAN)* -> VERIFY_CONE -> DONE
  (FAILURE como escape con aborto seguro)
```

Coordinacion con nav (observa, no controla): exito = `/nav_state == REACHED`;
bloqueo sostenido = `RECOVERY` -> `AVOID_OBSTACLE`; sin progreso/timeout -> `REPLAN`
(reintenta `max_replans` veces, luego `FAILURE`). Retargeting = re-emitir
`/goal_pose` (preempt).

## Invariante de seguridad (no cruzar paredes)

DOS chequeos, ambos antes de mover el robot:

1. **Cono detras de pared (mapa CRUDO)**: en `PLAN_TO_CONE`, si el punto estimado
   del cono cae sobre un obstaculo del mapa **crudo** (`_cone_on_obstacle`), se
   RECHAZA sin snapear y se vuelve a `SEARCH_CONE`. Es el caso "veo el cono por una
   reja": el LIDAR golpea la pared, el punto estimado cae sobre la pared -> se
   descarta. NO se snapea a la cara de la pared (eso haria acercarse al muro).
2. **Goal navegable (mapa INFLADO)**: el UNICO publicador de `/goal_pose` es
   `_emit_goal()`, que valida contra el mapa **inflado** (reusa
   `goal_validator`/`occupancy`): VALID / SNAPPED (ruido metrico chico) / REJECTED.

Se guarda el mapa crudo + inflado para poder distinguir "cono sobre pared real"
(rechazar) de "goal en el margen de inflado" (snap chico permitido).

## Del cono al goal

`ESTIMATE_CONE_GOAL` usa LIDAR-fusion (`cone_goal_estimator.cone_world_from_lidar`,
offset del LIDAR por perfil). `PLAN_TO_CONE` apunta a un punto con **stand-off**
(`cone_standoff_m`, frena antes del cono, no lo pisa). `VERIFY_CONE` confirma el
cono de cerca (area >= `verify_area_px_min`).

## Validacion C5 (`scripts/smoke_mission.sh`, cono mockeado)

Harness sin percepcion real: `map_publisher` + `navigator` + `fake_diff_drive`
(mini-sim) + `mission_node` + `mock_cone_publisher` (publica cone_detections + un
scan sintetico que ubican el cono en un punto de mundo fijo) + `mission_monitor`
(verifica el invariante: todo `/goal_pose` en celda libre).

| Escenario | Estados | Resultado |
|---|---|---|
| **reachable** (cono en libre) | SEARCH -> CONE_DETECTED -> ESTIMATE -> PLAN -> NAVIGATE -> VERIFY -> DONE | DONE; 1 goal, 0 en pared |
| **wall** (cono sobre pared) | SEARCH -> CONE_DETECTED -> ESTIMATE -> PLAN -> SEARCH (loop) | RECHAZADO; **0 goals** hacia la pared |

En ambos: `all_goals_free = true`, `goals_en_pared = 0`. La demo central (no cruzar
la pared) queda demostrada.

Correr:
```bash
bash scripts/smoke_mission.sh reachable
bash scripts/smoke_mission.sh wall
```

## Pendiente / real-only

- `LOCALIZE`: hoy pasa con pose disponible; el criterio real de convergencia de la
  MCL (varianza de `/particlecloud`) queda para el robot real. Ademas la MCL
  necesita el reencuadre +90 del scan (ver INTERFAZ_MAZE_NAV.md).
- `SEARCH_CONE`: recorre waypoints; falta el giro-scan en cada uno (para el cono en
  lugar desconocido) y waypoints reales sobre el mapa del laberinto (hoy placeholder).
- Fallback de servoing bearing-only si el LIDAR no da rango (hoy vuelve a SEARCH).
- C6: percepcion real (M1 ya calibrada) + MCL real, en el robot fisico.
