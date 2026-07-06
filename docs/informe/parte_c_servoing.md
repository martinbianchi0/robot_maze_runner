# Parte C — Búsqueda del cono: fallback de servoing visual

_Evidencia del turno de laboratorio del 2026-07-06 (TB4 real `tb4_0`, mapa `laberinto_lab_20260702`)._

## Problema detectado

El `mission_node` estima la posición del cono en el frame `map` usando el
**rango del LIDAR** en el bearing donde la cámara detecta el cono
(`cone_world_from_lidar`). En el laberinto real, el cono es **más bajo que el
plano del RPLIDAR del TB4**: la cámara (montada más alto, mirando ligeramente
hacia abajo) lo ve, pero el haz del LIDAR pasa por encima del cono y mide **la
pared de fondo**.

Consecuencia: el rango usado para proyectar es el de la pared, así que la
posición estimada del cono cae **sobre una celda ocupada del mapa crudo**. La
lógica original la rechazaba (`_cone_on_obstacle` → "DETRAS DE PARED") y volvía a
patrullar, **ignorando el cono aunque la cámara lo estuviera viendo**.

Verificación en vivo (robot al lado del cono):
- Las 3 posiciones estimadas rechazadas —`(0.45,-0.61)`, `(0.45,-0.64)`,
  `(-1.05,-5.08)`— caen sobre **pared real** del mapa (celda=0, `pocc=1.00`,
  rodeadas de pared en vecindad 5x5).
- Rango mínimo del LIDAR = 0.43 m: no ve nada más cerca (el cono, a ~0.3 m, queda
  bajo su plano).

## Solución: estado `SERVO_TO_CONE` (servoing visual)

Cuando el LIDAR no da un rango confiable del cono (sin retorno, o retorno sobre
una pared), en vez de descartar, la misión entra en un nuevo estado
`SERVO_TO_CONE` que **avanza con micro-goals cortos (0.25 m) sobre el bearing de
la cámara** hasta que el blob del cono es suficientemente grande
(`area_px >= verify_area_px_min`) → `DONE`.

**Invariante de seguridad preservado:** cada micro-goal pasa por `_emit_goal`, que
lo valida contra el mapa **inflado**. Nunca se emite un goal que cruce una pared;
ante un falso positivo tras pared, el micro-goal se rechaza y se vuelve a buscar.

Parámetros nuevos (`mission_config.py`, tunables por YAML/`ros2 param`):
- `servo_step_m = 0.25` — largo de cada micro-goal hacia el cono.
- `servo_max_steps = 8` — tope de micro-goals antes de rendirse.

## Máquina de estados (camino del cono)

```
SEARCH_CONE ──(cono estable ≥3 frames)──► CONE_DETECTED ──► ESTIMATE_CONE_GOAL
                                                                    │
                        ┌───────────────────────────────┬──────────┴─────────┐
                        │ rango LIDAR válido y libre     │ sin rango / sobre pared
                        ▼                                ▼
                  PLAN_TO_CONE ──libre──► NAVIGATE   [SERVO_TO_CONE]  ◄── NUEVO
                        │  cae en pared ─────────────────►│ micro-goal sobre bearing
                        ▼                                  │ (validado vs mapa inflado)
                  NAVIGATE_TO_CONE ──REACHED──► VERIFY ──► │
                                                           ▼
                                              area_px ≥ umbral ─► DONE
                                              cono perdido / tope ─► SEARCH_CONE
```

## Evidencia — corrida exitosa por servoing

Log del `mission_node` (el cono se puso en frente; el LIDAR volvió a ubicarlo
sobre la pared, pero el servoing lo alcanzó y confirmó):

```
mission_node iniciado. pose=/amcl_pose map=/map scan=/tb4_0/scan waypoints=16 lidar_offset=-90deg
MCL convergida (std xy=0.11 m, yaw=0.09 rad)
goal emitido (-1.58,0.73) [VALID]
cono en (-1.81,0.95) sobre pared (LIDAR vio el fondo): paso a servoing visual   ← fallback disparado
goal emitido (-1.41,0.62) [VALID]                                                ← micro-goal hacia el cono
cono confirmado de cerca por servoing visual                                     ← blob suficientemente grande
MISION COMPLETA: cono rojo alcanzado y verificado                                ← éxito
```

Estado final publicado: `/mission_state = DONE`, `/nav_state = REACHED`.

A diferencia de corridas previas, el éxito **no** dependió de que un waypoint de
patrulla cayera casualmente cerca del cono: la misión navegó al cono de forma
dirigida a partir de la detección visual.

Trayectoria: `figures/parte_c_trayectoria_c6.png` (la ✕ cian marca la estimación
LIDAR que cae sobre la pared y dispara el servoing).

## Evidencia — búsqueda con el cono FUERA del campo de visión inicial

Segunda validación: se colocó el cono **fuera de la vista inicial** de la cámara.
La misión patrulló waypoints con giro-scan de 360° hasta barrer el cono, y al
detectarlo navegó a él (esta vez el LIDAR sí pudo rangearlo, sin necesitar
servoing) y verificó:

```
mission_node iniciado. ... waypoints=16 lidar_offset=-90deg
MCL convergida (std xy=0.10 m, yaw=0.11 rad)
goal emitido (-1.58,0.73) [VALID]     ← waypoint patrulla 1 (giro-scan)
goal emitido (-0.74,-1.63) [VALID]    ← waypoint patrulla 2 (giro-scan)
goal emitido (-0.54,-1.98) [VALID]    ← standoff al cono ya detectado
MISION COMPLETA: cono rojo alcanzado y verificado
```

Trayectoria: `figures/parte_c_trayectoria_c7.png` (start → 2 waypoints → cono).

En conjunto, las dos corridas cubren los dos caminos del código: navegación
directa cuando el LIDAR rangea el cono, y servoing visual cuando no (cono bajo el
plano del LIDAR).

## Figuras

| Figura | Descripción |
|--------|-------------|
| `figures/parte_c_cono_debug.png` | Imagen anotada del `cone_detector`: bounding box verde + centroide sobre el cono rojo (confianza 0.97, área 1937 px, bearing −2°). |
| `figures/parte_c_cono_mask.png` | Máscara HSV: silueta del cono segmentada (2077 px). |
| `figures/parte_c_cono_raw.png` | Frame crudo de la cámara OAK-D (preview 250×250) con el cono en cuadro. |
| `figures/parte_c_laberinto_raw.png` | Vista del laberinto real (paredes de cartón) sin cono en cuadro. |
| `figures/parte_c_trayectoria_c7.png` | Run de búsqueda (cono fuera de campo): secuencia de goals numerada (start=0 → waypoints → cono) sobre el mapa. |
| `figures/parte_c_trayectoria_c6.png` | Run de servoing: la ✕ cian marca la estimación LIDAR sobre la pared que dispara el fallback. |

Los gráficos muestran la **secuencia de goals emitidos** numerada sobre el mapa (no
se dibuja línea entre goals porque la ruta real la resuelve el A* del navigator
siguiendo pasillos). Los CSV crudos de `/amcl_pose` por-muestra están en
`logs/traj_amcl_pose_*.csv`; el registro por-frame de una corrida completa quedó
pendiente (se agotó la batería del TB4 antes de cerrar la captura).

Logs de mission de las tres corridas exitosas: `logs/parte_c6_mission.log`
(servoing), `logs/parte_c7_mission.log` (búsqueda + nav directa),
`logs/parte_c10_mission.log` (servoing, corrida final).

## Archivos modificados

- `src/maze_mission/maze_mission/mission_config.py` — params `servo_step_m`, `servo_max_steps`.
- `src/maze_mission/maze_mission/mission_node.py` — estado `SERVO_TO_CONE`, `_to_servo()`, `_state_servo_to_cone()`; redirección de los dos puntos de rechazo ("sin rango LIDAR" y "sobre pared") al servoing.

Tests del paquete `maze_mission`: 21/21 OK.
