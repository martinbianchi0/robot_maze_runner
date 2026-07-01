# Parte C - Estimacion cono -> mundo: LIDAR-fusion (validado en C1)

> Estado: VALIDADO contra el rosbag `laberinto_conos`. LIDAR-fusion queda como
> estrategia PRIMARIA de estimacion de la posicion del cono. El fallback
> bearing-only servoing queda implementado como respaldo (no fue necesario).

## Decision

Para convertir una deteccion visual del cono (bearing de la camara) en una
coordenada de mundo, se usa **LIDAR-fusion**: el bearing de la camara indexa el
`/scan` para obtener el rango metrico, y con la pose del robot se pasa a mundo.
Modulo: `maze_mission/cone_goal_estimator.py` (`cone_world_from_lidar`).

## Geometria (de la TF estatica del bag)

- La camara OAK-D mira al frente: cadena camara->base identidad en yaw, asi que
  `bearing_base = atan2(cx - u, fx)` (+ a la izquierda del robot). Intrinsecos del
  bag: `fx=203.14`, `cx=122.57`, imagen 250x250, HFOV ~63 deg.
- **El RPLIDAR esta montado a +90 deg respecto de base** (`shell_link ->
  rplidar_link`, yaw +90). Por eso el angulo 0 del scan apunta a la IZQUIERDA del
  robot y el indice del scan para un bearing de camara se busca en
  **`angulo_scan = bearing_base - 90 deg`** (`lidar_yaw_offset = -pi/2`).
- Origen del LIDAR ~4 cm detras del origen de base (`lidar_offset_x = -0.04`).
- Scan: 720 rayos, `angle_min=-179 deg`, incremento 0.499 deg, rango [0.15, 12] m.

## Validacion (etapa C1, `scripts/lidar_fusion_validate.py`)

Como el cono es estatico, sus estimaciones de mundo deben agrupar. Pero con el
robot casi quieto, hasta la geometria erronea agrupa; por eso el discriminador
principal es **independiente de la pose**: la fisica exige `sqrt(area) ~ 1/rango`
(cono mas cerca -> blob mas grande -> rango menor). Se compararon 3 offsets x 2
modos de rango sobre 240-400 detecciones del bag:

| offset / modo | corr(sqrt(area),1/rango) | cluster (ratio @0.3m) | MAD | rango_med |
|---|---|---|---|---|
| **-90 / nearest** | **+0.74 a +0.81** | 0.75 - 0.91 | **0.016 - 0.040 m** | 1.0 - 1.7 m |
| -90 / median | +0.75 | 0.60 | 0.011 - 0.048 | 1.1 - 2.3 |
| 0 / * | -0.10 a -0.60 | 0.22 | 0.10 | 0.6 - 1.6 |
| +90 / * | -0.12 a -0.89 | 0.11 | 0.12 | 0.3 - 0.7 |

Con diversidad de pose (robot recorriendo 2.5 x 5.6 m):
- **-90 / nearest gana claro**: correlacion positiva fuerte (mide el cono de
  verdad) y cluster de ~1.6 cm de dispersion. Offset 0 y +90 se desarman
  (ratio 0.22 / 0.11) y sus estimaciones se estiran en arcos siguiendo al robot.
- **`nearest` > `median`**: el median a veces agarra la pared detras del cono
  (ratio 0.60 vs 0.75-0.91). Default: `nearest`.

Evidencia en `results/parte_c/C1/lidar_fusion/`: `cone_positions_by_offset.png`
(scatter por offset), `frame_XX.png` (detecciones anotadas con rango/bearing),
`detections.csv`, `summary.json`.

## Parametrizacion

`lidar_yaw_offset` es parametro de mision (perfiles): **sim = 0.0** (LIDAR
alineado), **bag/real = -1.5708** (TB4 real). Tambien `lidar_offset_x`,
`lidar_sector_halfwidth` (3) y `cone_range_mode` (`nearest`). Defaults en
`MissionConfig` = TB4 real.

## Como reproducir

```bash
conda activate rosenv
python scripts/lidar_fusion_validate.py rosbags/laberinto_conos 400 2
```

## Calibracion HSV del detector (C1)

Barrido de umbrales sobre el bag, usando el ratio de cluster de LIDAR-fusion como
metrica de precision (`scripts/hsv_calibrate.py` para caracterizar,
`scripts/hsv_sweep.py` para comparar candidatos en una sola pasada). Hallazgos:

- El cono es rojo-naranja MUY saturado (S~207); los distractores (barreras
  beige/madera) son naranja poco saturado (S~66). **El piso de saturacion es el
  discriminador clave**: subirlo de 120 a 160 llevo la precision de 0.74 a 0.87
  manteniendo todas las detecciones del cono.
- El hue DEBE quedar angosto (rojo, 0-10 / 170-180). Ensancharlo hacia el naranja
  mete objetos naranja saturados y arruina la precision (cae a ~0.25-0.30).
- Umbrales finales en `RedHSVThresholds` (defaults): S>=160, hue rojo angosto,
  V>=70. Evidencia en `results/parte_c/C1/hsv/` (histograma S separando cono vs
  espurio, samples). Son parametros del nodo (`hsv.*`): reajustables por perfil.

## Limitaciones / pendientes

- Queda ~13% de detecciones outliers (rojo saturado espurio: reflejos u otro
  objeto rojo puntual). Se maneja en la FSM (M3) con deteccion estable N frames
  (`detection_stable_frames`) + validacion del goal contra el mapa.
- Clustering en frame odom: sobre el bag completo (~15 min) la odometria deriva;
  no afecta la conclusion (la correlacion es pose-independiente).
- Robot real: puede hacer falta reajustar S/V por iluminacion distinta del
  laboratorio; los umbrales son parametros (`hsv.*`), no hardcodeados.
