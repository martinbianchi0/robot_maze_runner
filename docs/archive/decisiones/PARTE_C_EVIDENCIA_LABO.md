# Parte C - Kit de evidencia reproducible para laboratorio

Fecha: 2026-07-02
Estado: aceptada

## Contexto

La evidencia grabada con celular no muestra mapa, path, estados ni motivos de
bloqueo. Para la defensa hace falta reconstruir una corrida real con datos ROS:
trayectoria, goals, plan, estados de mision/navegacion, detecciones y eventos de
recovery ante obstaculos no mapeados.

## Opciones consideradas

- Solo video de RViz: es visual y rapido, pero no deja datos para diagnosticar.
- Solo rosbag completo: conserva todo, pero cuesta convertirlo en evidencia para
  informe en el momento.
- Rosbag + logger liviano + reporte post-run: guarda datos crudos y tambien deja
  resumen, timeline y overlay listos para revisar.

## Decision

Usar un kit chico en `scripts/lab_record_all.sh`: graba rosbag, corre
`lab_live_logger.py`, guarda metadata y genera `summary.md`, `timeline.csv` y
`map_overlay.png`/`.svg` en `results/labo_demo/<timestamp>/`.

## Consecuencias

No se introduce OBS ni dependencias nuevas obligatorias. Si `matplotlib` no esta,
el reporte cae a SVG manual. La grabacion de pantalla queda opcional en
`scripts/lab_record_rviz.sh` y se desactiva sin fallar si no existe `ffmpeg`.

## Uso en laboratorio

```bash
# Terminal 1
./scripts/lab_record_all.sh tb4_0

# Terminal 2
ros2 launch maze_nav nav_tb4_live.launch.py \
  map_yaml:=maps/laberinto_lab_20260702.yaml ns:=tb4_0

# Terminal 3
ros2 launch maze_mission mission.launch.py \
  params_file:=$(pwd)/config/parte_c/real.yaml

# Terminal 4
ros2 launch maze_perception cone_detector.launch.py \
  params_file:=$(pwd)/config/parte_c/real.yaml

# Terminal 5
rviz2 -d src/maze_nav/rviz/nav.rviz --ros-args -p use_sim_time:=false \
  --remap /tf:=/tb4_0/tf --remap /tf_static:=/tb4_0/tf_static --remap /scan:=/tb4_0/scan
```

Post-run manual si hiciera falta:

```bash
python3 scripts/lab_make_report.py results/labo_demo/<timestamp>
```

Grabacion opcional de RViz:

```bash
./scripts/lab_record_rviz.sh results/labo_demo/<timestamp>
```

## Como generar un video para el informe

### A. Grabacion en vivo

En una terminal dejar corriendo la captura de datos:

```bash
./scripts/lab_record_all.sh tb4_0
```

En otra terminal, con RViz abierto, grabar pantalla si el entorno tiene
`ffmpeg` y `DISPLAY`:

```bash
./scripts/lab_record_rviz.sh results/labo_demo/<timestamp>
```

El MP4 queda dentro del mismo directorio de resultados, por ejemplo
`results/labo_demo/<timestamp>/rviz_screen_<date>.mp4`.

### B. Replay post-labo

El replay usa el rosbag guardado en `<run_dir>/rosbag`, publica `/clock` y abre
RViz con `use_sim_time:=true`:

```bash
./scripts/lab_replay_rviz.sh results/labo_demo/<timestamp> tb4_0
./scripts/lab_record_rviz.sh results/labo_demo/<timestamp>
```

Para reproducir mas lento:

```bash
RATE=0.5 ./scripts/lab_replay_rviz.sh results/labo_demo/<timestamp> tb4_0
```

La vista dedicada esta en `rviz/lab_replay.rviz`. Por default usa `/tb4_0/scan`;
para `tb4_1` puede requerir editar ese display o ajustar la configuracion RViz.
