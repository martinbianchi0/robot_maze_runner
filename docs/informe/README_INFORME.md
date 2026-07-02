# README informe sucio

Carpeta de trabajo para la primera version del informe IEEE/Overleaf.

Archivos:

- `informe_sucio.tex`: borrador LaTeX compilable con placeholders.
- `evidence_pack.md`: dump tecnico para copiar/depurar contenido.
- `figuras_pendientes.md`: lista de figuras a capturar o dibujar.
- `claims_checklist.md`: claims seguros y pendientes.
- `figures/`: destino de imagenes para Overleaf.

## Compilar localmente

Desde la raiz del repo:

```bash
cd docs/informe
pdflatex informe_sucio.tex
pdflatex informe_sucio.tex
```

Alternativa si hay `latexmk`:

```bash
cd docs/informe
latexmk -pdf informe_sucio.tex
```

Si no hay LaTeX instalado, subir `docs/informe/` a Overleaf y compilar ahi. Los
placeholders no dependen de imagenes existentes: si un archivo de `figures/` no
esta, el PDF muestra una caja con texto.

Estado verificado en esta maquina: no se encontro `pdflatex` ni `latexmk`, por
lo tanto no se genero `informe_sucio.pdf` localmente.

## Que falta completar mañana

- Agregar autores reales.
- Insertar mapa final/captura de `maps/laberinto_lab_20260702`.
- Grabar una corrida con `lab_record_all.sh`.
- Generar `summary.md`, `timeline.csv` y `map_overlay.png`/`.svg`.
- Capturar replay RViz con trayectoria, path, states y detecciones.
- Obtener captura de `/cone_debug_image` y `/cone_mask`.
- Probar un obstaculo no mapeado real tipo silla/patas.
- Decidir si el resultado real permite afirmar "evito obstaculo" o solo "safe
  stop defendible".
- Completar la seccion de resultados con datos reales, no supuestos.
- Revisar el checklist de claims antes de entregar.

## Actualizar figuras

Guardar imagenes en `docs/informe/figures/` con los nombres sugeridos en
`figuras_pendientes.md`. El `.tex` ya busca esos nombres:

```text
fig_arquitectura_general.png
fig_mapa_laberinto.png
fig_flujo_navegacion.png
fig_rviz_goal_nav.png
fig_obstaculo_nav_debug.png
fig_fsm_mision_cono.png
fig_detector_cono.png
fig_rviz_replay_real.png
```

Cuando la imagen exista, LaTeX la incluye automaticamente. Si no existe, aparece
el placeholder.

## Pasar a Overleaf

Subir la carpeta `docs/informe/` completa:

```text
informe_sucio.tex
figures/
```

En Overleaf, configurar `informe_sucio.tex` como archivo principal. Mantener los
Markdown como material auxiliar fuera del proyecto final si no se quieren
entregar.

## Comandos para generar evidencia

Terminal 1:

```bash
./scripts/lab_record_all.sh tb4_0
```

Terminal 2:

```bash
ros2 launch maze_nav nav_tb4_live.launch.py \
  map_yaml:=maps/laberinto_lab_20260702.yaml ns:=tb4_0
```

Terminal 3:

```bash
ros2 launch maze_mission mission.launch.py \
  params_file:=$(pwd)/config/parte_c/real.yaml
```

Terminal 4:

```bash
ros2 launch maze_perception cone_detector.launch.py \
  params_file:=$(pwd)/config/parte_c/real.yaml
```

Terminal 5:

```bash
rviz2 -d src/maze_nav/rviz/nav.rviz --ros-args -p use_sim_time:=false \
  --remap /tf:=/tb4_0/tf \
  --remap /tf_static:=/tb4_0/tf_static \
  --remap /scan:=/tb4_0/scan
```

Post-run:

```bash
python3 scripts/lab_make_report.py results/labo_demo/<timestamp>
```

Replay:

```bash
./scripts/lab_replay_rviz.sh results/labo_demo/<timestamp> tb4_0
```

Video de RViz si hay `ffmpeg` y `DISPLAY`:

```bash
./scripts/lab_record_rviz.sh results/labo_demo/<timestamp>
```
