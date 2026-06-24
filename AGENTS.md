# AGENTS.md

Instrucciones de trabajo para agentes/IA en este repositorio.

Este repositorio es un workspace ROS 2 Humble para el proyecto `robot_maze_runner`, Trabajo Final de I402 - Principios de la Robótica Autónoma.

## Antes de modificar código

1. Leer primero:
   - `docs/contexto/FLUJO_TP_FINAL.md`
   - `docs/contexto/PARTE_A_SLAM.md`
   - `docs/contexto/PARTE_B_NAVEGACION.md`
   - `docs/contexto/PARTE_C_ROBOT_REAL.md`
   - `docs/contexto/ANALISIS_DECISION_ARQUITECTURA.md`
   - `docs/contexto/SIMULACION_BASE.md`
   - `docs/contexto/ESTRUCTURA_REPO.md`
   - `docs/consignas/`

2. Revisar si hay decisiones vigentes en:
   - `docs/decisiones/`

3. No implementar sin entender:
   - qué parte del TP afecta;
   - qué topics usa;
   - qué launch lo prueba;
   - qué evidencia se espera guardar;
   - qué decisión técnica está siguiendo.

4. Si la tarea es grande o ambigua, primero proponer plan. No saltar directo a código.

## Contexto del proyecto

El TP Final tiene tres partes:

```text
Parte A - SLAM
Parte B - Movimiento automático
Parte C - Autonomía completa real
```

La arquitectura preliminar recomendada, todavía no cerrada, es:

```text
Parte A:
  Opción 1 - Grid-Based FastSLAM,
  validada por etapas desde occupancy grid mapping con /calc_odom.

Parte B:
  mapa inflado + A* o Theta* + Pure Pursuit + máquina de estados + replanning simple.

Parte C:
  adaptar desde el principio a sim/real con topics, QoS y parámetros configurables.
```

La decisión final debe tomarse después de pruebas mínimas. Si una prueba contradice esta recomendación, documentar el hallazgo y proponer cambio.

## Estado verificado

Workspace:

```bash
~/Robotica/tp_final_ws
```

Comandos base:

```bash
cd ~/Robotica/tp_final_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Simulación base:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
```

Ya se verificó que la simulación actualizada publica:

```text
/scan
/odom
/calc_odom
```

En WSL puede fallar `gzclient`. No asumir que eso invalida la simulación si `gzserver` y los topics ROS siguen activos. Para visualizar, priorizar RViz.

RViz base:

```bash
LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 rviz2 -d rviz/tp_final_base.rviz
```

## Estructura esperada

```text
src/        paquetes ROS 2
docs/       consignas, contexto, decisiones, informe
maps/       mapas generados por el grupo
rviz/       configuraciones RViz
results/    capturas/resultados livianos
scripts/    utilidades
rosbags/    rosbags pesados, no subir salvo .gitkeep
```

No confundir `maps/` con los mundos/mapas internos de `src/turtlebot3_custom_simulation`.

## Reglas técnicas

- No hardcodear topics si pueden ser parámetros.
- Separar parámetros de simulación y robot real.
- No asumir que `/odom` y `/calc_odom` significan lo mismo.
- No asumir que TurtleBot3 simulado y TurtleBot4 real tienen los mismos topics, frames, QoS o geometría.
- Mantener velocidades conservadoras para lógica que luego pueda ir al robot real.
- Filtrar mediciones inválidas del LIDAR cuando se trabaje con robot real.
- Documentar toda decisión importante en `docs/decisiones/`.

## Separación de opciones de Parte A

No mezclar estas ramas:

```text
Opción 1:
  Grid-Based FastSLAM
  Gazebo
  LIDAR
  occupancy grid

Opción 2:
  Features con LIDAR
  EKF SLAM / Graph SLAM / SEIF SLAM
  problema principal: extracción robusta de features

Opción 3:
  RosBag / TurtleBot4
  cámara + LIDAR
  ArUco tags
  Graph SLAM
  problema principal: visión, frames, RosBag y optimización
```

FastSLAM con landmarks de TP5 sirve como referencia conceptual, pero no debe mezclarse sin criterio con Opción 2.

## Reutilización de TPs previos

Puede servir como referencia:

```text
TP4:
  EKF, belief, odometría, paths.

TP5:
  SLAM, landmarks, covarianzas, FastSLAM, MarkerArray, RViz.

TP6:
  planificación, path planning, obstáculos, mapas.
```

No copiar código viejo sin revisar compatibilidad con el TP Final.

## Git y limpieza

No commitear:

```text
build/
install/
log/
__pycache__/
*.pyc
rosbags pesados
videos pesados
archivos temporales
```

Antes de commit:

```bash
git status
```

Si se generó build local:

```bash
rm -rf build install log
```

## Criterio de calidad

Cada cambio debe dejar claro:

```text
qué hace;
por qué se hizo así;
cómo se prueba;
qué parte del TP afecta;
qué queda pendiente.
```

Preferir cambios chicos, verificables y documentados antes que reescrituras grandes.

## Si se usa IA para implementar

Primero pedir:

```text
Leé AGENTS.md, docs/contexto y docs/consignas.
No modifiques código todavía.
Proponé plan de arquitectura e implementación incremental.
Listá riesgos y pruebas mínimas.
```

Después de revisar el plan, recién implementar.

## Objetivo final

No buscar un sistema perfecto de entrada. Buscar un sistema incremental, defendible y comprobable que permita explicar en informe y defensa:

```text
qué se hizo;
por qué se eligió;
qué alternativas se descartaron;
qué se validó;
qué falló;
cómo se mejoraría.
```
