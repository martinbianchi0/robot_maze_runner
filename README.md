# Robot Maze Runner

Repositorio del TP final de Robotica: SLAM, navegacion autonoma y mision de
busqueda de un cono rojo con TurtleBot.

El entregable principal es el informe final:

- [docs/informe/Robot_Maze.pdf](docs/informe/Robot_Maze.pdf)

## Que hace

El sistema resuelve el recorrido autonomo de un robot en un laberinto:

1. Construye un mapa de ocupacion del entorno.
2. Localiza al robot sobre ese mapa.
3. Planifica y sigue caminos hasta objetivos.
4. Detecta un cono rojo con vision.
5. Ejecuta una mision end-to-end: patrulla, detecta el cono, valida el objetivo y
   navega hacia el.

El codigo esta separado por responsabilidades. La Parte A produce mapas, la
Parte B usa esos mapas para navegar, y la Parte C integra navegacion,
percepcion y logica de mision en TurtleBot4 real.

## Paquetes principales

| Paquete | Rol |
| --- | --- |
| `maze_slam` | Grid-Based FastSLAM. Publica mapa, pose estimada y particulas. |
| `maze_nav` | Mapa estatico, MCL, A*, seguimiento de path, FSM de navegacion y recovery ante obstaculos. |
| `maze_perception` | Segmentacion HSV y deteccion del cono rojo desde la camara. |
| `maze_mission` | FSM de Parte C: localizacion, patrulla, deteccion, validacion del cono, goals y fallback `SERVO_TO_CONE`. |
| `turtlebot3_custom_simulation` | Mundos y launches de simulacion TurtleBot3 usados para Parte A/B. |

Documentacion de arquitectura:

- [docs/arquitectura/arquitectura.md](docs/arquitectura/arquitectura.md)
- [docs/arquitectura/paquetes.md](docs/arquitectura/paquetes.md)

## Como correr

Construir el workspace:

```bash
./shs/build.sh
```

Parte A en simulacion TurtleBot3:

```bash
./shs/casa.sh
./shs/slam_casa.sh
./shs/teleop.sh
./shs/save_map.sh
```

Parte B en simulacion TurtleBot3:

```bash
./shs/casa.sh obs
./shs/nav.sh
```

Parte A/B en TurtleBot4 real:

```bash
export ROS_DOMAIN_ID=<dominio_del_TB4>
./shs/mapear_tb4.sh --ns tb4_0
./shs/navegar_tb4.sh --ns tb4_0 --map maps/laberinto_lab_20260702.yaml
```

Parte C end-to-end en TurtleBot4 real:

```bash
export ROS_DOMAIN_ID=<dominio_del_TB4>
./shs/parte_c_tb4.sh --ns tb4_0 --map maps/laberinto_lab_20260702.yaml
```

Mas detalle:

- [docs/uso/instalacion.md](docs/uso/instalacion.md)
- [docs/uso/ejecucion.md](docs/uso/ejecucion.md)
- [docs/uso/comandos.md](docs/uso/comandos.md)

## Evidencia y validacion

Validaciones documentadas en esta entrega:

- Simulacion TurtleBot3: SLAM y navegacion en casa/laberinto, con mapas en
  `maps/` y figuras en [docs/informe/figures](docs/informe/figures).
- Rosbags TurtleBot4: replay/depuracion soportados por scripts en `scripts/` y
  wrappers `--bag` en `shs/`; los rosbags pesados no se versionan por defecto.
- TurtleBot4 real en laboratorio: mapa `maps/laberinto_lab_20260702.{yaml,pgm}`,
  figuras y logs en [docs/laboratorio/evidencia](docs/laboratorio/evidencia).
- Parte C end-to-end con cono rojo: corridas de busqueda, navegacion al cono y
  confirmacion final.
- Fallback `SERVO_TO_CONE`: validado cuando el cono queda por debajo del plano
  del LIDAR y la estimacion por rango cae sobre pared.

La evidencia de laboratorio esta resumida en:

- [docs/laboratorio/README.md](docs/laboratorio/README.md)
- [docs/laboratorio/evidencia/parte_c_servoing.md](docs/laboratorio/evidencia/parte_c_servoing.md)

## Estructura de documentacion

```text
docs/
  informe/        PDF final y figuras usadas por la entrega
  laboratorio/    evidencia del robot real y notas de validacion
  uso/            instalacion, ejecucion y comandos
  arquitectura/   vision general y paquetes
  consignas/      enunciados originales de la catedra
  archive/        borradores y notas historicas no principales
```

Los documentos en `docs/archive/` se conservan por trazabilidad y no deben usarse
como documentacion principal de la entrega.
