# Paquetes y carpetas

## Paquetes ROS

| Path | Descripcion |
| --- | --- |
| `src/maze_slam` | Parte A. FastSLAM y launches de SLAM en simulacion/TB4. |
| `src/maze_nav` | Parte B. Publicador de mapa, localizador MCL y navegador. |
| `src/maze_perception` | Parte C. Detector visual del cono rojo. |
| `src/maze_mission` | Parte C. Maquina de estados de mision. |
| `src/turtlebot3_custom_simulation` | Mundos y recursos de simulacion TurtleBot3. |

## Configuracion

| Path | Uso |
| --- | --- |
| `config/parte_c/sim.yaml` | Perfil de Parte C para simulacion. |
| `config/parte_c/real.yaml` | Perfil principal para TurtleBot4 real `tb4_0`. |
| `config/parte_c/real_tb4_1.yaml` | Perfil alternativo para TurtleBot4 `tb4_1`. |
| `config/parte_c/waypoints_lab_tb4_0.yaml` | Waypoints sobre el mapa real de laboratorio. |

## Datos versionados

| Path | Uso |
| --- | --- |
| `maps/casa_slam.*` | Mapa de simulacion/casa. |
| `maps/maze_slam.*` | Mapa de laberinto base. |
| `maps/laberinto_lab_20260702.*` | Mapa real usado para Parte B/C en laboratorio. |
| `rviz/` | Configuraciones RViz auxiliares. |

## Scripts

| Path | Uso |
| --- | --- |
| `shs/` | Wrappers operativos para build, sim, TB4, RViz y e-stop. |
| `scripts/` | Herramientas de calibracion, smoke tests, evidencia y replay. |

Los rosbags y videos grandes estan ignorados por git para mantener liviano el
repositorio. Si existen localmente, se tratan como insumos de reproduccion.
