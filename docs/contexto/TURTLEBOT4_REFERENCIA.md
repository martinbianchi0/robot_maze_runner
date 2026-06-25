# TurtleBot4 — Referencia rápida para la Parte C

> Fuente principal: [TurtleBot4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/) e [iRobot Create® 3 ROS 2 API](https://iroboteducation.github.io/create3_docs/api/ros2/).
> Este documento es una síntesis pragmática para no atarse al simulador y poder portar a real.
> Resumen, no reemplazo del manual: si una sección lo dice, verificarlo igual antes del turno de laboratorio.

---

## 1. Por qué importa este doc

La Parte C corre sobre el robot físico del laboratorio.
La consigna menciona que el rosbag fue grabado con el **TurtleBot4 número 0**, y los topics que muestra (`tb4_0/scan`, `tb4_0/odom`) confirman que el robot real está configurado con namespace por robot.
Saber qué publica el robot, en qué frame y con qué namespace evita la mayoría de los "no me suscribe" del primer día.

---

## 2. Plataforma

| Componente | Modelo | Notas |
|---|---|---|
| Base móvil | **iRobot Create® 3** | publica odometría, IMU, comandos de velocidad, hazards |
| Compute | Raspberry Pi 4B 4GB | acá corren los nodos de ROS 2 del robot |
| LIDAR | **RPLIDAR A1M8** | rango ~12 m, 360° |
| Cámara (Lite) | OAK-D-Lite (IMX214 4K + estéreo OV7251) | tiene par estéreo → puede dar profundidad |
| Cámara (Standard) | OAK-D-Pro (estéreo OV9282 + IR + láser) | mejor en baja luz |
| Batería | 26 Wh Li-Ion, 14.4 V | autonomía limitada, ojo en el turno |
| Sistema de coords | mano derecha: **x adelante, y izquierda, z arriba** | estándar ROS |

Modelo del lab: a confirmar. Para la Parte C alcanza con tratarlo como TurtleBot4 (cualquier variante) más LIDAR RPLIDAR y cámara OAK-D.

---

## 3. Topics ROS 2 — lo que vamos a usar

### 3.1. Convención de namespaces

El TurtleBot4 puede correr con namespace por robot (típico cuando hay varios).
El rosbag de la cátedra confirma que el robot del lab usa **`tb4_0/`** como namespace.

Regla en el código: **topics como parámetro**, con perfiles separados.

```text
sim:
  scan_topic:    /scan
  odom_topic:    /odom            # o /calc_odom según el flujo del TP
  cmd_vel_topic: /cmd_vel
  imu_topic:     /imu

real:
  scan_topic:    /tb4_0/scan
  odom_topic:    /tb4_0/odom
  cmd_vel_topic: /tb4_0/cmd_vel
  imu_topic:     /tb4_0/imu
```

Nada hardcodeado (regla de `AGENTS.md`).

### 3.2. Topics que nos importan

| Topic (sin namespace) | Tipo | Dirección | Uso en TP |
|---|---|---|---|
| `scan` | `sensor_msgs/LaserScan` | publica (RPLIDAR) | SLAM, localización, evasión de obstáculos |
| `odom` | `nav_msgs/Odometry` | publica (Create 3) | predicción en MCL / FastSLAM |
| `imu` | `sensor_msgs/Imu` | publica (Create 3) | opcional, podría mejorar predicción |
| `cmd_vel` | `geometry_msgs/Twist` | suscribe (Create 3) | output del path follower |
| `battery_state` | `sensor_msgs/BatteryState` | publica | monitoreo durante el turno |
| `hazard_detection` | `irobot_create_msgs/HazardDetectionVector` | publica | safety, frenar si la base detecta peligro |
| `wheel_status` | `irobot_create_msgs/WheelStatus` | publica | debug |
| `dock` / `dock_status` | `irobot_create_msgs/DockStatus` | publica | saber si está dockeado |
| `color/preview/image` | `sensor_msgs/Image` | publica (OAK-D) | detección del cono rojo |

Para la Parte C **lo crítico es `scan`, `odom`, `cmd_vel` y la imagen de la cámara**.

### 3.3. QoS — atención

El manual no lista QoS explícito de cada topic, pero por experiencia con Create 3 y RPLIDAR:

- Sensores de alto rate (LIDAR, IMU, cámara) pueden venir con **`BEST_EFFORT`**.
- Si el suscriptor pide `RELIABLE` por defecto, **no se suscribe en silencio** (no hay error).
- Probar el QoS de cada topic crítico con `ros2 topic info -v <topic>` el primer día y dejar el suscriptor compatible.

Esto es exactamente lo que decía `docs/contexto/PARTE_C_ROBOT_REAL.md` sobre no hardcodear una sola configuración.

---

## 4. Frames TF

Estructura típica que esperamos del robot:

```text
map  (provisto por nuestra localización)
 └── odom              (provisto por Create 3)
      └── base_footprint   (proyección 2D del robot)
           └── base_link    (centro de rotación)
                ├── rplidar_link  (frame del LIDAR)
                ├── imu_link
                └── oakd_link     (cámara)
```

Notas prácticas:

- `base_link` y `base_footprint` ambos vienen del Create 3 (`odom → base_footprint` y `odom → base_link`). Usar `base_footprint` para planificación 2D.
- El frame del LIDAR (`rplidar_link` o nombre análogo) puede tener orientación distinta a la del simulado: **cuidar ángulo cero y sentido de giro** al procesar el scan.
- `map → odom` lo publica **nuestra localización** (MCL de Parte B), no el robot.

Verificar el día del lab con:

```bash
ros2 run tf2_tools view_frames.py
```

---

## 5. Servicios y acciones útiles del Create 3

No los necesitamos para SLAM/navegación, pero son útiles para operación segura.

- `/e_stop` — emergency stop, parar el robot.
- `/dock` / `/undock` — acciones para dockeo automático.
- `/drive_distance`, `/rotate_angle` — para tests aislados sin nuestro controlador.
- Parámetros configurables: `max_speed`, `min_speed`, safety overrides, brillo de LEDs.

---

## 6. Networking — punto que más rompe el primer día

Dos modos:

- **Simple Discovery** (default): todos en la misma WiFi, multicast; con 1-2 robots alcanza.
- **Discovery Server** (recomendado por iRobot): el Create 3 no necesita WiFi, sólo la RPi; escala mejor.

El día del lab:

1. confirmar en qué modo está el robot;
2. configurar `ROS_DOMAIN_ID` igual en PC y robot;
3. probar `ros2 topic list` y `ros2 topic echo /tb4_0/scan` ANTES de lanzar nuestros nodos.

---

## 7. Diferencias sim ↔ real a tener en cuenta

| Aspecto | Simulación (TurtleBot3 en Gazebo) | Real (TurtleBot4) |
|---|---|---|
| Topics | `/scan`, `/odom`, `/calc_odom`, `/cmd_vel` | `/tb4_0/scan`, `/tb4_0/odom`, `/tb4_0/cmd_vel` |
| Ground truth | hay `/odom` perfecto en sim | NO hay `/odom` "perfecto" — todo es ruidoso |
| LIDAR | ideal | RPLIDAR A1, ruido, intensidades, frame distinto |
| Cámara | a veces no en sim | OAK-D real, latencia, exposición variable |
| Dinámica | instantánea | inercia, demora al frenar, overshoot al girar |
| QoS | default suele bastar | algunos topics piden `BEST_EFFORT` |

Refuerza lo que ya documentamos en `PARTE_C_PLAN_ROBOT_REAL.md`: **velocidades conservadoras, perfiles sim/real, validar con rosbag antes del turno**.

---

## 8. Checklist primer minuto en el laboratorio

Antes de tocar nada de nuestros nodos:

1. `ros2 topic list` — confirmar que aparecen `/tb4_0/scan`, `/tb4_0/odom`, `/tb4_0/cmd_vel`, `/tb4_0/imu`, imagen de cámara.
2. `ros2 topic info -v /tb4_0/scan` — confirmar QoS del LIDAR.
3. `ros2 topic echo /tb4_0/scan --once` — confirmar que llega data.
4. `ros2 topic echo /tb4_0/odom --once` — confirmar odometría.
5. `ros2 run tf2_tools view_frames.py` — confirmar TF tree real.
6. `ros2 topic pub --once /tb4_0/cmd_vel geometry_msgs/Twist '{linear:{x:0.05}}'` — confirmar que el robot acepta comandos (velocidad baja).

Recién entonces lanzar nuestros nodos con el perfil "real".

---

## 9. Qué cambia esto en los planes A/B/C

- **Parte A**: nada estructural, pero el rosbag de cátedra es de TurtleBot4 → confirmamos que los nombres `tb4_0/scan` y `tb4_0/odom` ya están en el flujo.
- **Parte B**: refuerza la decisión de tener perfiles sim/real para topics, frames y QoS.
- **Parte C**: **la cámara OAK-D puede dar profundidad** (estéreo). Esto matiza el punto que marcó Martín en la PR #3:
  - Si conseguimos el topic de depth de la OAK-D alineado con el color, una detección de cono rojo se convierte en una coordenada del mundo razonable (no solo bearing).
  - Si no, queda como bearing y se aplican las estrategias ya planteadas (tamaño aparente, goals intermedios validados contra costmap).
  - Verificar qué topic de depth publica el robot del lab antes de comprometernos a una u otra estrategia.

---

## 10. Verificaciones pendientes (hacer en el lab)

- [ ] Confirmar modelo exacto (Lite vs Standard) del robot del lab.
- [ ] Confirmar namespace exacto (¿siempre `tb4_0/`? ¿hay otros?).
- [ ] Confirmar QoS de `scan`, `odom`, `imu`, imagen de cámara.
- [ ] Confirmar nombres exactos de los frames TF.
- [ ] Confirmar topic(s) de la OAK-D (color, depth, point cloud) y si están alineados.
- [ ] Confirmar modo de discovery (Simple vs Discovery Server) y `ROS_DOMAIN_ID`.
