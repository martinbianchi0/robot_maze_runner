# PARTE B - NAVEGACIÓN AUTÓNOMA

Versión de contexto: v2  
Base usada: consignas oficiales, Tutorial 10, Tutorial 12, Tutorial 13 y teóricas 12-16.

Este documento define cómo pensar la Parte B del Trabajo Final: navegación autónoma a partir del mapa generado en Parte A.

---

# 1. Objetivo de Parte B

Con el mapa ya generado, el robot debe poder:

1. recibir una estimación de pose inicial;
2. recibir una pose objetivo;
3. localizarse en el mapa;
4. planificar un camino válido;
5. seguir el camino automáticamente;
6. evitar obstáculos no mapeados;
7. replantear si el camino queda bloqueado;
8. llegar al objetivo con posición y orientación razonables.

Parte B no es solamente path planning. Es una integración de localización, planning, control y máquina de estados.

---

# 2. Input de Parte B

El input principal depende de Parte A.

## 2.1. Si Parte A produce Resultado V1

Resultado V1:

- mapa grillado del entorno.

Parte B usa:

- mapa de ocupación;
- LIDAR;
- odometría;
- localización probabilística;
- planificación sobre grilla;
- path following;
- obstacle avoidance.

## 2.2. Si Parte A produce Resultado V2

Resultado V2:

- mapa grillado;
- landmarks/features.

Parte B puede usar:

- mapa de ocupación para planning;
- landmarks para mejorar localización;
- LIDAR/cámara/sensor virtual de landmarks;
- planificación y control igual que en V1.

---

# 3. Mapa para navegación

El mapa de SLAM no necesariamente está listo para planificar directamente.

Hay que transformarlo en un mapa navegable.

## 3.1. De probabilidad a grilla navegable

Un occupancy grid puede tener:

- ocupado;
- libre;
- desconocido.

Para planning hay que decidir:

- umbral de ocupado;
- umbral de libre;
- qué hacer con celdas desconocidas.

Ejemplo conceptual:

- p ocupada alta: obstáculo;
- p ocupada baja: libre;
- p cerca de 0.5: desconocido.

## 3.2. Inflado de obstáculos

No alcanza con planificar por celdas libres exactas.

El robot tiene tamaño físico, error de localización y error de control. Entonces se debe inflar obstáculos.

Decisión necesaria:

- radio de seguridad;
- margen extra para el robot real;
- si el inflado es fijo o configurable.

Esto es clave para evitar caminos que en el mapa parecen posibles, pero el robot real no puede atravesar.

## 3.3. Espacio desconocido

Hay que decidir si el planner puede atravesar espacio desconocido.

Para una entrega segura, lo recomendable es:

- en simulación, evitar desconocido salvo que sea necesario;
- para robot real, tratar desconocido como riesgoso;
- documentar la decisión.

---

# 4. Localización

Parte B requiere localización probabilística. La localización puede usar:

- mapa generado;
- `/scan`;
- `/odom` o `/calc_odom`;
- landmarks si existen.

Opciones posibles:

- particle filter / MCL;
- EKF;
- UKF;
- localización por landmarks;
- localización híbrida.

## 4.1. MCL / filtro de partículas

Tiene sentido si se trabaja con mapa grillado.

Cada partícula representa una hipótesis de pose. El peso sale de comparar el scan real con el mapa.

Ventajas:

- robusto a no linealidades;
- compatible con mapas de grilla;
- conceptualmente conectado con TP3/TP5.

Riesgos:

- tuning de cantidad de partículas;
- costo de likelihood;
- ambigüedad en pasillos/simetrías.

## 4.2. Localización con landmarks

Tiene sentido si Parte A produce landmarks.

Puede usar EKF u otro filtro.

Ventajas:

- landmarks ayudan a corregir odometría;
- útil si el mapa de features es estable.

Riesgos:

- landmarks tapados;
- mala asociación de datos;
- no alcanza para evitar paredes si no hay occupancy grid.

---

# 5. Path planning

El planner toma:

- mapa navegable;
- pose inicial;
- pose objetivo.

Devuelve:

- una secuencia de puntos;
- una trayectoria discreta;
- o un camino suavizado.

## 5.1. Algoritmos candidatos

### A*

Buena opción base.

Ventajas:

- simple;
- eficiente;
- fácil de defender;
- usa heurística;
- funciona bien en grillas.

### Dijkstra

Más general pero explora más.

Puede servir como baseline, pero A* suele ser mejor para objetivo único.

### Theta*

Similar a A*, pero permite caminos más rectos usando línea de visión.

Puede ser muy útil en grilla porque evita trayectorias con escalones innecesarios.

### D* / D* Lite

Útil para replanning incremental cuando aparecen obstáculos nuevos.

Puede ser más complejo, pero conceptualmente encaja con Parte B.

### RRT / RRT*

Útil en espacios continuos o de alta dimensión.

Para un mapa grillado 2D simple, puede ser más difícil de tunear que A*/Theta*.

## 5.2. Recomendación preliminar

Para arrancar:

1. A* sobre mapa inflado.
2. Si el camino queda muy cuadriculado, evaluar Theta*.
3. Replanning simple si aparece obstáculo.
4. No arrancar con RRT* salvo que haya una razón clara.

---

# 6. Path following

El robot debe seguir el camino generado.

Tutorial 12 introduce Pure Pursuit como controlador práctico.

## 6.1. Pure Pursuit

Idea:

1. Tener una trayectoria de puntos.
2. Elegir un punto objetivo adelante del robot.
3. Calcular curvatura hacia ese punto.
4. Convertir curvatura en velocidad angular.
5. Enviar `cmd_vel`.

Relación conceptual:

- `omega = v * gamma`
- donde `gamma` es curvatura.

## 6.2. Decisiones de Pure Pursuit

Hay que definir:

- lookahead distance;
- velocidad lineal;
- velocidad angular máxima;
- cuándo avanzar al siguiente punto;
- cuándo considerar que llegó al objetivo;
- cómo corregir orientación final.

## 6.3. Riesgos

- lookahead muy chico: zigzag/inestabilidad.
- lookahead muy grande: corta curvas y puede acercarse a paredes.
- velocidad alta: peor localización y más overshoot.
- caminos muy pegados a obstáculos: riesgo de choque.

---

# 7. Obstacle detection y avoidance

La Parte B debe manejar obstáculos estáticos o dinámicos que no estaban en el mapa.

Esto implica:

1. usar LIDAR en tiempo real;
2. detectar obstáculo sobre el camino;
3. frenar o rodear;
4. replantear si el camino queda bloqueado.

## 7.1. Estrategia simple

Una estrategia defendible:

- si hay obstáculo demasiado cerca adelante:
  - frenar;
  - marcar celdas como temporalmente ocupadas;
  - replanificar;
  - seguir nuevo camino.

## 7.2. Lo que no hay que hacer

No asumir que si el LIDAR ve algo detrás de una abertura el robot puede atravesar una pared.

Hay que combinar:

- mapa estático;
- obstáculos inflados;
- LIDAR actual;
- control de seguridad.

---

# 8. Máquina de estados

Parte B debería estar organizada con máquina de estados.

Estados posibles:

1. `WAITING_FOR_MAP`
2. `WAITING_FOR_INITIAL_POSE`
3. `LOCALIZING`
4. `WAITING_FOR_GOAL`
5. `PLANNING`
6. `FOLLOWING_PATH`
7. `AVOIDING_OBSTACLE`
8. `REPLANNING`
9. `GOAL_REACHED`
10. `FAILURE`

La máquina de estados permite que el sistema sea explicable en el informe y defensa.

---

# 9. Topics ROS esperados

Depende de la implementación, pero conviene parametrizar:

## Inputs

- `/map` o `/slam_map`
- `/scan`
- `/odom`
- `/calc_odom`
- `/initialpose`
- `/goal_pose`
- `/clicked_point` opcional

## Outputs

- `/cmd_vel`
- `/planned_path`
- `/robot_path`
- `/belief`
- `/particles`
- `/local_costmap` opcional
- `/state` opcional
- markers de debug

---

# 10. RViz

RViz debe mostrar como mínimo:

- mapa;
- LIDAR;
- robot;
- pose estimada;
- path planeado;
- path ejecutado;
- obstáculos;
- goal;
- initial pose;
- particles si se usa filtro de partículas.

Config base:

- `rviz/tp_final_base.rviz`

Config futura:

- `rviz/tp_final_nav.rviz`

---

# 11. Prueba mínima de Parte B

Antes de integrar todo:

## Test B1 - Planner offline

- Cargar mapa.
- Elegir start/goal manualmente.
- Generar path.
- Visualizar en RViz.
- Verificar que no atraviesa paredes.

## Test B2 - Pure Pursuit con path simple

- Usar camino predefinido.
- Robot sigue puntos sin planificación.
- Ajustar velocidades/lookahead.

## Test B3 - Planner + follower

- Generar path desde start/goal.
- Seguirlo.
- Ver si llega.

## Test B4 - Obstacle avoidance

- Agregar obstáculo no mapeado.
- Detectar bloqueo.
- Frenar/replanificar.

## Test B5 - Sistema completo

- Initial pose.
- Goal.
- Localización.
- Planning.
- Following.
- Replanning.
- Llegada.

---

# 12. Decisiones abiertas

- Qué mapa usar para navegación.
- Qué umbrales de ocupación usar.
- Qué hacer con desconocido.
- Cuánto inflar obstáculos.
- Qué algoritmo de planning usar.
- Pure Pursuit vs otro controlador.
- Cómo detectar llegada.
- Cómo corregir orientación final.
- Cómo manejar obstáculos no mapeados.
- Qué filtro usar para localización.
- Cómo parametrizar sim vs real.

---

# 13. Recomendación preliminar

Para una primera arquitectura defendible:

- mapa grillado generado por Parte A;
- conversión a mapa navegable;
- inflado de obstáculos;
- A* o Theta*;
- Pure Pursuit;
- obstacle detection con LIDAR;
- replanning simple;
- máquina de estados.

No conviene arrancar con una arquitectura demasiado sofisticada si todavía no está resuelta Parte A.

---

# 14. Notas para Codex

Cuando Codex trabaje en Parte B:

- No asumir que el mapa de SLAM ya es navegable.
- Separar módulos:
  - localización;
  - planning;
  - path following;
  - obstacle avoidance;
  - state machine.
- No mezclar planner con controlador en un solo script gigante.
- Mantener parámetros configurables.
- Publicar visualizaciones en RViz.
- Priorizar una prueba mínima antes de integrar todo.
