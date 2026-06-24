# Análisis de decisión de arquitectura del TP Final

> Documento de trabajo para decidir por dónde encarar el TP Final.  
> No es una decisión cerrada: es una recomendación inicial basada en consignas, tutoriales, teóricas 12–16, transcripciones y experiencia previa con TP4, TP5 y TP6.

---

## 1. Estado actual del proyecto

Ya existe una base de repositorio/workspace para el TP Final:

```text
tp_final_ws/
├── src/
│   └── turtlebot3_custom_simulation/
├── docs/
│   ├── consignas/
│   └── contexto/
├── rviz/
├── maps/
├── results/
├── scripts/
├── rosbags/
└── .gitignore
```

También se verificó que la simulación base actualizada de la cátedra compila y publica los tópicos importantes:

```text
/scan
/odom
/calc_odom
```

Esto es clave porque `/calc_odom` aparece explícitamente como insumo importante para trabajar con odometría calculada en la simulación del TP Final.

---

## 2. Resumen de la consigna

El TP Final está organizado en tres partes:

```text
Parte A - SLAM
Parte B - Movimiento automático
Parte C - Autonomía completa real
```

La Parte A define el mapa que después condiciona Parte B y Parte C.

La Parte B usa el mapa para localizarse, planificar, seguir una trayectoria, evitar obstáculos y replantear el camino si hace falta.

La Parte C lleva el sistema al robot real, con búsqueda de un cono rojo en un laberinto.

---

## 3. Decisión central: Parte A

La Parte A permite tres caminos.

### Opción 1 - Grid-Based FastSLAM

**Entorno:** Gazebo.  
**Sensor:** LIDAR.  
**Resultado principal:** mapa grillado / mapa de ocupación.

La idea es usar conceptos de FastSLAM, pero con mapas de grilla. Cada partícula representa una hipótesis de trayectoria y mantiene su propio mapa. La probabilidad/peso de cada partícula depende de qué tan compatibles son sus observaciones con su propio mapa.

**Fortalezas:**

- Produce directamente un mapa de ocupación, que es justo lo que después necesita la navegación.
- Usa Gazebo y LIDAR, no requiere cámara ni RosBag como base inicial.
- Conecta muy bien con lo visto en TP5 y con las teóricas de mapas de ocupación, FastSLAM y SLAM basado en grillas.
- Permite avanzar por etapas: primero mapeo con poses conocidas, luego partículas, luego mejoras.
- Es la opción más coherente si queremos una base funcional y defendible.

**Riesgos:**

- Computacionalmente puede ser pesado porque cada partícula lleva su propio mapa.
- Con muchas partículas y mapas grandes puede volverse lento.
- Sin scan matching o alguna mejora de odometría, el mapa puede deformarse mucho, especialmente en giros.
- Hay que decidir resolución, tamaño del mapa, cantidad de partículas, criterios de remuestreo, umbrales de ocupación y cómo elegir el mapa final.

**Punto clave de implementación:**

No conviene arrancar implementando “todo FastSLAM” de una. Conviene validar por etapas:

```text
Etapa 0: simulación + RViz + teleop + confirmación de /scan, /odom, /calc_odom.
Etapa 1: occupancy grid mapping con pose conocida usando /calc_odom.
Etapa 2: guardar mapa generado en maps/.
Etapa 3: agregar partículas con pocos mapas.
Etapa 4: usar mejor partícula como mapa final.
Etapa 5: evaluar si hace falta scan matching o simplificación.
```

**Mi inclinación:** es la mejor opción inicial.

---

### Opción 2 - Features con LIDAR

**Entorno:** Gazebo.  
**Sensor:** LIDAR.  
**Resultado:** mapa grillado + landmarks/features, según implementación.

La idea es extraer features del LIDAR, como esquinas, segmentos, columnas o patrones geométricos, y después usar un algoritmo de SLAM basado en features, como EKF SLAM, Graph SLAM o SEIF SLAM.

**Fortalezas:**

- Puede reutilizar conceptos de TP4 y TP5: EKF, landmarks, covarianzas, visualización en RViz.
- Es más liviana que llevar un mapa de grilla por partícula si las features están bien definidas.
- Es defendible teóricamente: se puede explicar por landmarks, asociación de datos y covarianzas.

**Riesgos:**

- El cuello de botella real es extraer features robustas del LIDAR.
- Si las features no son estables en el tiempo, todo lo demás se vuelve frágil.
- La asociación de datos puede romper el SLAM si se asocian mal landmarks.
- Puede terminar siendo más difícil que Opción 1 porque el problema no es solo el filtro, sino detectar landmarks consistentes.

**Punto clave de implementación:**

Antes de decidir esta opción habría que hacer una prueba mínima:

```text
Tomar /scan en custom_casa.launch.py.
Extraer features simples.
Visualizarlas en RViz.
Mover el robot y verificar si las mismas features aparecen de manera estable.
```

Si eso no funciona rápido, descartarla.

**Mi inclinación:** no la elegiría como camino principal salvo que el grupo quiera apostar fuerte a features. La usaría como alternativa o extensión.

---

### Opción 3 - Cámara / ArUco / RosBag / Graph SLAM

**Entorno principal Parte A:** RosBag / TurtleBot4 real grabado.  
**Sensores:** cámara + LIDAR.  
**Algoritmo esperado:** Graph SLAM.  
**Resultado:** mapa + landmarks/tags.

**Fortalezas:**

- Es la opción más cercana al robot real y a Parte C.
- ArUco/tags pueden dar landmarks más identificables que features geométricas del LIDAR.
- Graph SLAM tiene una defensa conceptual fuerte: nodos, restricciones, odometría, cierres de lazo y optimización.
- Permite explicar bien front-end/back-end y loop closure.

**Riesgos:**

- Es la opción más ambiciosa.
- Implica RosBag, cámara, detección de tags, calibración/frames y Graph SLAM.
- Después hay que adaptar la lógica a simulación/Gazebo para Parte B con landmarks simulados o sensor virtual.
- Puede volverse un proyecto paralelo de visión + SLAM + sim-to-real.

**Punto clave de implementación:**

No conviene elegirla sin antes confirmar:

```text
El RosBag abre bien.
Los tópicos de cámara/LIDAR/TF están claros.
Se detectan tags o landmarks de manera confiable.
Se puede construir una restricción útil para Graph SLAM.
```

**Mi inclinación:** no la elegiría como primera opción salvo que el equipo tenga muchas ganas de cámara/GraphSLAM y tiempo para pelearse con RosBag/TurtleBot4.

---

## 4. Recomendación principal

Mi recomendación preliminar es:

```text
Elegir Opción 1 - Grid-Based FastSLAM
con validación incremental.
```

Pero con una aclaración importante: no arrancar directamente por un Grid-Based FastSLAM completo. Arrancar por el caso más controlado:

```text
mapeo con poses conocidas usando /calc_odom + /scan
```

Si eso produce un mapa razonable, recién ahí se agregan partículas.

---

## 5. Por qué me inclino por Opción 1

### 5.1. Produce el insumo más útil para Parte B

Parte B necesita navegar sobre un mapa. Un mapa de ocupación es directamente usable para:

```text
- inflar obstáculos;
- planificar con A*, Dijkstra, Theta* o RRT;
- evaluar colisiones;
- generar un path;
- guardar mapa en formato compatible con navegación.
```

En cambio, un mapa solo de landmarks no alcanza para saber transitabilidad. Te ayuda a localizarte, pero no dice por dónde se puede pasar.

### 5.2. Evita depender de features frágiles

La Opción 2 suena elegante, pero el problema difícil es que las features del LIDAR sean robustas. Si hoy una esquina aparece como una feature y mañana se parte en dos, el SLAM se complica.

### 5.3. Evita depender demasiado de cámara/RosBag

La Opción 3 puede salir muy bien, pero tiene muchas piezas móviles. Para un primer sistema funcional, cámara + ArUco + GraphSLAM + adaptación a Gazebo parece más riesgoso.

### 5.4. Conecta con lo que ya hicimos

Ya se trabajó en:

```text
TP4: EKF, odometría, belief, paths.
TP5: SLAM, landmarks, covarianzas, FastSLAM.
TP6: planificación/path planning.
TP Final 0: control reactivo, máquina de estados, sim-to-real.
```

La Opción 1 aprovecha mejor esa base sin forzar una arquitectura demasiado nueva.

---

## 6. Plan recomendado por etapas

### Etapa 1 - Base operativa

Objetivo: confirmar que el workspace está bien.

```text
- custom_casa.launch.py corre.
- /scan aparece.
- /odom aparece.
- /calc_odom aparece.
- teleop mueve el robot.
- RViz muestra mapa, scan, odom y calc_odom.
```

### Etapa 2 - Mapeo con poses conocidas

Objetivo: construir un mapa de ocupación usando pose conocida/casi conocida.

Inputs:

```text
/scan
/calc_odom
```

Outputs:

```text
/map_generated o tópico equivalente
maps/sim/casa_map.pgm
maps/sim/casa_map.yaml
results/capturas_mapeo/
```

Técnica:

```text
- grilla 2D;
- log-odds;
- modelo inverso del sensor;
- ray tracing / Bresenham;
- celdas libres hasta impacto;
- celda ocupada cerca del impacto;
- unknown = 0.5.
```

### Etapa 3 - Evaluación del mapa

Preguntas:

```text
- Las paredes aparecen en lugar razonable?
- Hay mucha pared fantasma?
- Los giros rompen todo?
- La resolución es suficiente para navegación?
- El mapa se puede guardar y cargar?
```

Si falla acá, no tiene sentido agregar partículas todavía.

### Etapa 4 - Partículas

Objetivo: pasar de mapeo con pose conocida a Grid-Based FastSLAM.

```text
- N partículas bajo al inicio.
- Cada partícula mantiene pose + mapa.
- Propagación por odometría.
- Peso por likelihood de scan contra mapa de partícula.
- Resampling.
- Mapa final = mapa de mejor partícula.
```

### Etapa 5 - Integración con Parte B

Objetivo: que el mapa generado sirva para navegar.

```text
- limpiar/inflar mapa;
- exportar mapa;
- usarlo para localización;
- planificar trayectoria;
- seguir path;
- evitar obstáculos no mapeados.
```

---

## 7. Decisión preliminar por parte

### Parte A

```text
Recomendación: Opción 1 - Grid-Based FastSLAM.
Estrategia: validación incremental.
Primera prueba: occupancy grid mapping con /calc_odom.
```

### Parte B

```text
Recomendación: mapa inflado + A* o Theta* + Pure Pursuit + máquina de estados + replanning simple.
```

Motivo:

- A* es directo sobre grilla.
- Theta* puede generar caminos más suaves si se implementa bien.
- Pure Pursuit ya fue explicado en tutorial y es razonable para path following.
- Máquina de estados ayuda a ordenar: esperar initial pose, esperar goal, planificar, seguir path, evitar obstáculo, replantear, finalizar.

### Parte C

```text
Recomendación: diseñar desde el principio con parámetros sim/real.
```

Separar:

```text
topics_sim:
  scan: /scan
  odom: /odom o /calc_odom
  cmd_vel: /cmd_vel

topics_real:
  scan: /tb4_0/scan
  odom: /tb4_0/odom
  cmd_vel: /tb4_0/cmd_vel
```

Y preparar adaptaciones:

```text
- QoS para TurtleBot4;
- rotación/transformación del LIDAR real;
- filtrado de intensidades;
- velocidades más bajas;
- tolerancias más grandes;
- detección de cono rojo.
```

---

## 8. Riesgos principales

### Riesgo 1 - Subestimar el costo de Grid-Based FastSLAM

Mitigación:

```text
No empezar con muchas partículas.
No empezar con mapa enorme.
Primero validar mapeo con /calc_odom.
Usar baja resolución inicial.
Optimizar solo si el prototipo funciona.
```

### Riesgo 2 - Hacer código demasiado acoplado a simulación

Mitigación:

```text
Todos los topics deben ser parámetros.
Separar lógica de sensores de lógica de decisión.
Tener perfiles sim/real.
```

### Riesgo 3 - No llegar a Parte B/C por querer hacer Parte A perfecta

Mitigación:

```text
Definir un mapa suficientemente bueno, no perfecto.
Guardar decisiones y limitaciones.
Priorizar sistema completo funcional.
```

### Riesgo 4 - No documentar decisiones

Mitigación:

```text
Mantener docs/decisiones/DECISIONES_ABIERTAS.md.
Actualizarlo después de cada decisión importante.
```

---

## 9. Criterio para confirmar la arquitectura

Confirmaría Opción 1 si en la primera prueba se logra:

```text
- generar un mapa reconocible de custom_casa;
- guardar ese mapa;
- verlo en RViz;
- usarlo para planificar al menos un camino simple;
- mantener tiempo de ejecución razonable.
```

La descartaría o simplificaría si:

```text
- el mapa se deforma mucho incluso con /calc_odom;
- el algoritmo queda demasiado lento;
- no se logra exportar un mapa usable;
- el equipo queda trabado antes de llegar a Parte B.
```

---

## 10. Conclusión

La mejor estrategia no es elegir el algoritmo más sofisticado, sino el camino con mejor relación entre:

```text
- dificultad;
- posibilidad de implementación real;
- conexión con Parte B;
- conexión con Parte C;
- claridad para informe/defensa;
- reutilización de lo ya trabajado.
```

Con lo visto hasta ahora, la recomendación es:

```text
Opción 1 - Grid-Based FastSLAM,
empezando por mapeo con poses conocidas usando /calc_odom,
y avanzando incrementalmente hacia partículas.
```

La decisión final debería tomarse después de la primera prueba técnica de mapeo.
