# Flujo del TP Final

Este documento pasa a texto el diagrama general del Trabajo Práctico Final de I402 - Principios de la Robótica Autónoma.

El objetivo es conservar la lógica del flowchart original, pero en un formato más fácil de leer, versionar y usar como contexto para el grupo y para Codex.

> Nota importante: este archivo no toma decisiones de arquitectura. Resume el flujo y separa claramente las alternativas disponibles para evitar mezclar algoritmos de ramas distintas.

---

# 1. Vista general del TP Final

El TP Final se organiza en tres partes conectadas entre sí:

1. **Parte A - SLAM / Percepción estructural**
   - Se construye un mapa del entorno y se estima la pose del robot.
   - El resultado de esta parte alimenta directamente a la Parte B.

2. **Parte B - Movimiento automático / Navegación autónoma**
   - Con el mapa generado en la Parte A, el robot debe localizarse, planificar y seguir trayectorias hacia objetivos seleccionados por el usuario.
   - Debe poder manejar replanning, obstáculos y orientación final.

3. **Parte C - Autonomía completa real**
   - Se lleva el sistema al robot real.
   - El robot debe explorar el laberinto, detectar conos rojos, ignorar distractores y planificar caminos válidos hasta el objetivo.

La relación general es:

```text
Parte A: SLAM / mapa
        ↓
Parte B: localización + planificación + control en simulación
        ↓
Parte C: robot real + búsqueda autónoma de conos rojos
```

---

# 2. Parte A - SLAM

## 2.1 Objetivo general

En la Parte A se debe implementar un sistema de SLAM que use datos de sensores y odometría para construir una representación del entorno mientras estima la pose del robot.

La Parte A puede resolverse por tres caminos distintos. La elección de este camino es una decisión crítica porque condiciona cómo se implementa la Parte B.

Los posibles resultados de la Parte A son:

- **Resultado V1:** mapa grillado del entorno.
- **Resultado V2:** mapa grillado del entorno + landmarks.

---

# 3. Parte A - Opción 1: Grid-Based FastSLAM

## 3.1 Entorno de trabajo

- Gazebo.
- TurtleBot3 simulado.

## 3.2 Sensores

- LIDAR.
- Odometría publicada por el entorno de simulación.

## 3.3 Algoritmos y herramientas de esta rama

Esta rama corresponde a SLAM basado en grillas de ocupación y filtros de partículas.

Elementos principales:

- Modelo de odometría.
- Filtro de partículas.
- Likelihood Fields.
- Occupancy Grid.
- Grid-Based FastSLAM.

## 3.4 Lógica esperada

El robot explora el entorno en Gazebo usando LIDAR y odometría.

El algoritmo debe estimar simultáneamente:

1. la pose del robot;
2. el mapa de ocupación del entorno.

En esta opción, la localización se apoya en un conjunto de partículas. Cada partícula representa una hipótesis de pose y, según la implementación elegida, puede estar asociada a una hipótesis de mapa o a una forma de evaluar la consistencia de las mediciones con el mapa.

## 3.5 Dificultades principales

- Costo computacional alto si se usan muchas partículas.
- Necesidad de optimizar el cálculo de pesos.
- Necesidad de mantener una grilla de ocupación coherente.
- Riesgo de distorsión del mapa si la localización diverge.
- Necesidad de que el mapa final sea usable para navegación posterior.

## 3.6 Resultado esperado

Esta opción produce principalmente:

```text
Resultado V1: mapa grillado del entorno
```

Este mapa se usa luego en Parte B para navegación basada en grilla.

---

# 4. Parte A - Opción 2: Features con LIDAR

## 4.1 Entorno de trabajo

- Gazebo.
- TurtleBot3 simulado.

## 4.2 Sensores

- LIDAR.
- Odometría publicada por el entorno de simulación.

## 4.3 Algoritmos y herramientas de esta rama

Esta rama corresponde a SLAM basado en características geométricas detectadas con LIDAR.

Elementos principales:

- Detección y extracción de features a partir del LIDAR.
- Modelo de odometría.
- SLAM basado en landmarks/features.

Algoritmos posibles mencionados para esta rama:

- EKF SLAM.
- Graph SLAM.
- SEIF SLAM.

## 4.4 Aclaración importante

Esta rama **no es la misma** que la Opción 1.

No conviene mezclarla con Grid-Based FastSLAM. En esta opción el problema central es detectar features geométricos estables a partir del LIDAR y usar esos features como landmarks para localización y mapeo.

TP5 puede servir como referencia conceptual para landmarks, covarianzas y visualización en RViz, pero no implica que esta rama sea simplemente copiar FastSLAM de TP5.

## 4.5 Lógica esperada

El proceso esperado tiene dos etapas o pasadas:

1. **Primera pasada:** construir y consolidar un mapa de features/landmarks.
2. **Segunda pasada:** con la localización corregida o resuelta usando esos features, generar un mapa de grilla de ocupación con el LIDAR.

La segunda pasada simplifica el mapeo porque la trayectoria ya debería estar mejor estimada.

## 4.6 Dificultades principales

- Detectar features LIDAR estables y repetibles.
- Asociar observaciones nuevas con landmarks ya conocidos.
- Evitar landmarks falsos o inestables.
- Mantener consistencia entre mapa de landmarks y mapa grillado.
- Generar una grilla final útil para planificación.

## 4.7 Resultado esperado

Esta opción produce:

```text
Resultado V2: mapa grillado del entorno + landmarks LIDAR
```

Ese resultado puede alimentar una navegación híbrida en Parte B.

---

# 5. Parte A - Opción 3: Features con Cámara

## 5.1 Entorno de trabajo en Parte A

- RosBag.
- Datos reales o pregrabados de TurtleBot4.

## 5.2 Sensores

- LIDAR.
- Cámara.
- Odometría real del TurtleBot4.

## 5.3 Algoritmos y herramientas de esta rama

Esta rama corresponde a SLAM visual basado en ArUco Tags.

Elementos principales:

- Detección y extracción de features visuales usando ArUco Tags.
- Modelo de odometría.
- Graph SLAM.
- Loop closure / cierre de lazo.

## 5.4 Aclaración importante sobre Gazebo

En la Parte A de esta opción, el entorno principal es **RosBag**, no Gazebo.

Esta opción se apoya en datos reales/pregrabados porque Gazebo no modela con suficiente fidelidad los efectos ópticos de una cámara real.

Sin embargo, si se elige esta rama, en la Parte B habrá que adaptar la navegación para simulación en Gazebo mediante un nodo de landmarks virtuales o sensor equivalente. Esa adaptación pertenece a Parte B, no a Parte A.

## 5.5 Lógica esperada

El proceso esperado también tiene dos etapas:

1. **Primera pasada:** usar cámara, odometría, ArUco Tags y Graph SLAM para estimar trayectoria y landmarks visuales.
2. **Segunda pasada:** con la trayectoria corregida, proyectar lecturas de LIDAR para construir una grilla de ocupación consistente.

## 5.6 Dificultades principales

- Detección robusta de ArUco Tags.
- Motion blur.
- Baja densidad de tags.
- Tags parcialmente visibles.
- Asociación de observaciones con IDs de landmarks.
- Implementación obligatoria de Graph SLAM.
- Cierre de lazo para corregir deriva acumulada.
- Generar un mapa de ocupación final a partir de una trayectoria corregida.

## 5.7 Resultado esperado

Esta opción produce:

```text
Resultado V2: mapa grillado del entorno + landmarks visuales ArUco
```

---

# 6. Resultados posibles de Parte A

## 6.1 Resultado V1 - Mapa grillado del entorno

Este resultado corresponde principalmente a una solución basada en grillas de ocupación.

Sirve para navegación basada en mapa de ocupación.

Insumos esperados para Parte B:

- mapa de ocupación;
- resolución del mapa;
- origen del mapa;
- frames usados;
- método de localización sobre el mapa.

## 6.2 Resultado V2 - Mapa grillado del entorno + landmarks

Este resultado corresponde a soluciones que además del mapa de ocupación producen landmarks.

Sirve para navegación híbrida.

Insumos esperados para Parte B:

- mapa de ocupación;
- landmarks estimados;
- posible asociación de landmarks con IDs;
- método de localización basado en landmarks y/o grilla;
- relación entre el frame del mapa y el frame de los landmarks.

---

# 7. Parte B - Movimiento Automático / Navegación Autónoma

## 7.1 Objetivo general

En Parte B, el robot debe navegar automáticamente en simulación usando el mapa generado en Parte A.

El usuario debe poder indicar:

1. una pose inicial aproximada usando `2D Pose Estimate` en RViz;
2. una pose objetivo usando `2D Goal Pose` en RViz.

El sistema debe usar esa información para localizarse, planificar y moverse hasta el objetivo.

## 7.2 Entradas importantes

- Mapa generado en Parte A.
- Pose inicial publicada en `/initialpose`.
- Pose objetivo publicada en `/goal_pose`.
- Odometría.
- LIDAR.
- Landmarks si la arquitectura elegida los usa.

## 7.3 Entornos de prueba

La navegación se debe validar en Gazebo.

Entornos esperados:

- `custom_casa.launch.py`
- `custom_casa_obs.launch.py`
- `custom_casa_obs2.launch.py` como desafío opcional o entorno más difícil.

## 7.4 Separación correcta por módulos

Para evitar mezclar responsabilidades, Parte B debe pensarse como un sistema modular.

No todos los algoritmos pertenecen al mismo módulo.

---

## 7.5 Módulo 1 - Localización probabilística

### Responsabilidad

Estimar la pose del robot dentro del mapa.

### Entradas posibles

- Odometría.
- LIDAR.
- Mapa grillado.
- Landmarks, si existen.
- Pose inicial de RViz.

### Algoritmos posibles

- Filtro de partículas.
- EKF.
- UKF.
- Otro filtro probabilístico coherente con la arquitectura elegida.

### Salidas esperadas

- Pose estimada corregida.
- Incertidumbre, si corresponde.
- Posible publicación en un tópico tipo `/belief` o equivalente.

---

## 7.6 Módulo 2 - Planificación global

### Responsabilidad

Calcular un camino desde la pose actual estimada hasta la pose objetivo.

### Entradas posibles

- Mapa de ocupación.
- Pose inicial estimada.
- Pose objetivo.
- Inflado o margen de seguridad alrededor de obstáculos.

### Algoritmos posibles

- A*.
- Dijkstra.
- Theta*.
- RRT.
- RRT*.
- Otro planificador compatible con la grilla o representación elegida.

### Salidas esperadas

- Path global libre de colisiones.
- Camino lo suficientemente seguro como para no pasar pegado a paredes u obstáculos.

---

## 7.7 Módulo 3 - Seguimiento de trayectoria / Path Following

### Responsabilidad

Convertir el camino planificado en comandos de velocidad para que el robot lo siga.

### Entradas posibles

- Path global.
- Pose estimada actual.
- Velocidades máximas.
- Tolerancias de distancia y orientación.

### Algoritmos posibles

- Pure Pursuit.
- Control proporcional de heading/distancia.
- Otro controlador cinemático simple y defendible.

### Salidas esperadas

- Comandos en `/cmd_vel`.
- Seguimiento suave del camino.
- Llegada al objetivo.
- Ajuste del ángulo final.

---

## 7.8 Módulo 4 - Detección y evasión de obstáculos no mapeados

### Responsabilidad

Detectar obstáculos que no estaban en el mapa original y evitar colisiones.

### Entradas posibles

- LIDAR.
- Path actual.
- Pose estimada.

### Comportamientos posibles

- Frenar si hay obstáculo cercano.
- Replanificar si el camino queda bloqueado.
- Ejecutar una maniobra local de evasión.
- Volver al path global cuando sea seguro.

### Consideraciones

La Parte B exige manejar obstáculos no mapeados. No alcanza con planificar una vez y seguir el camino ciegamente.

---

## 7.9 Módulo 5 - Máquina de estados

### Responsabilidad

Coordinar los módulos anteriores.

### Estados posibles

Estos nombres son orientativos. La implementación final puede usar otros nombres.

```text
WAITING_FOR_INITIAL_POSE
WAITING_FOR_GOAL
LOCALIZING
PLANNING
FOLLOWING_PATH
AVOIDING_OBSTACLE
REPLANNING
ALIGNING_FINAL_ORIENTATION
GOAL_REACHED
ERROR_OR_RECOVERY
```

### Lógica esperada

La máquina de estados debe contemplar:

- espera de pose inicial;
- espera de objetivo;
- planificación inicial;
- ejecución del camino;
- cambio de objetivo durante el recorrido;
- replanning;
- obstáculos inesperados;
- llegada a posición final;
- ajuste de orientación final;
- posibilidad de recibir un nuevo objetivo después de llegar.

---

# 8. Parte B si se viene de Resultado V1

Si Parte A produjo solamente mapa grillado, Parte B se basa principalmente en:

- localización sobre grilla;
- LIDAR;
- odometría;
- planificación sobre mapa de ocupación;
- path following;
- evasión de obstáculos;
- máquina de estados.

En esta rama no hay landmarks como insumo central.

---

# 9. Parte B si se viene de Resultado V2

Si Parte A produjo mapa grillado + landmarks, Parte B puede usar navegación híbrida.

Además de los elementos de V1, puede incorporar:

- landmarks conocidos;
- localización usando landmarks;
- corrección de pose a partir de observaciones de landmarks;
- sensor virtual de landmarks en Gazebo, si la rama elegida requiere simular landmarks que en la realidad provienen de cámara o de otro sensor.

## 9.1 Caso especial: si se eligió cámara/ArUco en Parte A

Como la Parte A de cámara se basa en RosBag, para Parte B en Gazebo hay que crear o adaptar un nodo que emule landmarks virtuales.

Ese nodo debe considerar:

- densidad razonable de landmarks;
- visibilidad geométrica;
- oclusión por paredes u obstáculos;
- ruido en las mediciones;
- no publicar landmarks que no estén realmente visibles.

---

# 10. Parte C - Autonomía Completa Real

## 10.1 Objetivo general

La Parte C lleva el sistema al robot real.

El robot debe recorrer el laberinto real y buscar conos rojos de manera autónoma.

## 10.2 Capacidades esperadas

El sistema debe integrar:

- mapeo o uso del mapa construido;
- localización probabilística;
- planificación de caminos;
- seguimiento de trayectoria;
- evasión de obstáculos;
- percepción visual;
- detección de conos rojos;
- decisión autónoma;
- ejecución sobre hardware real.

## 10.3 Detección de conos rojos

El robot debe distinguir conos rojos de conos de otros colores.

Debe ignorar distractores cromáticos.

La detección visual no debe interpretarse como camino directo libre. Si el robot ve un cono a través de una abertura, rejilla o hueco, igual debe planificar un camino válido usando el mapa y evitando paredes.

## 10.4 Validación previa

Antes del turno de laboratorio, conviene validar con:

- RosBag de mapeo;
- RosBag de visión;
- pruebas en Gazebo;
- videos y capturas para la defensa.

## 10.5 Sim-to-Real

La Parte C puede fallar parcialmente en el robot real por factores físicos no modelados, como:

- ruido de sensores;
- cambios de iluminación;
- pérdida de tracción;
- deslizamiento;
- oclusiones;
- diferencias entre simulación y laboratorio.

Si ocurre, el informe debe incluir análisis técnico de fallas y estrategias de mitigación.

---

# 11. Resumen lógico del flowchart

```text
Parte A - SLAM
│
├── Opción 1: Grid-Based FastSLAM en Gazebo
│   ├── LIDAR
│   ├── odometría
│   ├── particle filter
│   ├── likelihood fields
│   └── Resultado V1: mapa grillado
│
├── Opción 2: Features con LIDAR en Gazebo
│   ├── LIDAR
│   ├── extracción de features geométricos
│   ├── EKF SLAM / Graph SLAM / SEIF SLAM
│   └── Resultado V2: mapa grillado + landmarks LIDAR
│
└── Opción 3: Features con Cámara usando RosBag
    ├── cámara
    ├── LIDAR
    ├── ArUco Tags
    ├── Graph SLAM obligatorio
    ├── loop closure
    └── Resultado V2: mapa grillado + landmarks visuales

Resultado de Parte A
│
└── Parte B - Movimiento Automático en Gazebo
    │
    ├── Localización probabilística
    ├── Planificación global
    ├── Path following
    ├── Evasión de obstáculos
    ├── Máquina de estados
    └── Llegada a posición + orientación final

Parte B validada
│
└── Parte C - Autonomía Completa Real
    ├── Robot real
    ├── laberinto físico
    ├── detección de conos rojos
    ├── planificación hacia el cono
    ├── evasión de paredes/obstáculos
    └── análisis sim-to-real
```

---

# 12. Decisión crítica pendiente

Antes de implementar fuerte, el grupo debe decidir qué opción de Parte A va a seguir.

## 12.1 Opción 1 - Grid-Based FastSLAM

Ventajas posibles:

- Produce directamente mapa grillado.
- Se conecta naturalmente con navegación por grilla.
- Usa LIDAR y Gazebo.

Riesgos:

- Puede ser pesada computacionalmente.
- Requiere manejo cuidadoso de partículas y mapa.
- Si no se optimiza, puede no correr bien en tiempo real.

## 12.2 Opción 2 - Features con LIDAR

Ventajas posibles:

- Se apoya en landmarks geométricos.
- Puede producir mapa grillado + landmarks.
- Conecta con conceptos ya trabajados de EKF/landmarks.

Riesgos:

- Detectar features robustos con LIDAR puede ser difícil.
- La asociación de landmarks puede complicarse.
- Requiere dos etapas/pasadas bien organizadas.

## 12.3 Opción 3 - Features con Cámara / ArUco

Ventajas posibles:

- Usa datos reales o pregrabados de TurtleBot4.
- Landmarks visuales con IDs pueden facilitar asociación.
- Graph SLAM permite corregir deriva con cierres de lazo.

Riesgos:

- Graph SLAM es obligatorio.
- Requiere cámara, calibración, detección ArUco y RosBags.
- Para Parte B hay que adaptar a Gazebo con sensor virtual de landmarks.
- Mayor riesgo técnico y de integración.

---

# 13. Regla de interpretación para Codex

Para usar este archivo como contexto de IA, respetar estas separaciones:

```text
Opción 1 = Grid-Based FastSLAM + particle filter + occupancy grid.
Opción 2 = Features LIDAR + EKF SLAM / Graph SLAM / SEIF SLAM.
Opción 3 = RosBag + cámara/LIDAR + ArUco + Graph SLAM obligatorio.
Parte B = módulos separados: localización, planificación, control, evasión y máquina de estados.
Parte C = robot real + detección de cono rojo + navegación autónoma + análisis sim-to-real.
```

No mezclar algoritmos entre ramas salvo que el grupo lo decida explícitamente y lo documente.

---

# 14. Qué no olvidar

- Parte A no termina hasta tener un mapa usable para navegación.
- Parte B depende directamente de la calidad del mapa de Parte A.
- Parte C depende de que Parte B ya funcione razonablemente en simulación.
- La máquina de estados debe coordinar el comportamiento del robot, no reemplazar a los algoritmos de localización, planificación o control.
- Los rosbags y videos pesados no deberían subirse al repo salvo decisión explícita del grupo.
- Hay que guardar evidencia para informe y defensa: mapas, capturas, RViz configs, videos y métricas.
- El informe debe justificar decisiones, limitaciones y fallas, especialmente en sim-to-real.
