# Flujo del TP Final

Este documento pasa a texto el diagrama general del TP Final.  
La idea es conservar la lógica del flowchart original, pero en un formato más fácil de leer, versionar y usar como contexto para Codex y para el grupo.

---

# Vista general

El TP Final se divide en tres grandes partes:

1. **Parte A - SLAM**
   - Se debe aplicar algún algoritmo que tome datos de sensores para construir un mapa del entorno.

2. **Parte B - Movimiento Automático**
   - Con el mapa ya formado, se debe lograr que el robot se localice, planifique y se mueva automáticamente hacia objetivos seleccionados.

3. **Parte C - Autonomía Completa Real**
   - Se debe llevar el sistema al robot real, recorriendo un laberinto y buscando conos rojos de forma autónoma.

---

# Parte A - SLAM

## Objetivo general

En esta parte hay que aplicar algún algoritmo que tome la data de los sensores para hacer un mapa del entorno.

El resultado de Parte A es la base para Parte B.  
Según la opción elegida, se puede obtener:

- **Resultado V1:** mapa grillado del entorno.
- **Resultado V2:** mapa grillado del entorno + landmarks.

---

## Opción 1 - Grid-Based FastSLAM

### Entorno de trabajo

- Gazebo.

### Sensores

- LIDAR.

### Algoritmos involucrados

- Likelihood Fields.
- Modelo de odometría.
- Filtro de partículas.
- Occupancy Grid.
- Grid-Based FastSLAM.

### Dificultades

Esta opción exige optimizar el código para que pueda correr rápido en la computadora y, al mismo tiempo, permita usar una gran cantidad de partículas.

La dificultad principal es que cada partícula tiene que mantener o evaluar información suficiente del mapa para poder calcular bien su peso, hacer likelihood field, mapear y resamplear.

### Qué se espera

Se espera que puedan correr el algoritmo con una buena cantidad de partículas, calculando likelihood field para el peso de cada partícula, mapeando bien y haciendo resampling correctamente.

### Resultado esperado

Esta opción lleva principalmente a:

## Resultado V1 - Mapa grillado del entorno

El producto principal es una grilla de ocupación del entorno.

Ese mapa luego se usa como entrada para la Parte B de movimiento automático.

---

## Opción 2 - Features con LIDAR

### Entorno de trabajo

- Gazebo.

### Sensores

- LIDAR.

### Algoritmos involucrados

- Detección y extracción de features en base a datos del LIDAR.
- Modelo de odometría.
- Cualquier algoritmo de SLAM basado en features, por ejemplo:
  - EKF SLAM.
  - Graph SLAM.
  - SEIF SLAM.

### Dificultades

La dificultad principal es la extracción de features del LIDAR.

Después de detectar features, hay que hacer SLAM con features, asumiendo que esos features son bastante estables.

El proceso esperado tiene dos pasadas:

1. **Primera pasada:** hacer un mapa de features.
2. **Segunda pasada:** con el mapa ya hecho y fijo, usarlo para localizarse y generar un mapa con LIDAR, asumiendo localización perfecta o corregida.

### Qué se espera

Se espera que, una vez obtenidos los features del LIDAR, se pueda hacer:

1. Una primera pasada para construir el mapa de features.
2. Una segunda pasada para generar el mapa de grilla, apoyándose en la localización corregida por los features.

### Resultado esperado

Esta opción lleva a:

## Resultado V2 - Mapa grillado del entorno + Landmarks

El producto final combina:

- mapa grillado del entorno;
- landmarks o features detectados y consolidados.

Ese resultado luego se usa como entrada para la Parte B de movimiento automático.

---

## Opción 3 - Features con Cámara

### Entorno de trabajo

- RosBag.

### Sensores

- LIDAR.
- Cámaras.

### Algoritmos involucrados

- Detección y extracción de features en base a ArUco Tags y cámaras.
- Modelo de odometría.
- Graph SLAM.

### Dificultades

Esta opción requiere usar una grabación de ROS, es decir, un RosBag.

Hay que probar el algoritmo de detección de ArUco Tags sobre datos reales o pregrabados.

La dificultad principal es que el algoritmo debe ser robusto ante:

- pérdida de definición por movimiento;
- separación entre tags;
- baja densidad o visibilidad parcial de landmarks;
- necesidad de implementar Graph SLAM obligatoriamente.

### Qué se espera

Se espera lograr features robustos ante pérdida de imagen o movimientos bruscos.

También se espera una buena localización y optimización de mapa con Graph SLAM.

El proceso esperado tiene dos pasadas:

1. Una primera pasada con mapa, cámara y tags.
2. Una segunda pasada con mapa de features ya fijo, localización resuelta y mapeo de paredes con LIDAR.

### Resultado esperado

Esta opción lleva a:

## Resultado V2 - Mapa grillado del entorno + Landmarks

El producto final combina:

- mapa grillado del entorno;
- landmarks visuales asociados a ArUco Tags;
- localización optimizada mediante Graph SLAM.

Ese resultado luego se usa como entrada para la Parte B de movimiento automático.

---

# Resultados de Parte A

## Resultado V1 - Mapa grillado del entorno

Este resultado aparece cuando el sistema de SLAM produce principalmente una grilla de ocupación.

Se usa para navegación basada en mapa de ocupación.

Este camino alimenta la Parte B con:

- mapa del entorno;
- localización en base al mapa;
- planificación sobre grilla;
- navegación evitando paredes y obstáculos.

---

## Resultado V2 - Mapa grillado del entorno + Landmarks

Este resultado aparece cuando el sistema de SLAM produce tanto una grilla de ocupación como un mapa de landmarks.

Se usa para navegación híbrida basada en:

- mapa grillado;
- landmarks;
- localización probabilística;
- planificación sobre mapa;
- posible corrección de pose usando landmarks.

Este camino alimenta la Parte B con más información que V1, pero también con mayor complejidad.

---

# Parte B - Movimiento Automático

La Parte B recibe como entrada el resultado de la Parte A.

Según el resultado obtenido, hay dos ramas principales:

1. Movimiento automático usando solo mapa grillado.
2. Movimiento automático usando mapa grillado + landmarks simulados o conocidos.

---

## Rama B1 - Movimiento con mapa grillado

### Entorno

- Gazebo.

### Sensor

- LIDAR.

### Descripción

Con el mapa ya formado, se debe emplear un algoritmo que permita al usuario seleccionar una estimación de la posición inicial del robot y la posición final del robot.

Una vez hecho eso, el robot debe:

1. localizarse continuamente;
2. planificar desde el punto de inicio hasta el punto final;
3. moverse automáticamente siguiendo el recorrido;
4. evitar obstáculos;
5. no atravesar paredes ni obstáculos bloqueantes;
6. llegar al objetivo final.

En el mapa de prueba puede haber obstáculos estáticos y dinámicos que no estaban presentes al momento de generar el mapa.

El algoritmo de localización y control debe ser robusto ante esos casos.

Hay que tener en cuenta que pueden existir obstáculos que no bloqueen completamente el camino, por ejemplo un LIDAR que detecta una pared o elemento a través de una abertura, pero eso no significa que el robot deba atravesarlo.

### Algoritmos sugeridos o posibles

- Likelihood Fields.
- Filtro de partículas.
- Máquina de estados.
- Localización.
- Path Planning.
- Path Following.
- Obstacle Detection.
- Obstacle Avoidance.

### Resultado esperado

El robot debe poder navegar automáticamente desde una pose inicial hacia una pose objetivo usando el mapa grillado.

---

## Rama B2 - Movimiento con mapa grillado + landmarks simulados

### Entorno

- Gazebo + landmarks simulados.

### Sensor

- LIDAR.
- Cámara simulada o sensor virtual de landmarks, según la arquitectura elegida.

### Descripción

Con el mapa ya formado, se debe emplear un algoritmo que permita al usuario seleccionar una estimación de la posición inicial del robot y la posición final del robot.

Una vez hecho eso, el robot debe:

1. localizarse continuamente;
2. planificar desde el punto de inicio hasta el punto final;
3. moverse automáticamente siguiendo el recorrido;
4. usar landmarks para mejorar la localización, si corresponde;
5. evitar obstáculos;
6. no atravesar paredes ni obstáculos bloqueantes;
7. manejar casos donde obstáculos tapen landmarks.

En el mapa de prueba puede haber obstáculos estáticos y dinámicos que no estaban presentes al momento de generar el mapa.

El algoritmo de localización y control debe ser robusto ante estos casos.

También pueden existir obstáculos que tapen landmarks. En ese caso, el sistema no debería usar mediciones de landmarks que no estén realmente visibles.

### Nodo de cámara o landmarks simulados

Para poder probarlo en simulación, se debe crear un nodo de “cámara” o sensor virtual que simule el sistema de reconocimiento de landmarks.

Este nodo debe publicar landmarks “vistos” para que puedan correr la localización en el simulador.

Hay que agregar ruido a las mediciones tal como ocurriría en la realidad.

Si no hay línea de visión directa entre el robot y el landmark, el robot no debería poder “ver” ese landmark.

### Algoritmos sugeridos o posibles

- Algún algoritmo de localización:
  - Filtro de partículas.
  - Kalman Filter.
  - EKF.
  - UKF.
  - otros métodos equivalentes.
- Máquina de estados.
- Localización.
- Path Planning.
- Path Following.
- Obstacle Detection.
- Obstacle Avoidance.

### Resultado esperado

El robot debe poder navegar automáticamente usando mapa grillado y, cuando corresponda, landmarks simulados o conocidos para mejorar la localización.

---

# Parte C - Autonomía Completa Real

## Entorno

- Robot real.
- Se recomienda usar Gazebo para pruebas previas.

## Objetivo general

En esta última sección se debe hacer un algoritmo que encuentre de manera completamente autónoma un cono rojo en el laberinto.

El robot real debe:

1. localizarse con su mapa;
2. buscar de manera autónoma el cono rojo;
3. tener en cuenta que puede haber otros conos de otros colores;
4. discriminar correctamente el cono rojo;
5. no intentar atravesar paredes aunque vea el cono a través de huecos;
6. planificar un camino válido hasta el cono rojo;
7. mostrar funcionamiento en el robot real.

## Relación con Parte A y Parte B

Para resolver Parte C, el grupo debe mostrar:

- el mapeo del entorno;
- la localización del robot;
- el movimiento autónomo;
- la autonomía completa para buscar el cono rojo.

Durante el turno de laboratorio, el grupo tendrá tiempo limitado para probar el sistema en el robot real.

## Recomendación de validación previa

Antes de probar con el robot real, se recomienda usar RosBags para verificar que el algoritmo de mapeo funcione.

También se recomienda probar el algoritmo de recorrido automático en Gazebo antes de la prueba real.

## Resultado esperado

El sistema completo debe integrar:

- SLAM o mapa generado;
- localización probabilística;
- planificación;
- seguimiento de trayectoria;
- evasión de obstáculos;
- detección visual del cono rojo;
- decisión autónoma;
- ejecución en robot real.

---

# Resumen lógico del flujo

```text
Parte A - SLAM
│
├── Opción 1: Grid-Based FastSLAM
│   └── Resultado V1: mapa grillado
│
├── Opción 2: Features con LIDAR
│   └── Resultado V2: mapa grillado + landmarks
│
└── Opción 3: Features con Cámara
    └── Resultado V2: mapa grillado + landmarks

Resultado V1 o V2
│
└── Parte B - Movimiento Automático
    │
    ├── Navegación con mapa grillado
    │
    └── Navegación con mapa grillado + landmarks simulados/conocidos
        │
        └── Localización + planificación + path following + obstacle avoidance

Parte B validada
│
└── Parte C - Autonomía Completa Real
    └── Robot real busca cono rojo de forma autónoma
```

---

# Decisión crítica pendiente

Antes de implementar, el grupo debe decidir qué camino tomar en la Parte A:

1. **Opción 1 - Grid-Based FastSLAM**
   - Más alineada con mapa grillado puro.
   - Puede ser pesada computacionalmente.
   - Requiere buen manejo de partículas, likelihood fields y occupancy grid.

2. **Opción 2 - Features con LIDAR**
   - Más alineada con TP5 de landmarks.
   - Requiere resolver bien extracción de features del LIDAR.
   - Permite mapa grillado + landmarks.

3. **Opción 3 - Features con Cámara**
   - Más ambiciosa.
   - Usa RosBags, cámara, ArUco Tags y Graph SLAM obligatorio.
   - Puede ser potente, pero tiene más riesgo técnico.

La decisión no debe tomarse solo por dificultad aparente, sino por:

- tiempo disponible;
- experiencia previa del grupo;
- calidad esperada del mapa;
- facilidad para conectar Parte A con Parte B;
- posibilidad de probar en Gazebo y luego en robot real;
- riesgo de implementación;
- qué tan defendible es la solución final.

---

# Qué no olvidar

- Parte A no termina hasta tener un mapa usable.
- Parte B depende directamente de la calidad del mapa de Parte A.
- Parte C depende de que Parte B ya funcione razonablemente en simulación.
- El sistema final debe poder ser explicado en informe y defensa.
- Hay que guardar evidencia: capturas, videos, mapas, RViz configs y resultados.
- Los rosbags y videos pesados no deberían subirse al repo, salvo que se acuerde otra cosa.
