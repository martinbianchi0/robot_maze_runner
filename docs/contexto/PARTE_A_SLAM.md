# PARTE A - SLAM

Versión de contexto: v2  
Base usada: consignas oficiales, Tutorial 10, Tutorial 11, Tutorial 12, Tutorial 13 y teóricas 12-16.

Este documento busca dejar clara la toma de decisión para la Parte A del Trabajo Final. No es una implementación todavía. Sirve para que el grupo y Codex entiendan qué pide cada opción, qué riesgos tiene y qué conviene probar primero.

---

# 1. Objetivo de Parte A

La Parte A consiste en aplicar algún algoritmo de SLAM para construir un mapa del entorno usando sensores del robot.

El mapa generado en Parte A alimenta directamente la Parte B de navegación autónoma y, después, la Parte C en robot real.

La decisión de Parte A condiciona todo el resto del proyecto, porque define:

- qué tipo de mapa se obtiene;
- cómo se localiza el robot después;
- qué tan fácil es planificar caminos;
- qué evidencia se puede mostrar en RViz/informe/defensa;
- cuánto riesgo técnico se asume.

---

# 2. Tipos de mapa involucrados

Hay dos familias principales de mapas.

## 2.1. Mapa de grilla / occupancy grid

Un mapa de grilla discretiza el mundo en celdas. Cada celda puede representar probabilidad de ocupación, espacio libre o desconocido.

Aspectos importantes:

- La grilla tiene resolución fija.
- A menor tamaño de celda, más detalle, pero más memoria y cómputo.
- Cada celda se modela como variable aleatoria binaria.
- Se suele asumir independencia entre celdas para simplificar.
- El entorno se asume estático para el mapeo base.
- Se puede usar log-odds para actualizar ocupación de manera eficiente.
- La actualización con log-odds transforma productos probabilísticos en sumas.
- El modelo inverso del sensor determina cómo cada medición afecta las celdas.

Esto conecta directamente con TP5 Punto 1 y con la teoría de mapas de ocupación.

## 2.2. Mapa de landmarks / features

Un mapa de landmarks no describe directamente todo el espacio libre/ocupado. Describe posiciones de elementos detectables del entorno.

Puede servir para localización, pero por sí solo no dice necesariamente por dónde puede pasar el robot.

Aspectos importantes:

- Los landmarks pueden venir de LIDAR, cámara, ArUco tags u otros detectores.
- La dificultad fuerte es detectar features robustas y asociarlas correctamente en el tiempo.
- Una asociación de datos incorrecta puede hacer divergir el algoritmo.
- Si se usa EKF SLAM, el estado crece con la cantidad de landmarks.
- Si se usa GraphSLAM, se guardan poses y restricciones para optimizar globalmente.

---

# 3. Opción 1 - Grid-Based FastSLAM

## 3.1. Qué pide esta opción

- Entorno: Gazebo.
- Sensor principal: LIDAR.
- Resultado principal: mapa grillado del entorno.
- Algoritmos esperados:
  - modelo de odometría;
  - likelihood field;
  - filtro de partículas;
  - occupancy grid;
  - Grid-Based FastSLAM.

## 3.2. Idea central

Grid-Based FastSLAM usa partículas para representar hipótesis de trayectoria del robot.

Cada partícula mantiene su propio mapa de grilla. Para cada partícula:

1. Se propaga la pose con un modelo de movimiento.
2. Se actualiza su mapa usando las mediciones del LIDAR.
3. Se calcula el peso comparando mediciones con su propio mapa.
4. Se remuestrean partículas.
5. El mapa mostrado puede ser el mapa de la partícula de mayor peso.

La idea teórica sale de Rao-Blackwellización: separar el problema en trayectoria del robot y mapeo con poses conocidas.

## 3.3. Ventajas

- Produce directamente el mapa que necesita Parte B.
- No depende de extraer landmarks robustos.
- Usa LIDAR y Gazebo, que ya están disponibles.
- Conecta con:
  - TP5 Punto 1: ocupación/log-odds/modelo inverso.
  - TP5 Punto 2: partículas, pesos, resampling, RViz.
  - TP6: mapa de grilla para planning.

## 3.4. Dificultades

La dificultad central es computacional.

Cada partícula mantiene un mapa completo. Si hay muchas partículas y un mapa grande, memoria y tiempo pueden explotar.

La teoría remarca:

- los mapas de grilla requieren bastante memoria;
- cada partícula lleva su propio mapa;
- por eso el número de partículas debería mantenerse bajo;
- si se usa odometría cruda, el error de rotación ensucia mucho el mapa;
- scan matching o alguna mejora local de odometría reduce la cantidad de partículas necesarias.

## 3.5. Puntos críticos de implementación

Para que esta opción sea viable, habría que ser pragmáticos:

- elegir una resolución de mapa razonable;
- limitar tamaño físico del mapa;
- no usar demasiadas partículas al principio;
- actualizar solo celdas alcanzadas por ray tracing del LIDAR;
- usar log-odds;
- clipear log-odds mínimos/máximos para evitar saturaciones absurdas;
- usar el mapa de la mejor partícula como resultado principal;
- evaluar si hace falta scan matching o una corrección local simple;
- no intentar promediar mapas de partículas sin criterio, porque puede generar mapas inconsistentes.

## 3.6. Qué significa “mejor partícula”

En Grid-Based FastSLAM, si cada partícula tiene su propio mapa, surge la pregunta: cuál es el mapa final?

La opción práctica es tomar el mapa de la partícula con mayor peso.

No conviene hacer un promedio ingenuo de mapas si la distribución es multimodal, porque el promedio puede no representar ninguna hipótesis consistente.

## 3.7. Riesgo

Riesgo medio-alto.

No es conceptualmente lo más difícil, pero puede volverse pesado y lento. La clave es implementar una versión acotada y defendible.

## 3.8. Prueba mínima para validar esta opción

Antes de hacer todo el sistema:

1. Lanzar `custom_casa.launch.py`.
2. Escuchar `/scan`, `/odom`, `/calc_odom`.
3. Crear un nodo simple que construya un occupancy grid usando `/calc_odom` como pose conocida.
4. Guardar un mapa inicial.
5. Ver si el mapa sale razonable.
6. Recién después agregar partículas.

Si con pose conocida el mapa ya sale mal, no tiene sentido meter FastSLAM todavía.

---

# 4. Opción 2 - Features con LIDAR

## 4.1. Qué pide esta opción

- Entorno: Gazebo.
- Sensor principal: LIDAR.
- Resultado: mapa grillado + landmarks/features.
- Algoritmos esperados según consigna:
  - extracción de features con LIDAR;
  - modelo de odometría;
  - EKF SLAM, Graph SLAM o SEIF SLAM.

## 4.2. Idea central

La idea es extraer features geométricas del LIDAR, por ejemplo:

- esquinas;
- columnas;
- segmentos;
- puntos distintivos;
- intersecciones o discontinuidades.

Después, esas features se usan como landmarks para hacer SLAM.

Luego, con el mapa de features y una localización más confiable, se puede construir un mapa de grilla del entorno.

## 4.3. Ventajas

- Conecta naturalmente con TP5 de landmarks.
- El mapa de landmarks puede ayudar a localización.
- Puede producir un resultado V2: grilla + landmarks.
- Permite defender mejor conceptos de SLAM basado en features.

## 4.4. Dificultades

El cuello de botella no es solo el algoritmo de SLAM, sino la extracción robusta de features.

Problemas esperables:

- una feature puede aparecer/desaparecer según la pose;
- una esquina puede moverse numéricamente por ruido;
- varias features cercanas pueden confundirse;
- la asociación de datos puede romper el SLAM;
- si los landmarks no son estables, EKF/Graph/SEIF no arreglan mágicamente el problema.

## 4.5. EKF SLAM en esta opción

EKF SLAM representa todo en una gaussiana grande:

- pose del robot: dimensión 3;
- cada landmark 2D agrega 2 dimensiones;
- con n landmarks, el estado tiene dimensión 3 + 2n;
- la matriz de covarianza contiene correlaciones robot-landmark y landmark-landmark.

Ciclo típico:

1. Predicción con odometría.
2. Predicción de medición.
3. Medición real.
4. Asociación de datos.
5. Actualización.
6. Integración de nuevos landmarks.

Ventaja:

- Muy defendible teóricamente.
- Se conecta con TP4.

Riesgo:

- La matriz crece.
- La data association incorrecta puede divergir.
- Depende muchísimo de features estables.

## 4.6. Graph SLAM en esta opción

Graph SLAM representa poses como nodos y restricciones como arcos.

Tipos de arcos:

- odometría entre poses consecutivas;
- cierre de lazo cuando se observa una zona o landmark ya visto.

Después se optimiza el grafo para minimizar error global.

Ventaja:

- Mejor para cierre de lazo.
- Corrige trayectorias pasadas.
- Más fuerte conceptualmente para defensa.

Riesgo:

- Requiere front-end y back-end:
  - front-end: crear nodos/arcos/asociaciones;
  - back-end: optimización.
- Implementar un GraphSLAM serio puede ser más pesado.

## 4.7. Nota sobre FastSLAM en esta rama

Aunque en clase oral se mencionó que podría considerarse algún algoritmo alternativo, para evitar confusiones con Codex y con la consigna escrita, esta rama debe documentarse principalmente como:

- Features LIDAR + EKF SLAM;
- Features LIDAR + Graph SLAM;
- Features LIDAR + SEIF SLAM.

No mezclarla de entrada con Grid-Based FastSLAM. Si el grupo quisiera usar FastSLAM con landmarks en esta opción, conviene confirmarlo con docentes.

## 4.8. Riesgo

Riesgo medio-alto.

Puede ser más liviana que Grid-Based FastSLAM si las features salen bien, pero esa condición es justamente lo difícil.

## 4.9. Prueba mínima para validar esta opción

Antes de hacer SLAM:

1. Lanzar simulación.
2. Tomar `/scan`.
3. Extraer features LIDAR.
4. Publicarlas en RViz como markers.
5. Mover el robot suave.
6. Ver si las features son estables en el tiempo.

Si las features bailan mucho, esta opción queda muy riesgosa.

---

# 5. Opción 3 - Features con cámara / ArUco / RosBag

## 5.1. Qué pide esta opción

- Entorno principal Parte A: RosBag.
- Robot/datos: TurtleBot4 real grabado.
- Sensores:
  - cámara;
  - LIDAR;
  - odometría/tf.
- Landmarks:
  - ArUco Tags u otras marcas visuales.
- Algoritmo clave:
  - Graph SLAM obligatorio o fuertemente esperado según consigna de esta rama.

## 5.2. Idea central

Usar la cámara del TurtleBot4 para detectar tags visuales, asociarlos como landmarks y construir un grafo de poses/restricciones.

El LIDAR se puede usar para construir mapa de grilla una vez que la localización/poses están resueltas o suficientemente corregidas.

## 5.3. Ventajas

- Puede ser la opción más potente y elegante.
- Usa robot real/rosbag desde temprano.
- Graph SLAM permite optimizar trayectoria y cierres de lazo.
- Si sale bien, es muy fuerte para informe y defensa.

## 5.4. Dificultades

Riesgo alto.

Problemas esperables:

- detección de ArUco bajo movimiento;
- calibración/cámara/tf;
- asociación de tags;
- sincronización de datos;
- implementación de GraphSLAM;
- adaptación posterior a Gazebo para Parte B;
- puente sim-to-real.

## 5.5. Relación con Gazebo

En Parte A, esta opción no usa Gazebo como entorno principal: usa RosBag.

Pero si se elige esta rama, para Parte B se necesita una adaptación en simulación. Según el flujo del TP, habría que crear un nodo que simule landmarks visibles para poder probar navegación en Gazebo.

Ese nodo debería:

- publicar landmarks visibles;
- agregar ruido;
- respetar campo de visión;
- no publicar landmarks tapados por paredes;
- no “ver” a través de obstáculos.

## 5.6. Riesgo

Riesgo alto.

Conviene elegirla solo si el grupo quiere apostar fuerte a cámara/GraphSLAM y tiene tiempo para debuggear RosBag, cámara y optimización.

---

# 6. Comparación rápida de opciones

| Criterio | Opción 1 Grid-Based FastSLAM | Opción 2 Features LIDAR | Opción 3 Cámara/ArUco |
|---|---|---|---|
| Resultado | Grilla | Grilla + landmarks | Grilla + landmarks |
| Entorno Parte A | Gazebo | Gazebo | RosBag |
| Sensor principal | LIDAR | LIDAR | Cámara + LIDAR |
| Mayor dificultad | Cómputo/memoria | Features robustas | GraphSLAM + cámara |
| Reutiliza TP5 | Sí, partículas/mapa | Sí, landmarks | Parcial |
| Reutiliza TP6 | Sí, mapa para planning | Sí | Sí, después |
| Riesgo | Medio-alto | Medio-alto | Alto |
| Más directa para Parte B | Sí | Sí, si hay grilla | Sí, pero requiere adaptación |
| Más defendible si sale bien | Buena | Buena | Muy buena |
| Más simple para arrancar | Opción 1 acotada | Solo si features salen fácil | No |

---

# 7. Recomendación preliminar

La recomendación preliminar es arrancar validando **Opción 1 - Grid-Based FastSLAM**, pero de forma acotada y progresiva.

Motivos:

- produce directamente el mapa grillado que se necesita para Parte B;
- evita depender de features LIDAR robustas;
- evita depender de cámara/ArUco/GraphSLAM desde el inicio;
- conecta mejor con TP5 y TP6;
- permite validar por etapas:
  1. mapeo con pose conocida;
  2. partículas con pocos mapas;
  3. pesos/resampling;
  4. mapa de mejor partícula.

La decisión no queda cerrada hasta hacer una prueba mínima.

---

# 8. Plan de validación recomendado antes de implementar fuerte

## Paso A1 - Mapeo con pose conocida

Crear un nodo que use:

- `/scan`;
- `/calc_odom`;
- occupancy grid;
- log-odds;
- modelo inverso del LIDAR.

Objetivo:

- producir un mapa razonable de `custom_casa`.

Si esto falla, hay que corregir ray tracing, frames, resolución o modelo inverso.

## Paso A2 - Publicar mapa en RViz

Publicar como:

- `nav_msgs/OccupancyGrid`;
- tópico sugerido: `/slam_map` o `/generated_map`.

Guardar también en `maps/sim/`.

## Paso A3 - Agregar partículas

Agregar filtro de partículas con pocos candidatos.

Cada partícula:

- pose;
- peso;
- mapa propio;
- actualización por scan.

## Paso A4 - Elegir mejor partícula

Publicar:

- trayectoria estimada;
- mapa de mejor partícula;
- opcional: partículas como `PoseArray`.

## Paso A5 - Evaluar rendimiento

Medir:

- frecuencia de actualización;
- memoria;
- calidad visual del mapa;
- estabilidad al cerrar lazo;
- si se puede usar en Parte B.

---

# 9. Decisiones abiertas

- Resolución del mapa.
- Tamaño del mapa.
- Cantidad inicial de partículas.
- Si se implementa scan matching o una corrección local simplificada.
- Cómo calcular pesos:
  - likelihood field;
  - comparación ray casting;
  - distancia a obstáculos;
  - modelo más simple.
- Cuándo resamplear.
- Cómo guardar el mapa.
- Si el mapa final será de la mejor partícula o alguna combinación más elaborada.
- Cómo convertir probabilidades a mapa navegable.
- Qué hacer con celdas desconocidas en planning.

---

# 10. Evidencia a guardar

Para informe y defensa:

- video/captura de RViz con mapa creciendo;
- mapa final guardado;
- trayectoria del robot;
- comparación odometría vs estimación si existe;
- explicación de errores;
- impacto de resolución/cantidad de partículas;
- justificación de por qué se eligió esta opción;
- limitaciones y mejoras futuras.

---

# 11. Notas para Codex

Cuando Codex lea este documento:

- No implementar todo de golpe.
- No mezclar las tres opciones.
- Primero proponer arquitectura.
- Primero validar mapeo con pose conocida.
- No crear una solución gigante sin pruebas intermedias.
- Mantener ROS 2 Humble.
- Usar topics reales del workspace.
- Documentar cada decisión técnica.
- Cuidar rendimiento: mapas por partícula pueden ser caros.
- Priorizar una solución defendible, medible y testeable.
