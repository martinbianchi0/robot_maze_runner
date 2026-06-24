# PARTE C - ROBOT REAL Y AUTONOMÍA COMPLETA

Versión de contexto: v2  
Base usada: consignas oficiales, Tutorial 10, Tutorial 11 y teóricas 12-16.

Este documento define cómo pensar la Parte C del Trabajo Final: llevar el sistema al robot real y lograr autonomía completa para encontrar un cono rojo.

---

# 1. Objetivo de Parte C

El robot real debe encontrar de manera autónoma un cono rojo en el laberinto.

Para eso debe integrar:

- mapa;
- localización;
- planificación;
- movimiento autónomo;
- obstacle avoidance;
- percepción del cono rojo;
- decisión de objetivo;
- ejecución en robot real.

No alcanza con que el sistema funcione en Gazebo. Hay que poder probarlo o justificar qué pasó en el robot real.

---

# 2. Relación con Parte A y Parte B

Parte C depende de las partes anteriores:

- Parte A produce mapa.
- Parte B permite navegar autónomamente.
- Parte C agrega misión: buscar y llegar al cono rojo real.

Si Parte A o B quedan frágiles, Parte C se vuelve muy difícil.

---

# 3. Diferencias simulado vs real

Tutorial 10 marca diferencias importantes entre TurtleBot3 simulado y TurtleBot4 real.

## 3.1. Topics

En simulación se usan topics típicos como:

- `/scan`
- `/odom`
- `/calc_odom`
- `/cmd_vel`

En TurtleBot4 real pueden aparecer con namespace, por ejemplo:

- `/tb4_0/scan`
- `/tb4_0/odom`
- `/tb4_0/cmd_vel`

Por eso el código debe tener topics configurables.

## 3.2. QoS

En TurtleBot4 puede hacer falta usar QoS tipo `BEST_EFFORT` para algunos sensores.

No hardcodear una única configuración si después se rompe en el real.

## 3.3. LIDAR y frames

El LIDAR del robot real puede tener diferencias de orientación/posición respecto al simulado.

Hay que cuidar:

- transformaciones de frames;
- ángulo cero del scan;
- orientación del LIDAR;
- offsets físicos.

## 3.4. Intensidades del LIDAR

En el robot real, el mensaje de LIDAR puede tener intensidades.

Si una lectura tiene intensidad inválida/cero, puede convenir descartarla o tratarla como no confiable.

Esto evita falsos positivos.

## 3.5. Dinámica real

El robot real no acelera/frena instantáneamente.

Diferencias prácticas:

- demora en frenar;
- overshoot al girar;
- velocidades reales distintas a las simuladas;
- inercia;
- rozamiento;
- error de odometría.

Por eso las velocidades deben ser conservadoras.

---

# 4. Percepción del cono rojo

Parte C requiere detectar un cono rojo.

Posibles entradas:

- cámara RGB;
- imagen del TurtleBot4;
- RosBag;
- sensor real;
- procesamiento simple por color.

## 4.1. Estrategia inicial

Una estrategia simple y defendible:

1. obtener imagen;
2. convertir a HSV;
3. segmentar color rojo;
4. limpiar máscara;
5. detectar contorno/blob;
6. estimar centro en imagen;
7. decidir si el cono está visible;
8. generar objetivo o conducta de acercamiento.

## 4.2. Riesgos

- iluminación;
- otros objetos rojos;
- reflejos;
- tamaño aparente;
- cono parcialmente oculto;
- cámara movida;
- delay;
- calibración;
- confundir cono rojo con otro elemento.

## 4.3. Validación mínima

Antes del robot real:

- probar con imágenes o RosBag;
- guardar capturas;
- ajustar umbrales;
- publicar debug en RViz o imagen procesada.

---

# 5. Cómo conectar percepción con navegación

Hay dos enfoques posibles.

## 5.1. Buscar cono y luego navegar

1. Robot explora o sigue una ruta.
2. Detecta cono rojo.
3. Estima dirección/posición aproximada.
4. Define goal navegable.
5. Planifica y se acerca.

## 5.2. Navegar por waypoints y detectar durante el recorrido

1. Se define una estrategia de recorrido.
2. Robot visita zonas.
3. Mientras navega, busca cono rojo.
4. Si lo ve, cambia estado a acercamiento/goal.

Este enfoque puede ser más robusto si no se tiene una localización exacta del cono.

---

# 6. Máquina de estados para Parte C

Estados posibles:

1. `INIT`
2. `LOAD_MAP`
3. `LOCALIZE`
4. `SEARCH_CONE`
5. `CONE_DETECTED`
6. `ESTIMATE_CONE_GOAL`
7. `PLAN_TO_CONE`
8. `NAVIGATE_TO_CONE`
9. `AVOID_OBSTACLE`
10. `REPLAN`
11. `VERIFY_CONE`
12. `DONE`
13. `FAILURE`

La máquina de estados debe coordinar percepción y navegación.

---

# 7. Estrategia de búsqueda

Si no se conoce dónde está el cono, se necesita una estrategia.

Opciones:

- recorrer waypoints predefinidos;
- explorar frontera;
- seguir camino por pasillos;
- hacer barrido de zonas visibles;
- rotar en puntos estratégicos;
- usar mapa para visitar regiones importantes.

Recomendación inicial:

- empezar con waypoints manuales o generados sobre el mapa;
- no arrancar con exploración compleja si Parte B no está sólida;
- documentar que la estrategia busca cubrir zonas visibles del laberinto.

---

# 8. Seguridad y robustez

En real, priorizar:

- velocidades bajas;
- margen de seguridad alto;
- frenar ante obstáculo cercano;
- no confiar ciegamente en cámara;
- no atravesar paredes aunque el cono se vea por un hueco;
- validar goal contra mapa navegable;
- permitir abortar.

---

# 9. Relación con mapas y SLAM

El mapa de Parte A puede no coincidir perfecto con el laberinto real.

Problemas esperables:

- paredes corridas;
- odometría distinta;
- obstáculos nuevos;
- zonas desconocidas;
- cambios de iluminación;
- diferencias entre Gazebo y real.

El informe debe explicar qué decisiones se tomaron para manejar esas diferencias.

---

# 10. Pruebas mínimas antes del robot real

## Test C1 - Percepción offline

- Usar imagen o RosBag.
- Detectar rojo.
- Guardar resultado.

## Test C2 - Percepción online en simulación o cámara

- Nodo publica detección.
- Visualizar máscara/centro.

## Test C3 - Navegación a goal artificial

- Usar Parte B para navegar a un punto.
- Confirmar que el robot se mueve seguro.

## Test C4 - Buscar cono simulado

- Simular o mockear detección de cono.
- Cambiar estado a navegación hacia el cono.

## Test C5 - Robot real

- Bajar velocidades.
- Confirmar topics.
- Confirmar scan/cámara.
- Confirmar que `/cmd_vel` funciona.
- Probar detección.
- Probar navegación corta.
- Probar misión completa.

---

# 11. Qué guardar para informe y defensa

- video de simulación;
- video del robot real;
- capturas de RViz;
- mapa usado;
- trayectoria planeada;
- trayectoria ejecutada;
- detecciones del cono;
- estados de la máquina;
- problemas encontrados;
- explicación de por qué falló si falla;
- mejoras futuras.

La clase remarca que si en el robot real algo no funciona, no necesariamente destruye la entrega. Pero el informe debe explicar técnicamente qué pasó, por qué pasó y cómo se mejoraría.

---

# 12. Decisiones abiertas

- Cómo detectar el cono rojo.
- Si se usa RosBag para calibrar visión.
- Si se estima posición del cono o solo dirección.
- Cómo convertir detección en goal navegable.
- Cómo evitar que el robot intente atravesar paredes si ve el cono.
- Qué estrategia de búsqueda usar.
- Cómo parametrizar topics sim/real.
- Cómo manejar QoS del TurtleBot4.
- Qué velocidades usar en real.
- Cómo abortar de forma segura.

---

# 13. Recomendación preliminar

Arquitectura inicial recomendada:

- mantener Parte B como módulo de navegación general;
- agregar percepción del cono como módulo separado;
- agregar una máquina de estados superior para misión;
- usar topics configurables para sim/real;
- usar velocidades conservadoras;
- validar todo con RosBag o simulación antes del turno real.

No mezclar percepción, planning y control en un único nodo gigante.

---

# 14. Notas para Codex

Cuando Codex trabaje en Parte C:

- No asumir que el robot real usa los mismos topics que Gazebo.
- No hardcodear `/scan` y `/cmd_vel` sin parámetros.
- No depender de que la cámara vea siempre el cono.
- No generar goals que atraviesen paredes.
- Separar detección visual, navegación y máquina de estados.
- Priorizar pruebas chicas y evidencias.
