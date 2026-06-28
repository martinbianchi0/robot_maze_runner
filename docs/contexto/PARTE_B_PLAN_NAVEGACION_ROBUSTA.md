# Parte B - Plan concreto de navegacion robusta

> Estado: roadmap tecnico para demo/defensa  
> Rama de referencia: `codex/parte-b-nav-robust-base`  
> Base conocida: `2dc39a5 test(parte B): add navigation diagnostic merge gate`

## 1. Objetivo real

En esta rama, "navegacion robusta" no significa que el robot llegue a cualquier
goal a cualquier costo. Significa que puede moverse de forma conservadora,
explicable y reproducible entre puntos del mapa sin chocar, y que cuando no
puede avanzar lo diagnostica sin esconder el problema.

Operativamente, Parte B debe lograr:

- moverse entre puntos arbitrarios del mapa sin chocar;
- atravesar rutas largas de punta a punta;
- doblar en pasillos, esquinas y zonas cerradas;
- no cortar esquinas contra paredes;
- no quedarse girando indefinidamente;
- reaccionar a obstaculos no mapeados vistos por LIDAR;
- replanificar si el camino queda bloqueado;
- declarar bloqueo seguro si no hay ruta o si hay duda razonable;
- mantener evidencia reproducible con unit tests, rollouts, smokes, matriz,
  rosbags/capturas/videos cuando aplique.

La politica de seguridad se mantiene: si el LIDAR ve un obstaculo frontal
cercano, no se fuerza avance para "hacer pasar" un caso.

## 2. Capacidades robustas que queremos construir

### C1 - Planificacion global segura

Debe lograr:

- A* no atraviesa celdas ocupadas/infladas;
- no planifica demasiado pegado a paredes;
- trata unknown de forma conservadora por defecto y configurable cuando haga
  falta diagnostico;
- busca start/goal libres cercanos sin inventar rutas peligrosas;
- si no hay ruta, falla limpio y deja razon clara en `/nav_debug`.

Tests/gates:

- mapas sinteticos con pasillos estrechos;
- rutas largas;
- goal bloqueado;
- celdas unknown;
- validacion de path contra costmap inflado;
- validacion de segmentos con Bresenham contra obstaculos.

Base actual:

- `planner.py` evita diagonales que cortan esquinas entre obstaculos;
- `limit_path_stride` mantiene waypoints densos;
- `test_planner.py` cubre pared con abertura, start/goal bloqueados, segmentos
  ocupados y densidad de waypoints;
- falta ampliar pasillos estrechos y rutas largas.

### C2 - Seguimiento de path en curvas y esquinas

Debe lograr:

- seguir waypoints densos sin recortar curvas peligrosamente;
- doblar en L, S y U;
- girar en el lugar cuando el heading es grande;
- no quedar oscilando cerca del goal;
- no avanzar si el heading es peligroso;
- mantener histeresis de rotacion para evitar alternancia de signo.

Tests/gates:

- rollout pasillo L;
- rollout pasillo S;
- esquina cerrada;
- giro 90 grados;
- giro 180 grados;
- goal detras del robot;
- goal cerca de pared;
- replay de bug de giro en cono.

Base actual:

- `PathFollower` tiene estados `ROTATE_TO_PATH`, `TRACK_PATH`,
  `STUCK_RECOVERY`, `BLOCKED_STOP`, `GOAL_REACHED`, `WATCHDOG_STOP`;
- `test_follower.py` cubre heading grande, giro sin avance, bloqueo frontal,
  recovery, watchdog, replay de giro en cono y goal cercano bloqueado;
- `test_navigation_rollout.py` ya cubre open map, goal detras, perpendicular,
  diagonal, pasillo L, pasillo S y pared con abertura;
- falta agregar casos mas largos, U-turn/dead-end y esquinas mas cerradas con
  criterio explicito de no cortar pared.

### C3 - Manejo de obstaculos no mapeados

Debe lograr:

- si el LIDAR ve obstaculo frontal, frenar;
- si el obstaculo bloquea el path, marcar overlay temporal y replanificar;
- si el obstaculo es temporal y desaparece, retomar;
- si no hay camino, declarar `BLOCKED_STOP` con razon clara;
- nunca ignorar LIDAR para "hacer pasar" un caso.

Tests/gates:

- mundo `custom_casa_obs.launch.py`;
- obstaculos simulados en matriz;
- caso bloqueado esperado;
- obstaculo frontal subito;
- path invalidado por overlay dinamico;
- recuperacion o bloqueo seguro.

Base actual:

- `nav_node.py` mantiene `static_costmap` separado del overlay dinamico;
- `test_nav_node_imports.py` cubre overlay, speckles aislados, cluster valido y
  replan cuando el overlay pisa el path;
- la matriz clasifica `A_MAP_OR_COSTMAP_ISSUE` cuando el scan/costmap dinamico
  contradice el camino;
- falta un smoke con obstaculo no mapeado controlado y ruta alternativa.

### C4 - Navegacion larga / punta a punta

Debe lograr:

- completar secuencias de varios goals;
- recorrer mapa grande sin acumular errores graves;
- no degradar despues de varios replans;
- mantener loop estable;
- reportar tiempo, distancia, replans, min scan y estados.

Tests/gates:

- smoke de ruta multi-goal;
- start a goal largo;
- ida y vuelta;
- goal diagonal lejano;
- secuencia de 4 a 6 goals;
- medicion de tiempo, distancia, replans y estados.

Base actual:

- `smoke_goal_nav.sh` manda una secuencia corta de goals;
- `smoke_nav_matrix.sh` reinicia sim por caso para aislar diagnostico;
- falta un smoke multi-goal que no reinicie sim entre goals y mida degradacion.

### C5 - Replanning y cambio de objetivo

Debe lograr:

- si llega un nuevo `/goal_pose`, cancelar path viejo;
- replanificar desde la pose actual;
- no quedarse con estado viejo latcheado;
- aceptar goals repetidos solo si corresponde;
- poder recibir otro goal despues de `GOAL_REACHED`;
- poder recibir otro goal despues de `BLOCKED_STOP`.

Tests/gates:

- publicar goal A y a mitad de camino goal B;
- publicar goal despues de llegar;
- publicar goal nuevo despues de `BLOCKED_STOP`;
- verificar `/nav_debug`, `/planned_path`, `/nav_state` y `/cmd_vel`.

Base actual:

- `_on_goal_pose` limpia el bloqueo latcheado;
- hay test de `BLOCKED_STOP` latcheado hasta nuevo goal;
- falta smoke end-to-end de cambio de goal durante movimiento.

### C6 - Recuperacion ante trabas

Debe lograr:

- detectar no-progress real;
- distinguir "avance lento correcto" de "estoy trabado";
- hacer recovery limitado;
- evitar loop infinito de rotacion;
- bloquear seguro si no resuelve;
- dejar razon clara: `no_progress`, `goal_regression`, `plan_failed`,
  `scan_front_blocked`, etc.

Tests/gates:

- no-progress controlado;
- rotacion prolongada;
- path invalidado repetidamente;
- near wall;
- blocked known.

Base actual:

- hay watchdog de progreso por waypoint y watchdog de regresion respecto del
  goal;
- `ROTATE_TO_PATH` participa en regresion para evitar giro eterno;
- matriz incluye `blocked_known`;
- falta medir recuperaciones repetidas y asegurar que no se acepten falsos
  positivos cuando el robot avanza lento pero correctamente.

### C7 - Integracion con localizacion

Debe lograr:

- separar modo debug `/odom` de modo final `/belief`;
- no vender `/odom` como localizacion final;
- cuando Parte A mejore, probar `live` con `/map` + `/belief`;
- tolerar cierta discrepancia mapa/scan sin romperse;
- clasificar problemas de capa A sin taparlos con tuning de Parte B.

Tests/gates:

- modo YAML + `/odom` para smoke base;
- modo live + `/belief` cuando Parte A este lista;
- comparacion de comportamiento con ambos modos;
- clasificacion de errores `A_MAP_OR_COSTMAP_ISSUE` vs `B_BUG_*`.

Base actual:

- `nav_base.launch.py` parametriza `pose_topic`, `pose_topic_type`,
  `map_source` y `map_topic`;
- `scripts/nav.sh live` deja preparado `/map` + `/belief`;
- falta validacion real cuando Parte A publique belief estable.

### C8 - Demo/defensa

Debe lograr:

- tener escenarios reproducibles para mostrar;
- tener videos/capturas;
- explicar por que frena;
- explicar por que no ignora obstaculos;
- mostrar matriz de diagnostico;
- mostrar diferencia entre "no llegue por seguridad" y "bug de Parte B".

Evidencia esperada:

- tabla de matriz;
- videos de RViz limpio;
- rosbag debug cuando algo falla;
- diagrama simple de estados;
- lista de limitaciones honestas.

## 3. Roadmap por objetivos chicos

Cada objetivo siguiente debe abrirse como tarea chica y cerrarse con evidencia.
No mezclar tuning, refactor y nuevos escenarios en un mismo cambio.

### R0 - Documentar plan de navegacion robusta

Alcance exacto:

- crear este documento;
- enlazarlo desde la decision de Parte B;
- no tocar logica de navegacion.

Archivos probables:

- `docs/contexto/PARTE_B_PLAN_NAVEGACION_ROBUSTA.md`;
- `docs/decisiones/PARTE_B_NAV_BASE_ROBUSTA.md`.

Que NO tocar:

- `src/maze_nav/maze_nav/*.py`;
- scripts de smoke;
- parametros para hacer pasar casos.

Validacion:

- `git diff --check`;
- `colcon build --symlink-install`;
- `colcon test --packages-select maze_nav --event-handlers console_direct+`.

Done:

- roadmap versionado;
- decision enlazada;
- validaciones verdes.

### R1 - Expandir matriz de navegacion a escenarios de robustez

Alcance exacto:

- agregar casos a `smoke_nav_matrix.sh` o crear una matriz nueva si conviene;
- mantener formato JSONL y clasificaciones;
- no cambiar el controlador.

Casos sugeridos:

- ruta larga;
- giro 90 grados;
- giro 180 grados;
- esquina cerrada;
- pasillo estrecho;
- goal cerca de pared;
- caso parecido a `perpendicular_turn` que hoy clasifica como
  `A_MAP_OR_COSTMAP_ISSUE`;
- secuencia multi-goal solo si no complica el diseno de la matriz por caso.

Archivos probables:

- `scripts/smoke_nav_matrix.sh`;
- `scripts/smoke_nav_matrix_client.py`;
- `docs/decisiones/PARTE_B_NAV_BASE_ROBUSTA.md`;
- opcional: nuevo doc corto de resultados.

Que NO tocar:

- `follower.py`;
- parametros de seguridad;
- mapas de Parte A.

Validacion:

- `python3 -m py_compile scripts/smoke_nav_matrix_client.py`;
- `SMOKE_SKIP_BUILD=1 ./scripts/smoke_nav_matrix.sh <casos nuevos>`;
- matriz completa default;
- `colcon test --packages-select maze_nav --event-handlers console_direct+`.

Done:

- matriz produce JSONL con clasificacion;
- los casos nuevos distinguen `B_OK_*`, `B_BUG_*` y `A_MAP_OR_COSTMAP_ISSUE`;
- docs actualizados;
- no cambia navegacion.

### R2 - Rollouts sinteticos de curvas/esquinas

Alcance exacto:

- fortalecer tests unitarios/rollout del follower y planner sin Gazebo;
- agregar escenarios que fallen rapido si el robot recorta esquinas o gira sin
  avanzar.

Casos sugeridos:

- pasillo L mas angosto;
- pasillo S mas largo;
- U-turn;
- dead-end;
- goal detras con pared cercana;
- esquina cerrada con footprint;
- no recortar esquina.

Archivos probables:

- `src/maze_nav/test/test_navigation_rollout.py`;
- `src/maze_nav/test/test_follower.py`;
- quizas `src/maze_nav/maze_nav/follower.py` si aparece bug claro.

Que NO tocar:

- smokes de Gazebo;
- parametros globales sin test que lo justifique;
- mapa YAML de Parte A.

Validacion:

- `colcon test --packages-select maze_nav --event-handlers console_direct+`;
- si se toca follower, correr tambien `./scripts/smoke_goal_nav.sh`.

Done:

- tests reproducibles sin Gazebo;
- si falla, arreglo chico y cubierto;
- no baja margen de seguridad.

### R3 - Smoke multi-goal largo

Alcance exacto:

- crear smoke headless que mande varios goals consecutivos sin reiniciar sim;
- medir degradacion de estado, replans y distancia;
- generar resumen JSON/tabla.

Casos sugeridos:

- ida;
- curva;
- vuelta;
- goal final;
- secuencia de 4 a 6 goals en `custom_casa`.

Archivos probables:

- nuevo `scripts/smoke_multi_goal_nav.sh`;
- nuevo cliente Python o extension de `smoke_goal_nav_client.py`;
- docs de Parte B.

Que NO tocar:

- logica de `nav_node.py`;
- tuning para hacer pasar la ruta larga.

Validacion:

- `python3 -m py_compile` del cliente;
- `SMOKE_SKIP_BUILD=1 ./scripts/smoke_multi_goal_nav.sh`;
- smokes existentes.

Done:

- pasa en `custom_casa`;
- deja tiempos, errores, replans y estados;
- no degrada `smoke_goal_nav.sh`, `smoke_safe_drive.sh` ni matriz.

### R4 - Mundo con obstaculos simples

Alcance exacto:

- validar `custom_casa_obs.launch.py`;
- no prometer `custom_casa_obs2.launch.py` todavia;
- clasificar si falla por Parte B o mapa/scan.

Archivos probables:

- nuevo smoke opcional para mundo obs;
- `scripts/smoke_nav_matrix.sh` si se parametriza launch;
- docs de resultados.

Que NO tocar:

- mundo `obs2`;
- controlador sin evidencia de bug;
- mapa de Parte A.

Validacion:

- launch headless de `custom_casa_obs.launch.py`;
- matriz o smoke minimo con 2 a 3 goals;
- rosbag si falla.

Done:

- guia reproducible o smoke;
- tabla de resultados;
- si aparece bug claro, test antes de fix.

### R5 - Obstaculo no mapeado y recuperacion

Alcance exacto:

- crear caso controlado donde aparece un obstaculo en path;
- demostrar freno, replan, retomar o bloqueo seguro.

Archivos probables:

- nuevo test/fixture de overlay dinamico;
- smoke con mundo obs o nodo auxiliar que publique obstaculo simulado si se
  decide hacerlo;
- docs de matriz.

Que NO tocar:

- politica de freno LIDAR;
- safety thresholds sin comparativa antes/despues.

Validacion:

- caso donde hay ruta alternativa: debe replanificar y retomar;
- caso sin ruta alternativa: debe terminar en `BLOCKED_STOP`;
- clasificacion diferenciable entre `B_OK_BLOCKED_SAFE`,
  `B_TOO_CONSERVATIVE` y `B_BUG_*`.

Done:

- evidencia headless;
- `/nav_debug` explica la causa;
- no se escribe el obstaculo dinamico permanentemente en mapa estatico.

### R6 - Replanning por cambio de goal

Alcance exacto:

- test/smoke para nuevo goal durante navegacion;
- verificar que se cancela el path viejo y el estado viejo no queda latcheado.

Archivos probables:

- `scripts/smoke_goal_nav_client.py` o nuevo cliente;
- `src/maze_nav/test/test_nav_node_imports.py`;
- docs.

Que NO tocar:

- planner;
- costmap;
- parametros de movimiento.

Validacion:

- goal A y a mitad de camino goal B;
- goal despues de `GOAL_REACHED`;
- goal despues de `BLOCKED_STOP`;
- revisar `/planned_path`, `/nav_state`, `/nav_debug`.

Done:

- el robot sigue el goal nuevo desde pose actual;
- no queda `BLOCKED_STOP` viejo;
- matriz/smokes existentes siguen verdes.

### R7 - Orientacion final de consigna

Alcance exacto:

- agregar gate con `align_final_yaw:=true`;
- mantener default demo sin alineacion final para evitar giros confusos.

Archivos probables:

- `test_follower.py`;
- smoke dedicado con goal cercano;
- docs.

Que NO tocar:

- default `align_final_yaw:=false`;
- tolerancias de posicion sin caso claro.

Validacion:

- posicion + yaw final dentro de tolerancia;
- no aparece giro infinito cerca del goal.

Done:

- modo con yaw final probado;
- modo demo no se rompe.

### R8 - Modo live con Parte A

Alcance exacto:

- cuando Parte A este madura, probar `/map` + `/belief`;
- no usar `/odom` como solucion final;
- comparar contra YAML + `/odom`.

Archivos probables:

- scripts de smoke live;
- docs de resultados;
- quizas launch args de `maze_nav` si falta un parametro.

Que NO tocar:

- filtros de Parte A desde esta tarea;
- tuning de Parte B para ocultar mala pose.

Validacion:

- modo YAML + `/odom`;
- modo live + `/belief`;
- misma secuencia de goals;
- clasificacion de errores.

Done:

- tabla comparativa;
- limites de Parte A documentados;
- si `/belief` falla, queda evidencia para Parte A.

### R9 - Evidencia para defensa

Alcance exacto:

- preparar capturas, videos, tabla de matriz y explicacion tecnica;
- no agregar features nuevas.

Archivos probables:

- `results/parte_b/`;
- docs de informe/defensa;
- scripts de grabacion si hace falta.

Que NO tocar:

- navegacion;
- mapas;
- parametros.

Validacion:

- reproducir los comandos de demo;
- guardar evidencia liviana;
- registrar rosbags pesados solo localmente o segun politica del repo.

Done:

- una demo limpia;
- una demo debug;
- tabla de resultados;
- limitaciones honestas.

## 4. Metricas minimas por prueba

Toda prueba robusta deberia registrar, como minimo:

- result/state final;
- goal error;
- yaw error si aplica;
- distancia recorrida;
- tiempo;
- numero de replans;
- min front scan;
- ultimo `/cmd_vel`;
- reason de `/nav_debug`;
- path_valid contra costmap;
- clasificacion diagnostica;
- si hubo `BLOCKED_STOP`, razon y evidencia de que `/cmd_vel` quedo en cero.

La matriz actual ya registra gran parte de esto. Los nuevos smokes deben copiar
ese formato para que las comparaciones sean automaticas.

## 5. Politica de tuning

Reglas para no perder robustez:

- no tocar tres parametros a la vez;
- cada cambio de parametro debe explicar que caso mejora y que riesgo trae;
- todo cambio debe correr matriz antes/despues;
- si mejora un caso pero empeora otro, no se acepta sin justificacion;
- no bajar seguridad para llegar;
- no cambiar `max_linear_mps`, `max_angular_rps`, distancias de stop o
  tolerancias de goal sin test que capture el bug real;
- si un caso falla dos veces, parar y clasificar causa raiz antes de seguir
  probando valores.

Plantilla minima para un cambio de parametro:

```text
Parametro:
Valor viejo / nuevo:
Caso que mejora:
Riesgo:
Tests antes:
Tests despues:
Resultado:
```

## 6. Mapa imperfecto

Parte A nunca va a producir un mapa perfecto. Parte B debe ser robusta ante:

- paredes ligeramente corridas;
- obstaculos fantasma moderados;
- espacios libres parcialmente mal mapeados;
- discrepancia mapa vs LIDAR;
- pose con ruido moderado.

Limites:

- si el LIDAR ve obstaculo frontal cercano, no se fuerza avance;
- si el mapa omite una pared, Parte B debe protegerse, no atravesarla;
- si el mapa esta muy mal, se clasifica como `A_MAP_OR_COSTMAP_ISSUE`;
- si `/belief` esta desalineado, se clasifica como `TF_POSE_ISSUE` o problema
  de localizacion antes de tocar el follower.

La navegacion fuerte para defensa no se mide solo por "llego". Tambien se mide
por saber frenar y explicar por que no era seguro seguir.

## 7. Que NO hacer

- No pasar directo a Nav2 salvo decision explicita y documentada.
- No esconder problemas de mapa con tuning agresivo.
- No ignorar LIDAR.
- No agregar Numba todavia.
- No hacer refactor grande sin tests.
- No mezclar Parte C/conos en este roadmap tecnico de Parte B.
- No usar `/odom` como argumento de localizacion final; es modo de debug/sim.
- No convertir un caso `A_MAP_OR_COSTMAP_ISSUE` en "OK" bajando seguridad.

## 8. Proximo objetivo tecnico recomendado

Recomendacion: empezar por **R1 - Expandir matriz de navegacion a escenarios de
robustez**.

Motivo: el problema historico fue que los tests unitarios podian pasar mientras
RViz mostraba comportamiento malo. La matriz headless es el puente entre ambos
mundos: levanta Gazebo, publica goals reales, observa `/odom`, `/cmd_vel`,
`/planned_path`, `/global_costmap`, `/nav_state` y `/nav_debug`, y clasifica el
fallo. Expandirla primero baja el riesgo de arreglar a ciegas.

Objetivo listo para copiar:

```text
Objetivo: R1 - Expandir matriz de navegacion a escenarios de robustez.

Contexto:
Trabajar en `codex/parte-b-nav-robust-base`. No tocar main, no mergear, no
rebasear, no agregar Numba, no tunear parametros para hacer pasar casos.

Alcance:
Agregar escenarios headless a `smoke_nav_matrix` o crear una matriz hermana:
ruta larga, giro 90 grados, giro 180 grados, esquina cerrada, pasillo estrecho,
goal cerca de pared y un caso tipo `perpendicular_turn` que pueda clasificar
mapa/scan vs bug de Parte B.

No tocar:
`follower.py`, `nav_node.py`, mapas de Parte A ni parametros de seguridad salvo
que la matriz descubra un bug claro y se agregue test antes del fix.

Validacion:
- `python3 -m py_compile scripts/smoke_nav_matrix_client.py`
- `git diff --check`
- `colcon test --packages-select maze_nav --event-handlers console_direct+`
- `SMOKE_SKIP_BUILD=1 ./scripts/smoke_nav_matrix.sh`

Done:
La matriz produce JSONL, clasifica cada caso, deja tabla resumida, docs
actualizados y no rompe los casos actuales.
```

Despues de R1, el siguiente paso natural es **R2 - Rollouts sinteticos de
curvas/esquinas**, para que los bugs descubiertos por la matriz tengan tests
rapidos y deterministas antes de tocar el controlador.
