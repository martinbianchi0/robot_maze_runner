# Parte C — Plan de implementación: Robot real y misión de conos rojos

> Estado: PLAN APROBADO PARA REVISIÓN. No implementado todavía.
> Depende de las Partes A y B: ver `PARTE_A_PLAN_GRID_FASTSLAM.md` y `PARTE_B_PLAN_NAVEGACION.md`.
> Base: consignas oficiales, `docs/contexto/PARTE_C_ROBOT_REAL.md`, `docs/contexto/ANALISIS_DECISION_ARQUITECTURA.md`.

---

## 1. Objetivo

Portar el sistema validado en simulación al robot físico del laboratorio y ejecutar una misión de percepción visual activa:
el robot explora un laberinto real de forma autónoma, busca, reconoce y navega hacia **conos rojos**, ignorando distractores de otros colores y sin intentar atravesar paredes caladas aunque vea el cono por un hueco.

La consigna aclara que si el robot real no se comporta perfecto, no implica reprobar, pero es obligatorio un **análisis Sim-to-Real** riguroso en el informe.

## 2. Arquitectura objetivo

Se reutiliza la Parte B como módulo de navegación general y se le suma percepción + una máquina de estados de misión.
Módulos separados (regla de AGENTS.md).

```
   cámara ─► cone_detector (visión rojo) ─► /cone_detection (dirección/posición estimada)
                                                   │
   /map ─► localización + planner + follower (Parte B)        ▼
                          ▲                          mission_state_machine
                          └──── goal navegable ◄──────────────┘ (coordina búsqueda + acercamiento)
                                                   │
                                              /cmd_vel (velocidades conservadoras)
```

## 3. Decisión central: diseño sim/real desde el principio

El mayor riesgo de la Parte C es el acoplamiento a la simulación.
Por eso, **todos los topics, QoS, frames y velocidades son parámetros**, con perfiles separados.

| Aspecto | Simulación | Robot real (TurtleBot4) |
|---|---|---|
| scan | `/scan` | `/tb4_0/scan` |
| odom | `/odom` o `/calc_odom` | `/tb4_0/odom` |
| cmd_vel | `/cmd_vel` | `/tb4_0/cmd_vel` |
| QoS sensores | default | evaluar `BEST_EFFORT` |
| LIDAR | nominal | cuidar frames, ángulo cero, offset, filtrar intensidades inválidas |
| Velocidades | moderadas | bajas (inercia, overshoot, demora al frenar) |
| Tolerancias | ajustadas | más amplias |

## 4. Percepción del cono rojo

Estrategia inicial simple y defendible (procesamiento por color):

1. obtener imagen de la cámara;
2. convertir a HSV;
3. segmentar el rango de rojo (dos bandas, por el wraparound del Hue);
4. limpiar la máscara (morfología);
5. detectar contorno/blob y su centro;
6. decidir si el cono está visible y estimar dirección/posición aproximada;
7. **filtrar distractores**: descartar otros colores y blobs muy chicos.

Validación crítica: la detección reporta una coordenada estimada al planner, que genera una trayectoria válida que esquiva los muros reales.
Ver el cono NO implica que el camino directo esté libre.

## 5. Máquina de estados de misión

```text
INIT → LOAD_MAP → LOCALIZE → SEARCH_CONE
   → CONE_DETECTED → ESTIMATE_CONE_GOAL → PLAN_TO_CONE → NAVIGATE_TO_CONE
   → (AVOID_OBSTACLE → REPLAN)* → VERIFY_CONE → DONE
   (FAILURE como escape, con aborto seguro)
```

Estrategia de búsqueda inicial: recorrer waypoints sobre el mapa cubriendo zonas visibles del laberinto, detectando el cono durante el recorrido.
No arrancar con exploración de fronteras compleja si la Parte B no está sólida.

## 6. Plan por etapas

Dada la disponibilidad acotada del hardware, se valida TODO con rosbags de cátedra antes del turno de laboratorio.

### Etapa C0 — Perfiles sim/real
- Parametrizar topics, QoS, frames y velocidades. Cargar perfil por archivo de parámetros.

### Etapa C1 — Percepción offline (Test C1)
- Con el **rosbag de visión**, detectar rojo, ajustar umbrales HSV, guardar capturas, validar filtrado de distractores.

### Etapa C2 — Percepción online (Test C2)
- Nodo que publica la detección y la máscara/centro para debug en RViz/imagen.

### Etapa C3 — SLAM/localización con rosbag de mapeo
- Con el **rosbag de mapeo**, validar SLAM y calibrar filtros frente al ruido real de LIDAR y odometría.

### Etapa C4 — Navegación a goal artificial (Test C3)
- Usar la Parte B para navegar a un punto, confirmando movimiento seguro con velocidades reales.

### Etapa C5 — Misión con cono mockeado (Test C4)
- Mockear la detección → cambiar de estado → planificar y acercarse al cono.

### Etapa C6 — Robot real (Test C5)
- En el turno de laboratorio (bloque de 2 h): confirmar topics, scan, cámara y `/cmd_vel`; bajar velocidades; probar detección, navegación corta y la misión completa.

## 7. Reutilización
- Parte A: SLAM/mapa para el laberinto real (con rosbag de mapeo).
- Parte B: localización, planner, path following, evasión, máquina de estados → módulo de navegación general.
- TP Final Parte 0: control reactivo y máquina de estados, base de la lógica de misión.

## 8. Seguridad y robustez (robot real)
- velocidades bajas y margen de seguridad alto;
- frenar ante obstáculo cercano;
- no confiar ciegamente en la cámara;
- no generar goals que atraviesen paredes;
- validar todo goal contra el mapa navegable;
- permitir abortar de forma segura.

## 9. Riesgos y mitigaciones (brecha Sim-to-Real)
- Iluminación / otros objetos rojos / reflejos → calibrar umbrales con rosbag, filtrar por tamaño.
- Mapa de Parte A no coincide exacto con el laberinto real (paredes corridas, odometría distinta) → tolerancias amplias, documentar.
- Pérdida de tracción, inercia, overshoot, ruido EM → velocidades conservadoras.
- Si el robot real no se comporta perfecto → **obligatorio**: apartado de diagnóstico científico de fallas en el informe (causas físicas + mitigaciones).

## 10. Evidencia a guardar
- video de simulación y del robot real;
- capturas de RViz, mapa usado, trayectoria planeada vs ejecutada;
- detecciones del cono y estados de la máquina;
- problemas encontrados y análisis Sim-to-Real.

## 11. Criterio de aceptación
- El sistema, validado con rosbags, explora el laberinto, discrimina conos rojos de distractores, genera goals navegables que respetan las paredes reales y se acerca al cono; y el informe documenta el desempeño y la brecha Sim-to-Real.
