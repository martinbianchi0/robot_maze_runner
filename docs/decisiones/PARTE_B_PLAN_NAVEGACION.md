# Parte B — Plan de implementación: Navegación autónoma

> Estado: PLAN APROBADO PARA REVISIÓN. No implementado todavía.
> Depende de la Parte A (Grid-Based FastSLAM): ver `PARTE_A_PLAN_GRID_FASTSLAM.md`.
> Base: consignas oficiales, `docs/contexto/PARTE_B_NAVEGACION.md`, `docs/contexto/ANALISIS_DECISION_ARQUITECTURA.md`, TP3/TP5 (localización), TP6 (planning).

---

## 1. Objetivo

Con el mapa generado en la Parte A, el robot debe navegar de forma autónoma entre dos puntos arbitrarios elegidos por el usuario en RViz.

Eso implica resolver de forma concurrente:

1. recibir una pose inicial (`2D Pose Estimate` → `/initialpose`);
2. recibir una pose objetivo (`2D Goal Pose` → `/goal_pose`);
3. localizarse de forma probabilística en el mapa;
4. planificar un camino válido libre de colisiones;
5. seguir el camino con un controlador cinemático suave;
6. llegar a la posición Y la orientación final deseada;
7. evitar obstáculos no mapeados y re-planificar;
8. aceptar nuevos goals (incluso durante un recorrido en curso).

La Parte B no es solo path planning: integra localización, planning, control y máquina de estados.

## 2. Sistema de navegación elegido

Como la Parte A es Grid-Based FastSLAM (grilla pura), quedamos en el **Sistema 1 — Navegación basada en grilla pura** del flujo del TP.

La localización se hace por concordancia de mapas (MCL / filtro de partículas) sobre la grilla.
No usamos landmarks (eso sería el Sistema 2 o 3, que corresponden a las Opciones 2 y 3 de la Parte A).

## 3. Arquitectura objetivo

Módulos separados (regla de AGENTS.md: no un script gigante).

```
                 /map (de Parte A)
                      │
        ┌─────────────┼──────────────────────────────┐
        ▼             ▼                                ▼
  map_server     localization (MCL)             state_machine
 (carga mapa)    /scan,/odom → /belief,/particles    │ orquesta todo
        │             │                                │
        ▼             ▼                                ▼
   costmap (inflado)  └──► planner (A*/Theta*) ──► path_follower (Pure Pursuit)
   obstáculos inflados      /goal_pose → /planned_path     └─► /cmd_vel
        ▲                                                   ▲
        └──────────── obstacle_monitor (/scan en vivo) ─────┘
```

## 4. Decisiones técnicas iniciales

Todo parametrizable, nada hardcodeado.

| Componente | Decisión inicial | Razón |
|---|---|---|
| Mapa navegable | umbral ocupado/libre + tratar desconocido como no transitable en sim | entrega segura |
| Inflado de obstáculos | radio = radio robot + margen, configurable | robot tiene tamaño y error de control/localización |
| Localización | MCL / filtro de partículas sobre grilla, inicializado con `/initialpose` | robusto, conectado con TP3/TP5 |
| Planner | A* sobre grilla inflada; Theta* si el camino sale muy cuadriculado | simple, eficiente, defendible |
| Path following | Pure Pursuit (Tutorial 12) | práctico, ya explicado en clase |
| Ángulo final | rotación in-place al llegar a la posición | la consigna exige posición Y orientación |
| Re-planning | simple: detecto bloqueo → marco celdas ocupadas → replanifico | defendible y suficiente |
| Velocidades | conservadoras | la lógica luego va al robot real (Parte C) |

## 5. Máquina de estados

```text
WAITING_FOR_MAP → WAITING_FOR_INITIAL_POSE → LOCALIZING → WAITING_FOR_GOAL
   → PLANNING → FOLLOWING_PATH → (AVOIDING_OBSTACLE → REPLANNING)* → GOAL_REACHED
   (FAILURE como estado de escape)
```

Acepta nuevo goal desde `WAITING_FOR_GOAL`, `FOLLOWING_PATH` o `GOAL_REACHED` → vuelve a `PLANNING`.

## 6. Plan por etapas

Cada etapa tiene una prueba mínima antes de integrar.

### Etapa B0 — Mapa navegable y costmap
- Cargar el mapa de la Parte A, convertir probabilidad → ocupado/libre/desconocido.
- Inflar obstáculos. Publicar el costmap en RViz.

### Etapa B1 — Planner offline (Test B1)
- Start/goal manuales, generar path con A*, visualizar en RViz, verificar que no atraviesa paredes.

### Etapa B2 — Pure Pursuit con path fijo (Test B2)
- Seguir un camino predefinido sin planificar. Tunear lookahead y velocidades.

### Etapa B3 — Localización (MCL)
- Filtro de partículas sobre el mapa, inicializado con `/initialpose`, corrigiendo con `/scan` + odometría.
- Publicar `/belief` y `/particles`.

### Etapa B4 — Planner + follower integrados (Test B3)
- Goal desde `/goal_pose` → planificar → seguir → llegar a posición y ángulo final.

### Etapa B5 — Obstáculos y re-planning (Test B4)
- Detectar obstáculo no mapeado con LIDAR en vivo, frenar, marcar ocupado, re-planificar.

### Etapa B6 — Sistema completo + máquina de estados (Test B5)
- Integrar todo bajo la state machine. Probar en `custom_casa.launch.py` y `custom_casa_obs.launch.py`.
- Opcional/desafío: `custom_casa_obs2.launch.py`.

## 7. Optimización

Mismo criterio que la Parte A: NumPy vectorizado → **Numba** (`@njit`) para hot loops (likelihood del MCL, expansión de A*) → GPU sólo si hace falta.
El MCL comparte estructura con el filtro de la Parte A, así que la optimización se reutiliza.

## 8. Reutilización
- Parte A: filtro de partículas, likelihood field, modelo de movimiento → base directa del MCL.
- TP6: planificación de caminos sobre grilla.
- TP3/TP5: localización probabilística, markers RViz.

## 9. Riesgos y mitigaciones
- Ambigüedad de localización en pasillos simétricos → más partículas, buena pose inicial.
- Lookahead mal calibrado en Pure Pursuit → zigzag o cortar curvas cerca de paredes; tunear con Test B2.
- Caminos muy pegados a obstáculos → inflado suficiente.
- No atravesar paredes aunque el LIDAR vea espacio detrás de una abertura → combinar mapa estático + inflado + LIDAR.

## 10. Criterio de aceptación
- El robot recibe pose inicial y goal, se localiza, planifica, sigue el camino, evita un obstáculo simple, re-planifica y llega a posición y orientación, en el entorno estándar y con obstáculos.
