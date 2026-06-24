# Parte A — Plan de implementación: Grid-Based FastSLAM (Opción 1)

> Estado: PLAN APROBADO PARA REVISIÓN. No implementado todavía.
> Decisión de arquitectura: se elige la **Opción 1 - Grid-Based FastSLAM** para la Parte A.
> Base: consignas oficiales, `docs/contexto/PARTE_A_SLAM.md`, `docs/contexto/ANALISIS_DECISION_ARQUITECTURA.md`, TP5 (FastSLAM de landmarks) y TP5 ejercicio 1 (occupancy grid con log-odds).

---

## 1. Por qué Opción 1

Se elige Grid-Based FastSLAM por sobre Features-LIDAR (Opción 2) y Cámara/ArUco (Opción 3) por estos motivos.

- Produce directamente el mapa de ocupación que necesitan las Partes B y C, sin pasos intermedios.
- No depende de extraer features geométricas robustas ni de resolver asociación de datos, que es el riesgo central de la Opción 2.
- Usa Gazebo y LIDAR, ya disponibles en el workspace, sin necesidad de RosBag ni cámara como en la Opción 3.
- Su único riesgo real es computacional, y se mitiga con vectorización y, de ser necesario, GPU (hay una RTX 3090 disponible).

La elección de SLAM en la Parte A condiciona la localización de la Parte B.
Con Opción 1 quedamos en el **Sistema 1 (navegación basada en grilla pura)** del flujo del TP, que es el camino más directo.

---

## 2. Arquitectura objetivo

```
/scan (LaserScan) ─────┐
/calc_odom (Odometry) ─┼─► nodo grid_fastslam ─► /map (OccupancyGrid, mejor partícula)
/odom (ground truth) ──┘                       ─► /belief (pose corregida)
                                               ─► /particles (PoseArray, debug)
                                               ─► /likelihood_field (opcional, debug)
```

Cada partícula representa una hipótesis de trayectoria y mantiene su propio mapa de grilla en log-odds.
Idea Rao-Blackwell: el filtro de partículas estima la trayectoria del robot; cada partícula construye su mapa condicionado a esa trayectoria.

Estado de cada partícula:

```text
particle = {
  pose:   (x, y, theta),
  weight: float,
  map:    grilla log-odds (H x W),
}
```

---

## 3. Decisiones técnicas iniciales

Todos los valores son parámetros, no se hardcodean (regla de AGENTS.md).

| Parámetro | Valor inicial | Razón |
|---|---|---|
| Resolución grilla | 0.05 m/celda | balance detalle vs cómputo |
| Tamaño mapa | ~12×12 m (240×240 celdas) | cubre `casa` con margen |
| N partículas | 30 al inicio | subir solo si el rendimiento lo permite |
| Clip log-odds | ±5 | evita saturaciones absurdas |
| Modelo de movimiento | deltas (δrot1, δtrans, δrot2) con ruido α1..α4 | se reutiliza el de TP5 `my_fastslam.py` |
| Pesado de partículas | likelihood field sobre endpoints del scan | método estándar (Thrun) |
| Resampling | systematic, sólo si `n_eff < N/2` | se reutiliza `systematic_resample` de TP5 |
| Mapa final | mapa de la partícula de mayor peso | evita promedios multimodales inconsistentes |

### Decisión sobre mensajes

Se decide **portar el paquete `custom_msgs` (DeltaOdom, Belief) de TP5 a `tpfinal/src/`**.
Motivo: la consigna exige que la entrega sea autocontenida y compile con `colcon build` sin dependencias externas no especificadas.

---

## 4. Plan por etapas

Cada etapa se valida antes de pasar a la siguiente.
No se implementa "todo FastSLAM" de una.

### Etapa 0 — Scaffolding del paquete

- Crear paquete `maze_slam` (ament_python) en `src/`.
- Portar `custom_msgs` desde TP5 a `tpfinal/src/`.
- Crear `slam.launch.py` y una config `.rviz` que muestre `/scan`, `/odom`, `/calc_odom`, `/map`.
- Verificar `colcon build --symlink-install` limpio.

Sin algoritmo todavía.

### Etapa 1 — Mapeo con pose conocida (gate de viabilidad)

- Nodo que construye UNA occupancy grid usando `/calc_odom` como pose conocida y `/scan`.
- Inverse sensor model + log-odds, reutilizando la lógica de `tp5/ejercicio1.py`.
- Ray casting tipo Bresenham: celdas libres hasta el impacto, celda ocupada en el endpoint.
- Publicar `/map` y guardar `maps/sim/casa.pgm` + `casa.yaml`.

Gate: si con pose conocida el mapa de `custom_casa` ya sale deforme, hay que corregir frames, ray casting, resolución o modelo inverso ANTES de agregar partículas.

### Etapa 2 — Partículas + likelihood field

- N partículas, cada una con su grilla propia.
- Predicción: motion model con deltas calculados de `/calc_odom`.
- Pesado: transformar los endpoints del scan al mundo con la pose de cada partícula y evaluar el likelihood field (distance transform del mapa de esa partícula).
- Actualización del mapa de cada partícula tras el pesado.
- Resampling con copia de mapas (parte computacionalmente cara).

### Etapa 3 — Salida y mejor partícula

- Publicar `/belief` (pose corregida), `/map` (mapa de la mejor partícula) y `/particles` (PoseArray para debug).
- Comparar en RViz las trayectorias `/odom` (real), `/belief` (SLAM) y `/calc_odom` (odometría pura).

### Etapa 4 — Rendimiento, Numba y GPU

Orden de optimización preferido:

1. Vectorizar en NumPy, sin loops sobre partículas ni rayos. Suele bastar.
2. **Numba** para los hot loops que no se vectorizan bien (ray casting Bresenham por partícula, actualización de log-odds por celda): decorar con `@numba.njit` (probar `parallel=True` para paralelizar sobre partículas en CPU).
3. GPU sólo si lo anterior no alcanza:
   - Kernels custom con `numba.cuda` para el ray casting y la actualización de mapas, o
   - mapas como tensor `(N, H, W)` en CuPy sobre la RTX 3090 (Ampere, mejor soportada que la RTX 5070 Blackwell), con distance transform vía `cupyx.scipy.ndimage.distance_transform_edt`.
   - En ambos casos, mantener los datos en GPU entre iteraciones para evitar transferencias host↔device.

Cuello de botella restante: copia de N mapas en resampling (memory-bound). Mitigar resampleando con menos frecuencia.

Numba, CuPy y/o PyTorch no están instalados todavía; instalar sólo lo que la Etapa 4 justifique.

### Etapa 5 — Mapa final y evidencia

- Exportar mapa optimizado para la Parte B, en formato compatible con navegación.
- Guardar capturas/video de RViz (mapa creciendo, cierre de lazo) y métricas de Hz y memoria.
- Documentar decisiones tomadas y limitaciones.

---

## 5. Reutilización de trabajo previo

- `tp5/ejercicio1.py`: inverse sensor model con log-odds, base de la Etapa 1.
- `tp5/.../my_fastslam.py`: estructura del nodo ROS, motion model con deltas, `systematic_resample`, patrón de markers/RViz. NO se reutiliza su parte de landmarks/EKF (eso es Opción 2).
- TP6: planificación sobre grilla, para la Parte B.

---

## 6. Riesgos y mitigaciones

- Deriva en giros (la odometría cruda ensucia el mapa): evaluar scan-matching local simple si el mapa se rompe en curvas.
- Particle depletion en loops largos: más partículas (habilitado por GPU) y ruido de movimiento bien calibrado.
- Resampling caro por copia de mapas: resamplear sólo cuando `n_eff` es bajo.
- No llegar a Parte B/C por perfeccionar la Parte A: definir un mapa suficientemente bueno, no perfecto.

---

## 7. Criterio de aceptación de la Parte A

Se considera la Opción 1 confirmada si se logra:

- generar un mapa reconocible de `custom_casa`;
- guardarlo y verlo en RViz;
- usarlo para planificar al menos un camino simple;
- mantener un tiempo de ejecución razonable.
