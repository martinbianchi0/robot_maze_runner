# Instalacion y dependencias

## Entorno

- ROS 2 Humble o Jazzy.
- Python del sistema usado por ROS.
- Paquetes Python usados por el repo: `numpy`, `scipy`, `pytest`, `setuptools`.
- Opcional para acelerar SLAM: `numba`.

Evitar instalar `numpy>=2` en `~/.local` si el entorno ROS usa `scipy` del
sistema con ABI de `numpy 1.x`.

## Build

Desde la raiz del workspace:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
./shs/build.sh
```

Build limpio:

```bash
./shs/build.sh --clean
```

El script usa bases de build/install/log fuera de la raiz versionada para evitar
mezclar artefactos locales con el repo.

## Robot real

Antes de usar TurtleBot4:

```bash
export ROS_DOMAIN_ID=<dominio_del_robot>
ros2 topic list | grep /tb4_0
```

Verificar que existan al menos `/<ns>/scan`, `/<ns>/odom`, `/<ns>/tf` y el topic
de camara esperado por el perfil de Parte C.
