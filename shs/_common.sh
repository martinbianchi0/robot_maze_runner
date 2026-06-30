# Helpers para los scripts de shs/. Lo sourcean todos los demas.
# Define WS_DIR (la raiz del workspace = padre de shs/) y trae el setup de ROS + del workspace.

# Canonizamos con realpath para que sea siempre el mismo path sin importar
# desde que symlink/bindmount entraste. Asi el build no se re-genera por cambiar
# de path de acceso.
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
export WS_DIR

# El build/install va a DISCO LOCAL, NO al repo. El repo esta en la raid5 (/dev/md0),
# que corrompe archivos grandes Y artefactos de build (vimos un entry_points.txt de
# maze_slam con basura binaria -> ros2 launch fallaba con UnicodeDecodeError). En
# disco local los artefactos quedan sanos. Taggeamos por hash del path (host vs
# container ven el repo en paths distintos -> CMake no se pisa).
WS_TAG="$(printf '%s' "$WS_DIR" | md5sum | cut -c1-8)"
COLCON_LOCAL="${MAZE_COLCON_CACHE:-/var/tmp/maze_colcon}/$WS_TAG"
export BUILD_BASE="$COLCON_LOCAL/build"
export INSTALL_BASE="$COLCON_LOCAL/install"
export LOG_BASE="$COLCON_LOCAL/log"

# Limpiamos cualquier path heredado (~/.bashrc puede haber sourceado otro WS, o
# tener restos de ROS Humble en el container). Asi arrancamos siempre con un
# ambiente fresco y los avisos "not found: /opt/ros/humble/..." desaparecen.
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset ROS_PACKAGE_PATH
unset ROS_DISTRO

ROS_DISTRO_AUTO=""
for d in jazzy humble iron rolling; do
    if [[ -f "/opt/ros/$d/setup.bash" ]]; then
        ROS_DISTRO_AUTO="$d"
        break
    fi
done
if [[ -z "$ROS_DISTRO_AUTO" ]]; then
    echo "ERROR: no encuentro /opt/ros/<distro>/setup.bash" >&2
    return 1 2>/dev/null || exit 1
fi
source "/opt/ros/$ROS_DISTRO_AUTO/setup.bash"

if [[ -f "$INSTALL_BASE/local_setup.bash" ]]; then
    # local_setup.bash sourcea SOLO el workspace, sin perseguir parents que pudieron
    # haber estado en COLCON_PREFIX_PATH cuando se buildeo (ROS humble, otro WS, etc).
    source "$INSTALL_BASE/local_setup.bash"
fi

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

# Dejamos el user-site de python habilitado para que se vea numba (acelera el SLAM).
# OJO: no instalar un numpy >=2 en ~/.local, rompe scipy (ABI). El proyecto usa el
# numpy 1.26 del sistema. Si numba no esta, el codigo cae a Python puro igual.
