#!/usr/bin/env bash
# Builda el workspace: turtlebot3_custom_simulation (catedra) + maze_slam (nuestro).
# Uso:  ./shs/build.sh           (incremental)
#       ./shs/build.sh --clean   (limpia el build/install de ESTE path antes)
#
# El build/install vive en .colcon/<hash-del-path>/ para que host y contenedor
# (que ven el mismo repo en paths distintos) no se pisen los artefactos de CMake.
set -e
source "$(dirname "$0")/_common.sh"
cd "$WS_DIR"

# Limpieza de artefactos viejos EN LA RAID5 (build/ install/ log/ en la raiz y el
# .colcon/ que antes estaba en el repo). Ahora el build va a disco local (ver
# _common.sh) porque la raid5 corrompe los artefactos. Borrar lo viejo es seguro.
for legacy in build install log .colcon; do
    if [[ -e "$WS_DIR/$legacy" ]]; then
        echo "Borrando build viejo en raid5: $WS_DIR/$legacy"
        rm -rf "${WS_DIR:?}/$legacy"
    fi
done

if [[ "${1:-}" == "--clean" || "${1:-}" == "-c" ]]; then
    echo "Limpiando $BUILD_BASE y $INSTALL_BASE..."
    rm -rf "$BUILD_BASE" "$INSTALL_BASE" "$LOG_BASE"
fi

# Forzamos el python del sistema (ROS usa 3.12). Si no, ament agarra el python3.11
# de ~/.local y no encuentra catkin_pkg.
SYS_PY="$(command -v python3.12 || command -v python3)"

colcon --log-base "$LOG_BASE" build --symlink-install \
    --build-base "$BUILD_BASE" \
    --install-base "$INSTALL_BASE" \
    --cmake-args "-DPython3_EXECUTABLE=$SYS_PY"

# Re-sourcear el install recien generado en esta shell.
source "$INSTALL_BASE/local_setup.bash"

echo ""
echo "Build OK -> $INSTALL_BASE"
