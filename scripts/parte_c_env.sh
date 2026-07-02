#!/usr/bin/env bash
# Helper compartido de la Parte C: activa el entorno conda 'rosenv' (donde vive
# ROS2 en esta maquina) y sourcea el workspace. Sourcear desde la raiz del repo:
#   source scripts/parte_c_env.sh
#
# Overridables: CONDA_BASE (default ~/miniforge3), ROSENV_NAME (default rosenv).

# Los scripts de activacion de conda y el setup.bash de ROS referencian variables
# no seteadas; con 'set -u' del caller romperian. Guardar y desactivar nounset
# alrededor del sourcing, y restaurarlo despues.
case $- in *u*) _RESTORE_U=1 ;; *) _RESTORE_U=0 ;; esac
set +u

CONDA_BASE="${CONDA_BASE:-$HOME/miniforge3}"
if [ -f "$CONDA_BASE/etc/profile.d/conda.sh" ]; then
  # shellcheck disable=SC1091
  source "$CONDA_BASE/etc/profile.d/conda.sh"
  conda activate "${ROSENV_NAME:-rosenv}"
else
  echo "parte_c_env: no encontre conda en $CONDA_BASE; uso ROS del sistema" >&2
  if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  elif ! command -v ros2 >/dev/null 2>&1; then
    echo "parte_c_env: no encontre ros2; setea CONDA_BASE o sourcea ROS" >&2
    [ "$_RESTORE_U" = 1 ] && set -u
    unset _RESTORE_U
    return 1 2>/dev/null || exit 1
  fi
fi

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

# Overlay del workspace (scripts corren en bash -> setup.bash).
WORKSPACE_SETUP="${WORKSPACE_SETUP:-install/setup.bash}"
if [ -f "$WORKSPACE_SETUP" ]; then
  # shellcheck disable=SC1091
  source "$WORKSPACE_SETUP"
fi

[ "$_RESTORE_U" = 1 ] && set -u
unset _RESTORE_U
