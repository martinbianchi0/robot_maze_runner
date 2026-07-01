#!/usr/bin/env bash
# Helper compartido de la Parte C: activa el entorno conda 'rosenv' (donde vive
# ROS2 en esta maquina) y sourcea el workspace. Sourcear desde la raiz del repo:
#   source scripts/parte_c_env.sh
#
# Overridables: CONDA_BASE (default ~/miniforge3), ROSENV_NAME (default rosenv).

CONDA_BASE="${CONDA_BASE:-$HOME/miniforge3}"
if [ -f "$CONDA_BASE/etc/profile.d/conda.sh" ]; then
  # shellcheck disable=SC1091
  source "$CONDA_BASE/etc/profile.d/conda.sh"
else
  echo "parte_c_env: no encontre conda en $CONDA_BASE (setea CONDA_BASE)" >&2
fi

conda activate "${ROSENV_NAME:-rosenv}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

# Overlay del workspace (scripts corren en bash -> setup.bash).
if [ -f "install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source install/setup.bash
fi
