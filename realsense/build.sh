#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

# ROS Humble's rosidl stack needs the system Python/empy pair.
# The conda environment in this machine ships a newer empy that breaks interface generation.
unset PYTHONHOME PYTHONPATH
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_EXE CONDA_PYTHON_EXE
export PATH="/usr/bin:/bin:${PATH}"

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

exec /usr/bin/colcon build \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-force-configure \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  "$@"
