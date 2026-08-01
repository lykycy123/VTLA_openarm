#!/usr/bin/env bash
# 在机械臂 forward_position 驱动仍运行时回 home。
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws_dir="$(cd "${script_dir}/../.." && pwd)"
project_dir="$(cd "${ws_dir}/.." && pwd)"

# ROS setup.bash 会读未定义变量
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ws_dir}/install/setup.bash"

_PYTHON=""
if [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
  # shellcheck disable=SC1091
  source "${HOME}/miniconda3/etc/profile.d/conda.sh"
  conda activate openarm 2>/dev/null || true
fi
if [[ -n "${CONDA_PREFIX:-}" && -x "${CONDA_PREFIX}/bin/python3" ]]; then
  _PYTHON="${CONDA_PREFIX}/bin/python3"
elif [[ -f "${project_dir}/openarm_env/bin/python3" ]]; then
  _PYTHON="${project_dir}/openarm_env/bin/python3"
else
  _PYTHON="$(command -v python3)"
fi
set -u

echo "[INFO] 关闭前将双臂回到 home（保持驱动运行）..."
exec "${_PYTHON}" "${script_dir}/go_home_arms.py"
