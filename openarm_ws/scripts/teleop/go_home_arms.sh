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

# Humble rclpy 只提供 Python 3.10。菜单常从 conda base(3.13) 启动，
# 不能用 PATH 上的 python3，必须切到 openarm 或系统 3.10。
_PYTHON=""
_conda_sh=""
for _conda_sh in \
  "${HOME}/anaconda3/etc/profile.d/conda.sh" \
  "${HOME}/miniconda3/etc/profile.d/conda.sh" \
  "/opt/conda/etc/profile.d/conda.sh"; do
  if [[ -f "${_conda_sh}" ]]; then
    # shellcheck disable=SC1090
    source "${_conda_sh}"
    while [[ -n "${CONDA_PREFIX:-}" && "${CONDA_DEFAULT_ENV:-}" != "openarm" ]]; do
      conda deactivate || break
    done
    conda activate openarm 2>/dev/null || true
    break
  fi
done

if [[ "${CONDA_DEFAULT_ENV:-}" == "openarm" && -x "${CONDA_PREFIX}/bin/python3" ]]; then
  _PYTHON="${CONDA_PREFIX}/bin/python3"
elif [[ -x "${project_dir}/openarm_env/bin/python3" ]]; then
  _PYTHON="${project_dir}/openarm_env/bin/python3"
elif [[ -x /usr/bin/python3 ]]; then
  _PYTHON="/usr/bin/python3"
else
  _PYTHON="$(command -v python3)"
fi
set -u

echo "[INFO] Python: ${_PYTHON} ($("${_PYTHON}" --version 2>&1))"
exec "${_PYTHON}" "${script_dir}/go_home_arms.py"
