#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This script must be sourced: source ~/Medibot_openarm/source_all.sh" >&2
  exit 1
fi

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="${project_dir}/openarm_ws"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble." >&2
  return 1
fi

if [[ ! -f "${workspace_dir}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ${workspace_dir}/scripts/build.sh first." >&2
  return 1
fi

source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"

export RBM_OPENARM_ROOT="${project_dir}"
export RBM_OPENARM_WS="${workspace_dir}"
export LD_LIBRARY_PATH="/opt/apps/roboticsservice/SDK/x64:${LD_LIBRARY_PATH:-}"
