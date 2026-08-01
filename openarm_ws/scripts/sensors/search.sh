#!/bin/bash
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/../.." && pwd)"

shell_type=${SHELL##*/}
shell_exec="exec $shell_type"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未找到 ROS 2 Humble: /opt/ros/humble/setup.bash" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash

if [[ ! -f "${workspace_dir}/install/setup.bash" ]]; then
  echo "OpenArm 工作空间尚未构建，请先运行 ${workspace_dir}/scripts/build.sh" >&2
  exit 1
fi
source "${workspace_dir}/install/setup.bash"

gnome-terminal --title="realsense" -- "$shell_type" -c "ros2 run realsense2_camera list_camera_node; $shell_exec"