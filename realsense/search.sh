#!/bin/bash
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

shell_type=${SHELL##*/}
shell_exec="exec $shell_type"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未找到 ROS 2 Humble: /opt/ros/humble/setup.bash" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash

if [[ ! -f "${script_dir}/install/setup.bash" ]]; then
  echo "RealSense 工作空间尚未构建，请先运行 ${script_dir}/build.sh" >&2
  exit 1
fi
source "${script_dir}/install/setup.bash"

gnome-terminal --title="realsense" -- "$shell_type" -c "ros2 run realsense2_camera list_camera_node; $shell_exec"