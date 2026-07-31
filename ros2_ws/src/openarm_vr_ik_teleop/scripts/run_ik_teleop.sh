#!/bin/bash
# 阶段 3 IK 遥操作节点：使用 conda pico_vr（含 placo）
# 用法: conda activate pico_vr && source <workspace>/install/setup.bash && 本脚本
set -e
if [ -z "$AMENT_PREFIX_PATH" ]; then
  _script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  _install_root="$(cd "$_script_dir/../../../.." && pwd)"
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  source "$_install_root/setup.bash"
fi
exec python3 -c "from openarm_vr_ik_teleop.ik_teleop_node import main; main()"
