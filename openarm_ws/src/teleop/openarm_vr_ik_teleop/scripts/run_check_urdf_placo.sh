#!/bin/bash
# 阶段 2 检查脚本：使用当前 python3（需 conda openarm，含 placo）运行 URDF 加载验证
# 用法: conda activate openarm && source <workspace>/install/setup.bash && 本脚本
set -e
if [ -z "$AMENT_PREFIX_PATH" ]; then
  _script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  _install_root="$(cd "$_script_dir/../../../.." && pwd)"
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  source "$_install_root/setup.bash"
fi
exec python3 -c "from openarm_vr_ik_teleop.check_urdf_placo import main; main()"
