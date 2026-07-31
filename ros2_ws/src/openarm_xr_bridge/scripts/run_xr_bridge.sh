#!/bin/bash
# 使用当前 shell 的 python3 运行 XR Bridge，规避 ros2 run 使用错误解释器的问题
# 用法: conda activate pico_vr && source <workspace>/install/setup.bash && 本脚本
set -e
# xrobotoolkit_sdk 需 libPXREARobotSDK.so，deb 安装在 /opt/apps/roboticsservice/SDK/x64
export LD_LIBRARY_PATH=/opt/apps/roboticsservice/SDK/x64:${LD_LIBRARY_PATH:-}
if [ -z "$AMENT_PREFIX_PATH" ]; then
  _script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  _install_root="$(cd "$_script_dir/../../../.." && pwd)"
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  source "$_install_root/setup.bash"
fi
exec python3 -c "from openarm_xr_bridge.xr_bridge_node import main; main()"
