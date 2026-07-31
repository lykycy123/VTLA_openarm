#!/bin/bash
# VR 遥操作 OpenArm 一键启动脚本
# 用法: ./launch_vr_teleop.sh [--fake|--real] [--bimanual|--unimanual] [--skip-check]
# 默认: --fake --bimanual

set -e

USE_FAKE=true
BIMANUAL=true
SKIP_CHECK=false
for arg in "$@"; do
  case "$arg" in
    --real)   USE_FAKE=false ;;
    --fake)   USE_FAKE=true ;;
    --unimanual) BIMANUAL=false ;;
    --bimanual)  BIMANUAL=true ;;
    --skip-check) SKIP_CHECK=true ;;
    -h|--help)
      echo "用法: $0 [--fake|--real] [--bimanual|--unimanual] [--skip-check]"
      echo "  --fake      假硬件（默认）"
      echo "  --real      真硬件"
      echo "  --bimanual  双臂（默认）"
      echo "  --unimanual 单臂"
      echo "  --skip-check  跳过启动前准备工作确认"
      exit 0
      ;;
  esac
done

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_WS="$(cd "$_SCRIPT_DIR/.." && pwd)"
_INSTALL="$_WS/install"

# 启动前准备工作提示
if ! $SKIP_CHECK; then
  echo ""
  echo "=========================================="
  echo "  VR 遥操作 OpenArm — 启动前请确认"
  echo "=========================================="
  echo ""
  echo "准备工作："
  echo "  1. XRoboToolkit PC Service 已启动"
  echo "     bash /opt/apps/roboticsservice/runService.sh"
  echo ""
  echo "  2. PICO 头显已开机、已安装 APK，与 PC 同网"
  echo ""
  echo "  3. pico_vr 环境已激活"
  echo "     conda activate pico_vr  或  source ~/RBM_openarm/pico_vr_env/bin/activate"
  echo ""
  if ! $USE_FAKE; then
    echo "  4. [真硬件] CAN 接口已配置，电机已使能"
    echo ""
  fi
  echo "------------------------------------------"
  read -p "以上均已完成？按 Enter 继续，Ctrl+C 取消："
  echo ""
fi

# libPXREARobotSDK.so
export LD_LIBRARY_PATH=/opt/apps/roboticsservice/SDK/x64:${LD_LIBRARY_PATH:-}

# pico_vr 环境
if [ -n "$CONDA_DEFAULT_ENV" ] && [ "$CONDA_DEFAULT_ENV" = "pico_vr" ]; then
  _ACTIVATE=""
elif [ -f "$HOME/RBM_openarm/pico_vr_env/bin/activate" ]; then
  _ACTIVATE="source $HOME/RBM_openarm/pico_vr_env/bin/activate && "
elif conda env list | grep -q "pico_vr"; then
  _ACTIVATE="conda activate pico_vr && "
else
  echo "[ERROR] 请先激活 pico_vr 环境，或确保 ~/RBM_openarm/pico_vr_env 存在"
  echo "  conda activate pico_vr  或  source ~/RBM_openarm/pico_vr_env/bin/activate"
  exit 1
fi

_FAKE_ARG="true"
$USE_FAKE || _FAKE_ARG="false"

# 清理函数
_cleanup() {
  echo "[INFO] 正在关闭..."
  kill ${_BRINGUP_PID:-} ${_XR_PID:-} ${_IK_PID:-} 2>/dev/null
  wait ${_BRINGUP_PID:-} ${_XR_PID:-} ${_IK_PID:-} 2>/dev/null
  exit 0
}
trap _cleanup SIGINT SIGTERM EXIT

# 0. 显示坐标系（供排查 VR 与机械臂对准）
echo "[INFO] 坐标系信息："
eval "${_ACTIVATE}source $_INSTALL/setup.bash 2>/dev/null" && python3 -c "from openarm_vr_ik_teleop.print_frames import main; main()" 2>/dev/null || echo "  (跳过，需先 colcon build)"
echo ""

# 1. bringup
echo "[INFO] 启动 bringup (use_fake_hardware=$_FAKE_ARG, bimanual=$BIMANUAL)..."
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$_INSTALL/setup.bash"
if $BIMANUAL; then
  ros2 launch openarm_bringup openarm.bimanual.launch.py \
    arm_type:=v10 use_fake_hardware:=$_FAKE_ARG robot_controller:=forward_position_controller &
else
  ros2 launch openarm_bringup openarm.launch.py \
    arm_type:=v10 use_fake_hardware:=$_FAKE_ARG robot_controller:=forward_position_controller &
fi
_BRINGUP_PID=$!

sleep 5

# 2. xr_bridge
echo "[INFO] 启动 xr_bridge..."
eval "${_ACTIVATE}source $_INSTALL/setup.bash && $_INSTALL/openarm_xr_bridge/share/openarm_xr_bridge/scripts/run_xr_bridge.sh" &
_XR_PID=$!

sleep 2

# 3. ik_teleop（前台，Ctrl+C 结束全部）
echo "[INFO] 启动 ik_teleop（Ctrl+C 结束）..."
(eval "${_ACTIVATE}source $_INSTALL/setup.bash" && python3 -c "from openarm_vr_ik_teleop.ik_teleop_node import main; main()") &
_IK_PID=$!
wait $_IK_PID
