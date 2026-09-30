#!/bin/bash
# VR 遥操作 OpenArm 一键启动脚本
# 用法: ./launch_vr_teleop.sh [--fake|--real] [--bimanual|--unimanual] [--skip-check] [--no-xr-bridge]
# 默认: --fake --bimanual

set -e

USE_FAKE=true
BIMANUAL=true
SKIP_CHECK=false
START_XR_BRIDGE=true
for arg in "$@"; do
  case "$arg" in
    --real)   USE_FAKE=false ;;
    --fake)   USE_FAKE=true ;;
    --unimanual) BIMANUAL=false ;;
    --bimanual)  BIMANUAL=true ;;
    --skip-check) SKIP_CHECK=true ;;
    --no-xr-bridge) START_XR_BRIDGE=false ;;
    -h|--help)
      echo "用法: $0 [--fake|--real] [--bimanual|--unimanual] [--skip-check] [--no-xr-bridge]"
      echo "  --fake      假硬件（默认）"
      echo "  --real      真硬件"
      echo "  --bimanual  双臂（默认）"
      echo "  --unimanual 单臂"
      echo "  --skip-check  跳过启动前准备工作确认"
      echo "  --no-xr-bridge  不启动 xr_bridge（由外部统一启动，便于与底盘 VR 共用）"
      exit 0
      ;;
  esac
done

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_WS="$(cd "$_SCRIPT_DIR/../.." && pwd)"
_REPO="$(cd "$_WS/.." && pwd)"          # 仓库根目录（openarm_env 若存在则在根目录）
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
  echo "  3. openarm 环境已激活"
  echo "     conda activate openarm  或  source ~/Medibot_openarm/openarm_env/bin/activate"
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

# openarm 环境（gnome-terminal 非交互 shell 需显式加载 conda hook）
_activate_openarm() {
  if [ "${CONDA_DEFAULT_ENV:-}" = "openarm" ]; then
    return 0
  fi
  if [ -f "$_REPO/openarm_env/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$_REPO/openarm_env/bin/activate"
    return 0
  fi
  local _conda_sh=""
  for _conda_sh in \
    "$HOME/miniconda3/etc/profile.d/conda.sh" \
    "$HOME/anaconda3/etc/profile.d/conda.sh" \
    "/opt/conda/etc/profile.d/conda.sh"; do
    if [ -f "$_conda_sh" ]; then
      # shellcheck disable=SC1090
      source "$_conda_sh"
      conda activate openarm
      return $?
    fi
  done
  if command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook)"
    conda activate openarm
    return $?
  fi
  echo "[ERROR] 请先激活 openarm 环境，或确保 ~/Medibot_openarm/openarm_env 存在"
  echo "  conda activate openarm  或  source ~/Medibot_openarm/openarm_env/bin/activate"
  return 1
}

_activate_openarm || exit 1

# 子进程继承当前已激活环境；显式使用 conda python，避免落到系统 python3
_PYTHON="${CONDA_PREFIX:-}/bin/python3"
if [ ! -x "$_PYTHON" ]; then
  _PYTHON="$(command -v python3)"
fi

echo "[INFO] Python: $_PYTHON"
if ! "$_PYTHON" -c "import xrobotoolkit_sdk, placo" 2>/dev/null; then
  echo "[ERROR] 当前 Python 缺少 xrobotoolkit_sdk 或 placo。"
  echo "  请在 openarm 环境中安装 VR 依赖（勿用 setup_conda.sh --install，会拉 torch/CUDA）："
  echo "    conda activate openarm"
  echo "    cd ~/Medibot_openarm/third_party/XRoboToolkit-PC-Service-Pybind"
  echo "    bash setup_ubuntu.sh"
  echo "    pip install placo"
  exit 1
fi

_FAKE_ARG="true"
$USE_FAKE || _FAKE_ARG="false"

# 清理函数：先停 IK 发布，再独占回 home，最后停 bringup。
# 不可在 IK 仍发布 hands_up 时并行 go_home，否则会剧烈抖动并停在 hands_up。
_cleanup() {
  trap - SIGINT SIGTERM EXIT
  echo "[INFO] 正在关闭 VR 遥操作（机械臂将回到 home）..."
  if [ -n "${_IK_PID:-}" ] && kill -0 "${_IK_PID}" 2>/dev/null; then
    kill -TERM "${_IK_PID}" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "${_IK_PID}" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL "${_IK_PID}" 2>/dev/null || true
  fi
  if [ -n "${_BRINGUP_PID:-}" ] && kill -0 "${_BRINGUP_PID}" 2>/dev/null; then
    if [ ! -f "/tmp/rbm-openarm-control-${UID}/home_done" ]; then
      bash "${_SCRIPT_DIR}/go_home_arms.sh" || true
    fi
  fi
  kill ${_BRINGUP_PID:-} ${_XR_PID:-} ${_IK_PID:-} 2>/dev/null || true
  wait ${_BRINGUP_PID:-} ${_XR_PID:-} ${_IK_PID:-} 2>/dev/null || true
  exit 0
}
trap _cleanup SIGINT SIGTERM EXIT

# 0. 显示坐标系（供排查 VR 与机械臂对准）
echo "[INFO] 坐标系信息："
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$_INSTALL/setup.bash" 2>/dev/null \
  && "$_PYTHON" -c "from openarm_vr_ik_teleop.print_frames import main; main()" 2>/dev/null \
  || echo "  (跳过，需先 colcon build)"
echo ""

# 1. bringup
echo "[INFO] 启动 bringup (use_fake_hardware=$_FAKE_ARG, bimanual=$BIMANUAL)..."
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$_INSTALL/setup.bash"
# RViz 改由菜单 9（相机）统一打开，避免与三相机窗口重复
if $BIMANUAL; then
  ros2 launch openarm_bringup openarm.bimanual.launch.py \
    arm_type:=v10 use_fake_hardware:=$_FAKE_ARG \
    robot_controller:=forward_position_controller launch_rviz:=false &
else
  ros2 launch openarm_bringup openarm.launch.py \
    arm_type:=v10 use_fake_hardware:=$_FAKE_ARG \
    robot_controller:=forward_position_controller launch_rviz:=false &
fi
_BRINGUP_PID=$!

sleep 5

# 2. xr_bridge（可选；菜单 5/8 共用外部 bridge，避免重复 init SDK）
if $START_XR_BRIDGE; then
  echo "[INFO] 启动 xr_bridge..."
  (
    export LD_LIBRARY_PATH=/opt/apps/roboticsservice/SDK/x64:${LD_LIBRARY_PATH:-}
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    source "$_INSTALL/setup.bash"
    exec "$_PYTHON" -c "from openarm_xr_bridge.xr_bridge_node import main; main()"
  ) &
  _XR_PID=$!
  sleep 2
else
  echo "[INFO] 跳过 xr_bridge（--no-xr-bridge；请确保外部已启动）"
  sleep 1
fi

# 3. ik_teleop（前台，Ctrl+C 结束全部）
echo "[INFO] 启动 ik_teleop（Ctrl+C 结束）..."
(
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  source "$_INSTALL/setup.bash"
  exec "$_PYTHON" -c "from openarm_vr_ik_teleop.ik_teleop_node import main; main()"
) &
_IK_PID=$!
wait $_IK_PID
