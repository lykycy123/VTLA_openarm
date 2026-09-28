#!/usr/bin/env bash

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
script_path="${script_dir}/$(basename "${BASH_SOURCE[0]}")"
project_dir="$(cd "${script_dir}/../.." && pwd)"
workspace_dir="${project_dir}/openarm_ws"
state_dir="/tmp/rbm-openarm-control-${UID}"
shell_type="${SHELL##*/}"
[[ -n "${shell_type}" ]] || shell_type=bash

usage() {
  cat <<'EOF'
用法：control_menu.sh

快速启动：
  11 首次启动
  12 全身 VR 遥操作
  13 数据采集
  14 数据回放

分布步骤：
  1  安装 CAN 绑定（仅首次）
  2  启动全部 CAN（can0/can1/can2）
  3  启动 OpenArm 驱动
  4  启动 OpenArm MoveIt 控制（需先启动 3）
  5  启动 OpenArm VR 遥操作（与 4 冲突）
  6  启动底盘控制
  7  启动底盘键盘控制（需先启动 6）
  8  启动底盘 VR 遥操作
  9  启动 camera+rviz 可视化
  0  关闭本菜单启动的全部终端并退出
EOF
}

keep_terminal_open() {
  local status="${1:-1}"
  echo
  echo "[错误] 组件已退出，退出码=${status}"
  read -r -p "按 Enter 关闭此终端..."
  exit "${status}"
}

# gnome-terminal 会继承菜单进程的 conda base（常为 Python 3.13）。
# ROS Humble 的 rclpy 只提供 cp310，必须在 source ROS 前退回 3.10。
leave_conda_if_not_openarm() {
  if [[ -z "${CONDA_PREFIX:-}" || "${CONDA_DEFAULT_ENV:-}" == "openarm" ]]; then
    return 0
  fi

  local conda_sh=""
  for conda_sh in \
    "${HOME}/anaconda3/etc/profile.d/conda.sh" \
    "${HOME}/miniconda3/etc/profile.d/conda.sh" \
    "/opt/conda/etc/profile.d/conda.sh"; do
    if [[ -f "${conda_sh}" ]]; then
      # shellcheck disable=SC1090
      source "${conda_sh}"
      break
    fi
  done

  while [[ -n "${CONDA_PREFIX:-}" && "${CONDA_DEFAULT_ENV:-}" != "openarm" ]]; do
    conda deactivate || break
  done

  local pyver=""
  pyver="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")' 2>/dev/null || true)"
  if [[ "${pyver}" != "3.10" ]]; then
    unset PYTHONHOME PYTHONPATH
    export PATH="/usr/bin:/bin:${PATH}"
  fi
}

load_ros_environment() {
  if [[ ! -f "${project_dir}/source_all.sh" ]]; then
    echo "[错误] 未找到 ${project_dir}/source_all.sh" >&2
    return 1
  fi

  # ROS setup.bash 会读未定义变量，不能在 set -u 下 source。
  set +u
  leave_conda_if_not_openarm
  # shellcheck disable=SC1091
  source "${project_dir}/source_all.sh"
}

wait_for_can_interface() {
  local iface="${1:-can2}"
  local flags=""
  local i=0
  # SocketCAN 的 operstate 经常是 UNKNOWN，不能用 `state UP` 判断。
  for i in {1..50}; do
    if [[ -f "/sys/class/net/${iface}/flags" ]]; then
      flags="$(cat "/sys/class/net/${iface}/flags" 2>/dev/null || echo 0)"
      if (( flags & 0x1 )); then
        echo "[INFO] ${iface} 已就绪。"
        return 0
      fi
    fi
    sleep 0.2
  done
  echo "[错误] 等待 ${iface} 就绪超时（约 10s）。请确认 CAN tab 已成功启动。" >&2
  return 1
}

activate_openarm_environment() {
  if [[ "${CONDA_DEFAULT_ENV:-}" == "openarm" ]]; then
    return 0
  fi

  if [[ -f "${project_dir}/openarm_env/bin/activate" ]]; then
    # shellcheck disable=SC1091
    source "${project_dir}/openarm_env/bin/activate"
    return 0
  fi

  local conda_sh=""
  for conda_sh in \
    "${HOME}/anaconda3/etc/profile.d/conda.sh" \
    "${HOME}/miniconda3/etc/profile.d/conda.sh" \
    "/opt/conda/etc/profile.d/conda.sh"; do
    if [[ -f "${conda_sh}" ]]; then
      # shellcheck disable=SC1090
      source "${conda_sh}"
      conda activate openarm
      return $?
    fi
  done

  if command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook)"
    conda activate openarm
    return $?
  fi

  echo "[错误] 未找到 openarm Conda 环境或 ${project_dir}/openarm_env。" >&2
  return 1
}

can_bringup_script="${script_dir}/can/bringup_can.sh"
can_bindings_script="${script_dir}/can/install_can_bindings.sh"
can_passwordless_script="${script_dir}/can/install_passwordless_sudo.sh"

can_sudo_ok() {
  sudo -n "${can_bringup_script}" --help >/dev/null 2>&1
}

ensure_can_passwordless_sudo() {
  if can_sudo_ok; then
    return 0
  fi
  echo "[INFO] CAN 启动需要 sudo。将安装免密规则（只需输入一次密码，之后菜单 1/2/11–14 不再询问）。"
  if ! bash "${can_passwordless_script}"; then
    echo "[错误] 免密 sudo 未配置成功。新开的 CAN 终端无法交互输入密码。" >&2
    return 1
  fi
  if ! can_sudo_ok; then
    echo "[错误] 免密规则已写入，但 sudo -n 仍失败。请新开一个终端后再试。" >&2
    return 1
  fi
  echo "[INFO] CAN sudo 免密已生效。"
  return 0
}

run_component() {
  local component="${1:-}"
  local pid_file="${state_dir}/${component}.pid"
  local status=0

  mkdir -p "${state_dir}"
  # 若外层启动脚本已写入 PID，则保留；否则记录当前进程。
  if [[ ! -f "${pid_file}" ]]; then
    printf '%s\n' "$$" >"${pid_file}"
  fi

  case "${component}" in
    can_install)
      echo "[INFO] 安装 CAN 持久绑定（udev，通常只需一次）..."
      sudo -n "${can_bindings_script}" || status=$?
      if (( status == 0 )); then
        echo
        echo "[INFO] CAN 绑定安装完成。日常重启或插拔后请执行菜单 2。"
        read -r -p "按 Enter 关闭此终端..."
      else
        keep_terminal_open "${status}"
      fi
      ;;

    can_bringup)
      echo "[INFO] 启动全部 CAN：can0/can1（OpenArm）+ can2（底盘）..."
      sudo -n "${can_bringup_script}" --all || status=$?
      if (( status == 0 )); then
        echo
        echo "[INFO] CAN 已就绪。可继续菜单 3/6 启动 OpenArm 或底盘驱动。"
        read -r -p "按 Enter 关闭此终端..."
      else
        keep_terminal_open "${status}"
      fi
      ;;

    arm_bringup)
      load_ros_environment || keep_terminal_open 1
      echo "[INFO] 启动真实双臂硬件驱动（openarm bringup）..."
      echo "[INFO] 请确认已先通过菜单 2 启动 CAN。"
      ros2 launch openarm_bringup openarm.bimanual.launch.py \
        arm_type:=v10 \
        use_fake_hardware:=false \
        robot_controller:=joint_trajectory_controller \
        right_can_interface:=can0 \
        left_can_interface:=can1 \
        launch_rviz:=false || status=$?
      ;;

    arm_moveit)
      load_ros_environment || keep_terminal_open 1
      echo "[INFO] 启动 MoveIt 2（move_group + RViz）..."
      echo "[INFO] 请确认已先通过菜单 3 启动机械臂驱动。"
      ros2 launch openarm_bimanual_moveit_config move_group.launch.py &
      local move_group_pid=$!
      sleep 2
      ros2 launch openarm_bimanual_moveit_config moveit_rviz.launch.py || status=$?
      kill "${move_group_pid}" 2>/dev/null || true
      wait "${move_group_pid}" 2>/dev/null || true
      ;;

    pc_service)
      local rs_dir="/opt/apps/roboticsservice"
      # 进程名超过 15 字符会被截断，pgrep -x RoboticsServiceProcess 匹配不到。
      if [[ ! -x "${rs_dir}/RoboticsServiceProcess" ]]; then
        echo "[错误] 未找到 ${rs_dir}/RoboticsServiceProcess" >&2
        keep_terminal_open 1
      fi
      echo "[INFO] 启动 XRoboToolkit PC Service..."
      # 不要再开桌面那个 Unity 窗口（run3D.sh / RobotLinuxDemo）。
      # 它和 xr_bridge 都连本机 60061，窗口会把头显数据独占走，
      # 机械臂侧位姿和握把会一直是 0。PICO 连上后台服务即可。
      export LD_LIBRARY_PATH="${rs_dir}:${rs_dir}/lib:${rs_dir}/SDK/x64:${LD_LIBRARY_PATH:-}"
      export QT_PLUGIN_PATH="${rs_dir}/plugins/:${QT_PLUGIN_PATH:-}"
      export QT_QML_PATH="${rs_dir}/qml/:${QT_QML_PATH:-}"
      cd "${rs_dir}"
      if pgrep -f '[/]RoboticsServiceProcess' >/dev/null 2>&1; then
        echo "[INFO] RoboticsServiceProcess 已在运行，本终端保持占用。"
        while pgrep -f '[/]RoboticsServiceProcess' >/dev/null 2>&1; do
          sleep 2
        done
        status=1
      else
        exec ./RoboticsServiceProcess || keep_terminal_open $?
      fi
      ;;

    arm_vr)
      load_ros_environment || keep_terminal_open 1
      activate_openarm_environment || keep_terminal_open 1
      echo "[INFO] 启动真实双臂 VR 遥操作（使用外部 XR bridge）..."
      "${script_dir}/teleop/launch_vr_teleop.sh" \
        --real --bimanual --skip-check --no-xr-bridge || status=$?
      ;;

    chassis_driver)
      load_ros_environment || keep_terminal_open 1
      echo "[INFO] Python: $(command -v python3) ($(python3 --version 2>&1))"
      echo "[INFO] 启动 Ranger Mini 3.0 底盘驱动（can2）..."
      wait_for_can_interface can2 || keep_terminal_open 1
      ros2 launch ranger_bringup ranger_mini_v3.launch.py port_name:=can2 || status=$?
      ;;

    chassis_keyboard)
      load_ros_environment || keep_terminal_open 1
      echo "[INFO] 启动底盘键盘控制（发布 /cmd_vel）..."
      echo "[INFO] 请确认已先通过菜单 6 启动底盘驱动。"
      echo "[INFO] 请保持本终端焦点：i/k 前进/后退，j/l 转向，其他键停止，Ctrl+C 退出。"
      ros2 run teleop_twist_keyboard teleop_twist_keyboard || status=$?
      ;;

    xr_bridge)
      load_ros_environment || keep_terminal_open 1
      activate_openarm_environment || keep_terminal_open 1
      # 与 PC Service 同窗口并行启动时多等一会
      sleep 3
      echo "[INFO] 启动 XR bridge..."
      "${workspace_dir}/install/openarm_xr_bridge/share/openarm_xr_bridge/scripts/run_xr_bridge.sh" || status=$?
      ;;

    chassis_vr)
      load_ros_environment || keep_terminal_open 1
      activate_openarm_environment || keep_terminal_open 1
      # 等待同窗口中的 XR Bridge 起来
      sleep 4
      echo "[INFO] 启动底盘 VR 摇杆控制（订阅 /vr/controller_input）..."
      ros2 launch rbm_base_control base_controller.launch.py || status=$?
      ;;

    camera_l)
      load_ros_environment || keep_terminal_open 1
      echo "[INFO] 启动左腕 D405..."
      # D405 无 RGB。对齐头部 D435I：brightness=10，关 AE，曝光/增益按现场亮度对齐头部。
      ros2 launch realsense2_camera rs_launch.py \
        camera_name:=camera_l \
        depth_module.color_profile:=640x480x30 \
        depth_module.enable_auto_exposure:=false \
        depth_module.exposure:=10000 \
        depth_module.gain:=32 \
        depth_module.brightness:=10 \
        enable_depth:=false \
        initial_reset:=true \
        serial_no:=_260322278361 || status=$?
      ;;

    camera_h)
      load_ros_environment || keep_terminal_open 1
      sleep 1
      echo "[INFO] 启动胸前 D435I..."
      ros2 launch realsense2_camera rs_launch.py \
        camera_name:=camera_h \
        rgb_camera.color_profile:=1280x720x30 \
        rgb_camera.brightness:=10 \
        enable_depth:=false \
        initial_reset:=true \
        serial_no:=_134222076235 || status=$?
      ;;

    camera_r)
      load_ros_environment || keep_terminal_open 1
      sleep 2
      echo "[INFO] 启动右腕 D405..."
      # D405 无 RGB。对齐头部 D435I：brightness=10，关 AE，曝光/增益按现场亮度对齐头部。
      ros2 launch realsense2_camera rs_launch.py \
        camera_name:=camera_r \
        depth_module.color_profile:=640x480x30 \
        depth_module.enable_auto_exposure:=false \
        depth_module.exposure:=10000 \
        depth_module.gain:=32 \
        depth_module.brightness:=10 \
        enable_depth:=false \
        initial_reset:=true \
        serial_no:=_260322276489 || status=$?
      ;;

    camera_rviz)
      load_ros_environment || keep_terminal_open 1
      sleep 3
      echo "[INFO] 启动统一 RViz（三相机画面 + 机械臂 RobotModel）..."
      echo "[INFO] 机械臂模型需先启动菜单 3 或 5（发布 /robot_description）。"
      rviz2 -d "${script_dir}/sensors/rviz/camera.rviz" || status=$?
      ;;

    data_collection)
      load_ros_environment || keep_terminal_open 1
      activate_openarm_environment || keep_terminal_open 1
      # 快速启动 13 时相机/VR 同窗并行启动，稍等话题就绪
      sleep 5
      echo "[INFO] 启动数据采集（相机 + 关节/末端 + odom/cmd_vel）..."
      echo "[INFO] 请确认已启动相机(9)、机械臂(3或5)、底盘(6/7/8，可选)。"
      echo "[INFO] SPACE=开始/停止录制并保存，ESC=退出。"
      python3 "${project_dir}/data_collection/data_collection.py" \
        --config "${project_dir}/data_collection/config.yaml" \
        --datasets-dir "${project_dir}/datasets" || status=$?
      ;;

    arm_bringup_fp)
      load_ros_environment || keep_terminal_open 1
      echo "[INFO] 启动双臂 forward_position 驱动（供数据回放）..."
      echo "[INFO] 请确认已先通过菜单 2 启动 CAN。"
      ros2 launch openarm_bringup openarm.bimanual.launch.py \
        arm_type:=v10 \
        use_fake_hardware:=false \
        robot_controller:=forward_position_controller \
        right_can_interface:=can0 \
        left_can_interface:=can1 \
        launch_rviz:=false || status=$?
      ;;

    data_replay)
      load_ros_environment || keep_terminal_open 1
      activate_openarm_environment || keep_terminal_open 1
      # 快速启动 14 时 FP/底盘同窗并行启动，稍等控制器就绪
      sleep 5
      echo "[INFO] 启动数据回放（HDF5 → 臂/夹爪/可选底盘）..."
      echo "[INFO] SPACE=播放/暂停，ESC=退出（默认回 home）。"
      echo "[INFO] 数据目录：${project_dir}/datasets"
      python3 "${project_dir}/data_collection/replay.py" \
        --config "${project_dir}/data_collection/config.yaml" \
        --datasets-dir "${project_dir}/datasets" || status=$?
      ;;

    *)
      echo "[错误] 未知组件：${component}" >&2
      keep_terminal_open 2
      ;;
  esac

  if (( status != 0 )); then
    keep_terminal_open "${status}"
  fi
}

pid_is_running() {
  local pid="${1:-}"
  [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null
}

component_is_running() {
  local component="$1"
  local pid_file="${state_dir}/${component}.pid"
  local pid=""

  [[ -f "${pid_file}" ]] || return 1
  read -r pid <"${pid_file}" || true

  if pid_is_running "${pid}"; then
    return 0
  fi

  rm -f "${pid_file}"
  return 1
}

wait_for_component_pid() {
  local component="$1"
  local title="$2"
  local pid_file="${state_dir}/${component}.pid"
  local pid=""

  for _ in {1..50}; do
    if [[ -f "${pid_file}" ]]; then
      read -r pid <"${pid_file}" || true
      if pid_is_running "${pid}"; then
        echo "[已启动] ${title}（PID=${pid}）"
        return 0
      fi
    fi
    sleep 0.1
  done

  echo "[错误] ${title} 未能保持运行，请查看弹出终端中的报错。" >&2
  return 1
}

start_terminal() {
  local component="$1"
  local title="$2"
  local pid_file="${state_dir}/${component}.pid"
  local title_file="${state_dir}/${component}.title"

  # 清理失效 PID，避免误报“已启动”
  component_is_running "${component}" >/dev/null || true

  if component_is_running "${component}"; then
    echo "[跳过] ${title} 已在运行。"
    return 0
  fi

  rm -f "${pid_file}"
  printf '%s\n' "${title}" >"${title_file}"

  # 终端主进程退出后窗口会关闭；不要再 exec 一个空 shell，否则 0 无法关窗。
  if ! gnome-terminal \
    --window \
    --title="${title}" \
    --working-directory="${project_dir}" \
    -- /bin/bash -c "printf '%s\\n' \$\$ > '${pid_file}'; exec /bin/bash '${script_path}' --run-component '${component}'"; then
    echo "[错误] 无法创建终端：${title}" >&2
    echo "提示：请在系统桌面终端中运行本菜单，不要只在无显示环境的远程 shell 中运行。" >&2
    rm -f "${title_file}"
    return 1
  fi

  wait_for_component_pid "${component}" "${title}"
}

# 将多个组件放进同一 gnome-terminal 窗口（多 tab）。
# 参数：组件名数组名、标题数组名、窗口用途说明、是否优先并入已有 VR 窗口。
start_components_window() {
  local -n _comp_names="$1"
  local -n _comp_titles="$2"
  local label="${3:-组件}"
  local prefer_existing_vr_window="${4:-0}"
  local -a terminal_args=()
  local first_tab=1
  local idx=0
  local component=""
  local title=""
  local pid_file=""
  local title_file=""
  local started=0
  local open_new_window=1

  if [[ "${prefer_existing_vr_window}" == "1" ]] && vr_teleop_window_active; then
    open_new_window=0
    focus_vr_teleop_window || true
  fi

  for idx in "${!_comp_names[@]}"; do
    component="${_comp_names[$idx]}"
    title="${_comp_titles[$idx]}"
    pid_file="${state_dir}/${component}.pid"
    title_file="${state_dir}/${component}.title"

    component_is_running "${component}" >/dev/null || true
    if component_is_running "${component}"; then
      echo "[跳过] ${title} 已在运行。"
      continue
    fi

    rm -f "${pid_file}"
    printf '%s\n' "${title}" >"${title_file}"

    if (( first_tab )); then
      if (( open_new_window )); then
        terminal_args+=(--window)
      else
        # 无 --window：新 tab 并入最近的 gnome-terminal（已尝试聚焦 VR 窗口）
        terminal_args+=(--tab)
      fi
      first_tab=0
    else
      terminal_args+=(--tab)
    fi

    # 多 tab 必须用 --command=；若用 -- 会吞掉后续 --tab 参数。
    terminal_args+=(
      --title="${title}"
      --working-directory="${project_dir}"
      --command="/bin/bash -c \"printf '%s\\n' \\\$\\\$ > '${pid_file}'; exec /bin/bash '${script_path}' --run-component '${component}'\""
    )
    started=1
  done

  if (( ! started )); then
    echo "[跳过] ${label}相关组件均已在运行。"
    return 0
  fi

  if ! gnome-terminal "${terminal_args[@]}"; then
    echo "[错误] 无法创建${label}终端窗口。" >&2
    return 1
  fi

  # 标记 VR 窗口已创建，便于 5/8 后续并入同一窗口
  if [[ "${prefer_existing_vr_window}" == "1" ]] || [[ "${label}" == *VR* ]]; then
    printf '%s\n' "1" >"${state_dir}/vr_teleop_window.active"
  fi

  for idx in "${!_comp_names[@]}"; do
    component="${_comp_names[$idx]}"
    title="${_comp_titles[$idx]}"
    if [[ -f "${state_dir}/${component}.title" ]]; then
      wait_for_component_pid "${component}" "${title}" || true
    fi
  done
}

vr_teleop_window_active() {
  [[ -f "${state_dir}/vr_teleop_window.active" ]] || return 1
  component_is_running pc_service \
    || component_is_running xr_bridge \
    || component_is_running arm_vr \
    || component_is_running chassis_vr \
    || component_is_running chassis_driver
}

focus_vr_teleop_window() {
  local candidate=""
  for candidate in \
    "XRoboToolkit PC Service" \
    "XR Bridge" \
    "OpenArm VR Teleoperation" \
    "Ranger Mini Driver" \
    "Ranger VR Control"; do
    if command -v wmctrl >/dev/null 2>&1; then
      if wmctrl -a "${candidate}" 2>/dev/null; then
        return 0
      fi
    fi
  done
  return 1
}

start_arm_vr_window() {
  local -a components=(pc_service xr_bridge arm_vr)
  local -a titles=(
    "XRoboToolkit PC Service"
    "XR Bridge"
    "OpenArm VR Teleoperation"
  )
  start_components_window components titles "OpenArm VR" 1
}

start_chassis_vr_window() {
  local -a components=(pc_service chassis_driver xr_bridge chassis_vr)
  local -a titles=(
    "XRoboToolkit PC Service"
    "Ranger Mini Driver"
    "XR Bridge"
    "Ranger VR Control"
  )
  start_components_window components titles "Chassis VR" 1
}

# 三相机 + RViz：同一窗口，多个 tab
start_cameras_window() {
  local -a components=(camera_l camera_h camera_r camera_rviz)
  local -a titles=(
    "camera_l - D405"
    "camera_h - D435I"
    "camera_r - D405"
    "Cameras + Robot RViz"
  )
  start_components_window components titles "相机" 0
}

prepare_for_vr_stack() {
  if component_is_running arm_moveit; then
    echo "[错误] MoveIt 2（菜单 4）已运行，与 VR 冲突；请先输入 0 关闭。" >&2
    return 1
  fi
  if component_is_running data_replay || component_is_running arm_bringup_fp; then
    echo "[错误] 数据回放/forward_position 驱动已运行，与 VR 冲突；请先输入 0 关闭。" >&2
    return 1
  fi
  if component_is_running arm_bringup; then
    echo "[INFO] 检测到菜单 3 驱动；VR 将使用自带驱动，先关闭菜单 3..."
    stop_component "${state_dir}/arm_bringup.pid"
    sleep 1
  fi
  return 0
}

prepare_for_replay_stack() {
  if component_is_running arm_vr; then
    echo "[错误] 机械臂 VR 已运行，与回放冲突；请先输入 0 关闭。" >&2
    return 1
  fi
  if component_is_running arm_moveit; then
    echo "[错误] MoveIt（菜单 4）已运行，与回放冲突；请先输入 0 关闭。" >&2
    return 1
  fi
  if component_is_running arm_bringup; then
    echo "[INFO] 检测到菜单 3 驱动；回放改用 forward_position，先关闭菜单 3..."
    stop_component "${state_dir}/arm_bringup.pid"
    sleep 1
  fi
  return 0
}

# 11：首次启动 1→2（同一终端多 tab）
start_first_boot_window() {
  local -a components=(can_install can_bringup)
  local -a titles=(
    "Install CAN Bindings"
    "Bringup All CAN"
  )
  echo "[INFO] 首次启动：安装 CAN 绑定 → 启动全部 CAN（同一终端多 tab）。"
  ensure_can_passwordless_sudo || return 1
  start_components_window components titles "首次启动" 0
}

# 12：全身 VR 2→5→7（同一终端多 tab）
start_full_body_vr_window() {
  local -a components=(
    can_bringup
    pc_service
    chassis_driver
    xr_bridge
    arm_vr
    chassis_vr
  )
  local -a titles=(
    "Bringup All CAN"
    "XRoboToolkit PC Service"
    "Ranger Mini Driver"
    "XR Bridge"
    "OpenArm VR Teleoperation"
    "Ranger VR Control"
  )
  prepare_for_vr_stack || return 1
  ensure_can_passwordless_sudo || return 1
  echo "[INFO] 全身 VR：CAN → 臂 VR(5) → 底盘 VR(8)（同一终端多 tab）。"
  echo "[提示] 请先确认 CAN tab 接口已就绪，再使用 VR。"
  start_components_window components titles "全身 VR" 1
}

# 13：数据采集（同一终端多 tab：CAN + VR + 相机 + 采集）
start_data_collection_stack_window() {
  local -a components=(
    can_bringup
    pc_service
    chassis_driver
    xr_bridge
    arm_vr
    chassis_vr
    camera_l
    camera_h
    camera_r
    camera_rviz
    data_collection
  )
  local -a titles=(
    "Bringup All CAN"
    "XRoboToolkit PC Service"
    "Ranger Mini Driver"
    "XR Bridge"
    "OpenArm VR Teleoperation"
    "Ranger VR Control"
    "camera_l - D405"
    "camera_h - D435I"
    "camera_r - D405"
    "Cameras + Robot RViz"
    "Data Collection"
  )
  prepare_for_vr_stack || return 1
  ensure_can_passwordless_sudo || return 1
  echo "[INFO] 数据采集：CAN → VR → 相机 → 采集（同一终端多 tab）。"
  echo "[提示] 先完成 CAN tab；话题就绪后在 Data Collection 按 SPACE 录制。"
  start_components_window components titles "数据采集" 1
}

# 14：数据回放（同一终端多 tab）
start_data_replay_stack_window() {
  local -a components=(
    can_bringup
    chassis_driver
    arm_bringup_fp
    camera_l
    camera_h
    camera_r
    camera_rviz
    data_replay
  )
  local -a titles=(
    "Bringup All CAN"
    "Ranger Mini Driver"
    "OpenArm FP Driver (Replay)"
    "camera_l - D405"
    "camera_h - D435I"
    "camera_r - D405"
    "Cameras + Robot RViz"
    "Data Replay"
  )
  prepare_for_replay_stack || return 1
  ensure_can_passwordless_sudo || return 1
  echo "[INFO] 数据回放：CAN → 底盘 → FP 驱动 → 三相机 → RViz → replay（同一终端多 tab）。"
  echo "[提示] 先完成 CAN tab；驱动与相机起来后在 Data Replay 按 SPACE 播放。"
  start_components_window components titles "数据回放" 0
}

collect_process_tree() {
  local parent="$1"
  local child=""

  while read -r child; do
    [[ -n "${child}" ]] || continue
    collect_process_tree "${child}"
  done < <(pgrep -P "${parent}" 2>/dev/null || true)

  printf '%s\n' "${parent}"
}

close_terminal_by_title() {
  local title="$1"

  [[ -n "${title}" ]] || return 0

  if command -v wmctrl >/dev/null 2>&1; then
    wmctrl -c "${title}" 2>/dev/null || true
  fi

  if command -v xdotool >/dev/null 2>&1; then
    local wid=""
    while read -r wid; do
      [[ -n "${wid}" ]] || continue
      xdotool windowclose "${wid}" 2>/dev/null || true
    done < <(xdotool search --name "${title}" 2>/dev/null || true)
  fi
}

stop_arm_command_publishers() {
  # 停掉仍在写 forward_position /commands 的节点，避免与 go_home 对打。
  local pid=""
  local -a pids=()
  local alive=0

  while read -r pid; do
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    pids+=("${pid}")
  done < <(pgrep -f 'openarm_vr_ik_teleop[.]ik_teleop_node|data_collection/replay[.]py' 2>/dev/null || true)

  ((${#pids[@]})) || return 0

  echo "[INFO] 先停止遥操作/回放发布（PID: ${pids[*]}），再回 home..."
  kill -TERM "${pids[@]}" 2>/dev/null || true
  for _ in $(seq 1 30); do
    alive=0
    for pid in "${pids[@]}"; do
      pid_is_running "${pid}" && alive=1
    done
    ((alive == 0)) && break
    sleep 0.1
  done
  for pid in "${pids[@]}"; do
    pid_is_running "${pid}" && kill -KILL "${pid}" 2>/dev/null || true
  done
  sleep 0.2
}

return_arms_home_if_needed() {
  # 必须在杀掉 bringup 之前调用，否则 forward_position 已死，无法回 home。
  if ! component_is_running arm_vr \
    && ! component_is_running arm_bringup \
    && ! component_is_running arm_bringup_fp; then
    return 0
  fi
  stop_arm_command_publishers
  echo "[INFO] 关闭前先将机械臂回到 home（驱动保持运行）..."
  if ! bash "${script_dir}/teleop/go_home_arms.sh"; then
    echo "[警告] 回 home 失败或超时，将继续关闭终端。" >&2
    return 1
  fi
  mkdir -p "${state_dir}"
  printf '%s\n' "1" >"${state_dir}/home_done"
  return 0
}

stop_component() {
  local pid_file="$1"
  local component
  local title_file=""
  local title=""
  local pid=""
  local pgid=""
  local target=""
  local -a targets=()
  local wait_loops=40

  component="$(basename "${pid_file}" .pid)"
  title_file="${state_dir}/${component}.title"
  [[ -f "${title_file}" ]] && read -r title <"${title_file}" || true

  read -r pid <"${pid_file}" 2>/dev/null || true
  if pid_is_running "${pid}"; then
    mapfile -t targets < <(collect_process_tree "${pid}")
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ' || true)"

    if [[ "${component}" == "arm_vr" ]]; then
      # 不要先杀进程组：否则 bringup 与 ik 同时死，回 home 无效。
      # 只对入口进程发 INT，让 launch_vr_teleop 的 trap 再停子进程。
      kill -INT "${pid}" 2>/dev/null || true
      wait_loops=100
    else
      if [[ "${pgid}" =~ ^[0-9]+$ ]]; then
        kill -TERM -- "-${pgid}" 2>/dev/null || true
      fi
      if ((${#targets[@]})); then
        kill -TERM "${targets[@]}" 2>/dev/null || true
      fi
    fi

    for _ in $(seq 1 "${wait_loops}"); do
      pid_is_running "${pid}" || break
      sleep 0.1
    done

    if [[ "${pgid}" =~ ^[0-9]+$ ]]; then
      kill -KILL -- "-${pgid}" 2>/dev/null || true
    fi

    for target in "${targets[@]}"; do
      if pid_is_running "${target}"; then
        kill -KILL "${target}" 2>/dev/null || true
      fi
    done
  fi

  close_terminal_by_title "${title}"
  rm -f "${pid_file}" "${title_file}"
}

stop_all() {
  local pid_file=""
  local title_file=""
  local title=""
  local -a ordered_pid_files=()
  local f=""

  echo
  echo "[INFO] 正在关闭本菜单启动的全部终端..."

  # 关键：在驱动还活着时回 home，再关终端。
  return_arms_home_if_needed || true

  shopt -s nullglob
  # 先关回放/采集与 arm_vr，再关 bringup，避免驱动先于控制端退出
  for f in \
    "${state_dir}/data_replay.pid" \
    "${state_dir}/data_collection.pid" \
    "${state_dir}/arm_vr.pid" \
    "${state_dir}"/*.pid; do
    [[ -f "${f}" ]] || continue
    [[ "${f}" == "${state_dir}/menu.pid" ]] && continue
    ordered_pid_files+=("${f}")
  done
  # 去重（arm_vr 可能被扫两次）
  local -A seen=()
  for pid_file in "${ordered_pid_files[@]}"; do
    [[ -n "${seen[${pid_file}]:-}" ]] && continue
    seen["${pid_file}"]=1
    stop_component "${pid_file}"
  done

  # 清理可能残留的标题记录
  for title_file in "${state_dir}"/*.title; do
    title=""
    read -r title <"${title_file}" || true
    close_terminal_by_title "${title}"
    rm -f "${title_file}"
  done
  rm -f "${state_dir}/vr_teleop_window.active" "${state_dir}/home_done"
  shopt -u nullglob
  echo "[INFO] 已全部关闭。"
}

check_menu_requirements() {
  if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "[错误] 未安装 gnome-terminal。" >&2
    exit 1
  fi

  if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    echo "[错误] 当前没有图形显示环境（DISPLAY/WAYLAND_DISPLAY 为空）。" >&2
    echo "请在本机桌面终端中运行本菜单。" >&2
    exit 1
  fi

  if [[ ! -f "${workspace_dir}/install/setup.bash" ]]; then
    echo "[错误] 工作空间尚未构建，请先运行：" >&2
    echo "  ${workspace_dir}/scripts/build.sh" >&2
    exit 1
  fi
}

show_menu() {
  cat <<'EOF'

==========================================
          RBM OpenArm 控制菜单
==========================================
【快速启动】
 11. 首次启动
 12. 全身 VR 遥操作
 13. 数据采集
 14. 数据回放

【分布步骤】
  1. 安装 CAN 绑定（仅首次）
  2. 启动全部 CAN（can0/can1/can2）
  3. 启动 OpenArm 驱动
  4. 启动 OpenArm MoveIt 控制（需先启动 3）
  5. 启动 OpenArm VR 遥操作（与 4 冲突）
  6. 启动底盘控制
  7. 启动底盘键盘控制（需先启动 6）
  8. 启动底盘 VR 遥操作
  9. 启动 camera+rviz 可视化
  0. 关闭本菜单启动的全部终端并退出
------------------------------------------
推荐：日常用 11（仅首次）/12/13/14；需拆分时再用 1–9。
提示：首次 CAN 会要一次 sudo 密码以安装免密，之后启动不再询问。
提示：VR 臂启动到 hands_up，握把跟随；X/A=复位；结束或按 0 时回 home。
提示：回放(14)勿与 VR/MoveIt 同时开；采集数据在 datasets/。
提示：启动真机前请确认急停、CAN 接口及工作空间安全。
EOF
}

if [[ "${1:-}" == "--run-component" ]]; then
  run_component "${2:-}"
  exit $?
fi

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

check_menu_requirements
mkdir -p "${state_dir}"

menu_pid_file="${state_dir}/menu.pid"
if [[ -f "${menu_pid_file}" ]]; then
  existing_menu_pid=""
  read -r existing_menu_pid <"${menu_pid_file}" || true
  if pid_is_running "${existing_menu_pid}"; then
    echo "[错误] 控制菜单已在运行，PID=${existing_menu_pid}" >&2
    echo "如需强制重启，可先关闭旧菜单，或删除 ${menu_pid_file}" >&2
    exit 1
  fi
fi
printf '%s\n' "$$" >"${menu_pid_file}"

cleanup_menu() {
  rm -f "${menu_pid_file}"
}
trap cleanup_menu EXIT
trap 'stop_all; exit 130' INT TERM

while true; do
  show_menu
  read -r -p "请输入序号 [0-9,11-14]：" choice

  case "${choice}" in
    11)
      start_first_boot_window
      ;;
    12)
      start_full_body_vr_window
      ;;
    13)
      start_data_collection_stack_window
      ;;
    14)
      start_data_replay_stack_window
      ;;
    1)
      ensure_can_passwordless_sudo || continue
      start_terminal can_install "Install CAN Bindings"
      ;;
    2)
      ensure_can_passwordless_sudo || continue
      start_terminal can_bringup "Bringup All CAN"
      ;;
    3)
      if component_is_running arm_vr || component_is_running arm_bringup_fp || component_is_running data_replay; then
        echo "[错误] VR/回放相关驱动已运行；请先输入 0 关闭，避免重复占用机械臂。"
      else
        start_terminal arm_bringup "OpenArm Driver"
      fi
      ;;
    4)
      if component_is_running arm_vr || component_is_running data_replay; then
        echo "[错误] 机械臂 VR 或数据回放已运行；请先输入 0 关闭。"
      elif ! component_is_running arm_bringup; then
        echo "[错误] 尚未启动机械臂驱动；请先输入 3。"
      else
        start_terminal arm_moveit "OpenArm MoveIt 2"
      fi
      ;;
    5)
      prepare_for_vr_stack && start_arm_vr_window
      ;;
    6)
      start_terminal chassis_driver "Ranger Mini Driver"
      ;;
    7)
      if ! component_is_running chassis_driver; then
        echo "[错误] 尚未启动底盘驱动；请先输入 6。"
      elif component_is_running chassis_vr; then
        echo "[错误] 底盘 VR（菜单 8）已在发布 /cmd_vel；请先输入 0 关闭，再使用键盘控制。"
      else
        start_terminal chassis_keyboard "Chassis Keyboard Teleop"
      fi
      ;;
    8)
      if component_is_running chassis_keyboard; then
        echo "[错误] 底盘键盘控制（菜单 7）已在发布 /cmd_vel；请先输入 0 关闭，再使用底盘 VR。"
      else
        # 与菜单 5 并入同一 VR 窗口；已运行的 PC Service / XR Bridge 会跳过。
        start_chassis_vr_window
      fi
      ;;
    9)
      start_cameras_window
      ;;
    0)
      stop_all
      exit 0
      ;;
    *)
      echo "[错误] 无效序号：${choice}"
      ;;
  esac
done
