#!/bin/bash

shell_type=${SHELL##*/}
shell_exec="exec $shell_type"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
script_path="${script_dir}/$(basename "${BASH_SOURCE[0]}")"
workspace_dir="$(cd "${script_dir}/../.." && pwd)"

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

if [[ $# -eq 0 ]] && pgrep -f "^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch realsense2_camera rs_launch.py" >/dev/null; then
  echo "检测到 RealSense 相机节点已在运行，请先关闭现有相机终端后再启动。" >&2
  exit 1
fi

declare -A CAMS=(
  [camera_l]="260322278361"
  [camera_h]="134222076235"
  [camera_r]="260322276489"
)
CAM_ORDER=(camera_l camera_h camera_r)

declare -A COLOR_PROFILES=(
  [camera_l]="640x480x30"
  [camera_h]="1280x720x30"
  [camera_r]="640x480x30"
)
declare -A COLOR_PROFILE_ARGS=(
  [camera_l]="depth_module.color_profile"
  [camera_h]="rgb_camera.color_profile"
  [camera_r]="depth_module.color_profile"
)

normalize_serial() 
{
  local s="${1//[[:space:]]/}"

  while [[ "$s" == _* ]]; do s="${s#_}"; done
  printf '%s' "$s"
}

serial_is_set() 
{
  local s="$1"
  [[ -n "${s}" && "${s}" != "_" ]]
}

serial_is_online() 
{
  local s="$1"
  [[ ${#ONLINE[@]} -eq 0 ]] && return 0
  [[ -n "${ONLINE[$s]:-}" ]]
}

launch_cam() 
{
  local cam_name="$1" raw_serial="$2"
  local delay="${3:-0}"
  local color_profile="${COLOR_PROFILES[$cam_name]}"
  local color_profile_arg="${COLOR_PROFILE_ARGS[$cam_name]}"
  local extra_args=()

  if [[ "$cam_name" == "camera_h" ]]; then
    extra_args+=("rgb_camera.brightness:=10")
  fi

  if ! serial_is_set "$raw_serial"; then
    return 0
  fi

  if [[ "$delay" =~ ^[0-9]+$ ]] && (( delay > 0 )); then
    sleep "$delay"
  fi

  digits="$(normalize_serial "$raw_serial")"
  local sn="_${digits}"

  echo "$cam_name -> serial=$sn"
  ros2 launch realsense2_camera rs_launch.py \
  camera_name:=${cam_name} \
  "${color_profile_arg}:=${color_profile}" \
  enable_depth:=false \
  initial_reset:=true \
  "${extra_args[@]}" \
  serial_no:=${sn}
  $shell_exec
}

launch_rviz()
{
  local delay="${1:-0}"

  if [[ "$delay" =~ ^[0-9]+$ ]] && (( delay > 0 )); then
    sleep "$delay"
  fi

  rviz2 -d "${script_dir}/rviz/camera.rviz"
  $shell_exec
}

if [[ "${1:-}" == "--launch-camera" ]]; then
  launch_cam "${2:-}" "${3:-}" "${4:-0}"
  exit $?
fi

if [[ "${1:-}" == "--launch-rviz" ]]; then
  launch_rviz "${2:-0}"
  exit $?
fi

terminal_args=()
first_tab=1
delay=0

terminal_cmd() {
  local title="$1"
  local raw_serial="$2"
  local start_delay="$3"
  local tab_option="--tab"

  if (( first_tab )); then
    tab_option="--window"
    first_tab=0
  fi

  terminal_args+=(
    "$tab_option"
    --title="$title"
    --command="$shell_type -ic '/bin/bash $script_path --launch-camera $title $raw_serial $start_delay'"
  )
}

for cam in "${CAM_ORDER[@]}"; do
  if serial_is_set "${CAMS[$cam]}"; then
    echo "$cam -> serial=_$(normalize_serial "${CAMS[$cam]}")"
    terminal_cmd "$cam" "${CAMS[$cam]}" "$delay"
    delay=$((delay + 1))
  fi
done

terminal_args+=(
  --tab
  --title="rviz"
  --command="$shell_type -ic '/bin/bash $script_path --launch-rviz $delay'"
)

if [[ ${#terminal_args[@]} -gt 0 ]]; then
  gnome-terminal "${terminal_args[@]}"
fi
