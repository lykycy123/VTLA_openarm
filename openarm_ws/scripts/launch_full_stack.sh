#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/.." && pwd)"
project_dir="$(cd "${workspace_dir}/.." && pwd)"

launch_arm=false
launch_chassis=false
launch_cameras=false
launch_vr=false
launch_record=false
real_arm=false
episode_name=""

usage() {
  cat <<'EOF'
Usage: launch_full_stack.sh [components] [options]

Components:
  --arm       Launch OpenArm bringup
  --chassis   Launch Ranger Mini V3 bringup
  --cameras   Launch three RealSense cameras and RViz
  --vr        Launch VR arm teleoperation (includes OpenArm bringup)
  --record    Run data_collection.py (SPACE to record; needs openarm env)
  --all       Launch chassis, cameras, VR and recording

Options:
  --real-arm          Use real OpenArm hardware (default: fake hardware)
  --episode NAME      Unused for data_collection (kept for CLI compatibility)
  -h, --help          Show this message

Tip: daily workflows prefer ./openarm_ws/scripts/control_menu.sh (11–14).
EOF
}

while (($#)); do
  case "$1" in
    --arm) launch_arm=true ;;
    --chassis) launch_chassis=true ;;
    --cameras) launch_cameras=true ;;
    --vr) launch_vr=true ;;
    --record) launch_record=true ;;
    --all)
      launch_chassis=true
      launch_cameras=true
      launch_vr=true
      launch_record=true
      ;;
    --real-arm) real_arm=true ;;
    --episode)
      shift
      episode_name="${1:-}"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if ! $launch_arm && ! $launch_chassis && ! $launch_cameras && ! $launch_vr && ! $launch_record; then
  usage
  exit 2
fi

if $launch_arm && $launch_vr; then
  echo "--vr already launches OpenArm; do not combine it with --arm." >&2
  exit 2
fi

if [[ ! -f "${workspace_dir}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ${script_dir}/build.sh first." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"

pids=()
cleanup() {
  if ((${#pids[@]})); then
    kill "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if $launch_chassis; then
  ros2 launch ranger_bringup ranger_mini_v3.launch.py &
  pids+=("$!")
  sleep 2
fi

if $launch_cameras; then
  "${script_dir}/sensors/realsense.sh"
  sleep 2
fi

if $launch_arm; then
  fake_hardware=true
  $real_arm && fake_hardware=false
  ros2 launch openarm_bringup openarm.bimanual.launch.py \
    arm_type:=v10 \
    use_fake_hardware:="${fake_hardware}" \
    robot_controller:=forward_position_controller &
  pids+=("$!")
  sleep 3
fi

if $launch_vr; then
  vr_mode=--fake
  $real_arm && vr_mode=--real
  "${script_dir}/teleop/launch_vr_teleop.sh" "${vr_mode}" --bimanual --skip-check &
  pids+=("$!")
  sleep 3
fi

if $launch_record; then
  if [[ -n "${episode_name}" ]]; then
    echo "[WARN] --episode is ignored; data_collection.py writes datasets/episode_N.hdf5" >&2
  fi
  # Prefer conda openarm for h5py/opencv; fall back to current python3.
  record_cmd=(python3 "${project_dir}/data_collection/data_collection.py"
    --config "${project_dir}/data_collection/config.yaml"
    --datasets-dir "${project_dir}/datasets")
  if [[ -f "${project_dir}/openarm_env/bin/activate" ]]; then
    # shellcheck disable=SC1091
    source "${project_dir}/openarm_env/bin/activate"
  elif command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook)"
    conda activate openarm 2>/dev/null || true
  fi
  echo "[INFO] data_collection: SPACE=record/stop, ESC=quit"
  "${record_cmd[@]}" &
  pids+=("$!")
fi

wait
