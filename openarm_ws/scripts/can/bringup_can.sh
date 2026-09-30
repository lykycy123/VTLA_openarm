#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/../../.." && pwd)"
openarm_cfg="${project_dir}/openarm_ws/src/arm/openarm_can/setup/configure_socketcan.sh"

ARM_BITRATE=1000000
ARM_DBITRATE=5000000
CHASSIS_BITRATE=500000

usage() {
  cat <<'EOF'
用法：bringup_can.sh [--arm] [--chassis] [--all]

  --arm       配置 OpenArm：can0/can1（CAN-FD 1M/5M）
  --chassis   配置底盘：can2（500 kbps）
  --all       同时配置机械臂和底盘（默认）
EOF
}

print_can_diagnostics() {
  echo
  echo "诊断："
  echo "  当前 CAN 接口："
  if ! ip -br link show type can 2>/dev/null | grep -q .; then
    echo "    （无）"
  else
    ip -br link show type can 2>/dev/null | sed 's/^/    /'
  fi
  echo "  USB PEAK（Vendor 0c72）："
  if lsusb 2>/dev/null | grep -qi '0c72:'; then
    lsusb 2>/dev/null | grep -i '0c72:' | sed 's/^/    /'
  else
    echo "    未检测到。请插入 PCAN-USB Pro FD / 双通道 USB-CANFD 卡。"
  fi
  echo "  USB 底盘适配器（gs_usb 常见 ID）："
  if lsusb 2>/dev/null | grep -qiE '1d50:606f|16d0:117e|0483:5740'; then
    lsusb 2>/dev/null | grep -iE '1d50:606f|16d0:117e|0483:5740' | sed 's/^/    /'
  else
    echo "    未检测到。请插入 Ranger USB-CAN（松灵/candleLight 一类）。"
  fi
}

chassis_usb_present() {
  lsusb 2>/dev/null | grep -qiE '1d50:606f|16d0:117e|0483:5740'
}

find_can_by_driver() {
  local want_driver="$1"
  local iface=""
  local driver=""

  shopt -s nullglob
  for iface in /sys/class/net/can*; do
    [[ -e "${iface}/device/driver" ]] || continue
    driver="$(basename "$(readlink -f "${iface}/device/driver")")"
    if [[ "${driver}" == "${want_driver}" ]]; then
      basename "${iface}"
      shopt -u nullglob
      return 0
    fi
  done
  shopt -u nullglob
  return 1
}

# 菜单 11 会并行跑「安装绑定」：那边 modprobe -r gs_usb 会把 can2 拆掉，
# USB 设备还在，SocketCAN 接口要等驱动重新绑定。机械臂 PEAK 往往先就绪。
wait_for_gs_usb_iface() {
  local i=0
  local found=""

  for i in {1..40}; do
    found="$(find_can_by_driver gs_usb || true)"
    if [[ -n "${found}" ]]; then
      printf '%s\n' "${found}"
      return 0
    fi
    if (( i == 1 || i == 10 || i == 20 )); then
      modprobe gs_usb 2>/dev/null || true
      udevadm trigger --subsystem-match=net --action=add 2>/dev/null || true
    fi
    sleep 0.25
  done
  return 1
}

# 把 current 重命名为 desired；可选 expect_driver 用于区分「同名但驱动不对」的情况。
# 背景：udev 规则按 peak_usb 的 dev_id 分通道，但当 USB 枚举顺序让 gs_usb(底盘) 先占住
# can1 时，一轮 udev 无法把 can1<->can2 对调（两个目标名互相占用 → 死锁），
# 结果可能是 can1=gs_usb、can2=peak 通道1。这里先把占着目标名但驱动不对的接口挪到
# 临时名（仍是 can*，随后 bringup_chassis 会按驱动认领），避免 set -e 直接中止导致
# can0/can1 连 up 的机会都没有。
ensure_name() {
  local current="$1"
  local desired="$2"
  local expect_driver="${3:-}"

  if [[ "${current}" == "${desired}" ]]; then
    return 0
  fi

  if ip link show "${desired}" &>/dev/null; then
    local occ_driver=""
    occ_driver="$(basename "$(readlink -f "/sys/class/net/${desired}/device/driver" 2>/dev/null)" 2>/dev/null || true)"
    if [[ -n "${expect_driver}" && "${occ_driver}" != "${expect_driver}" ]]; then
      local tmp="${desired}tmp"
      local n=0
      while ip link show "${tmp}" &>/dev/null; do
        n=$((n + 1))
        tmp="${desired}tmp${n}"
      done
      echo "[INFO] ${desired} 当前驱动为 ${occ_driver:-未知}（应为 ${expect_driver}），先让位到 ${tmp}"
      ip link set "${desired}" down 2>/dev/null || true
      ip link set "${desired}" name "${tmp}"
    else
      echo "[错误] 目标接口 ${desired} 已存在，无法将 ${current} 重命名为 ${desired}。" >&2
      return 1
    fi
  fi

  echo "[INFO] 将 ${current} 重命名为 ${desired}"
  ip link set "${current}" down 2>/dev/null || true
  ip link set "${current}" name "${desired}"
}

bringup_arm() {
  local right=""
  local left=""
  local -a peak_ifaces=()
  local iface=""
  local driver=""

  shopt -s nullglob
  for iface in /sys/class/net/can*; do
    [[ -e "${iface}/device/driver" ]] || continue
    driver="$(basename "$(readlink -f "${iface}/device/driver")")"
    if [[ "${driver}" == "peak_usb" ]]; then
      peak_ifaces+=("$(basename "${iface}")")
    fi
  done
  shopt -u nullglob

  if ((${#peak_ifaces[@]} < 2)); then
    echo "[错误] 未找到 OpenArm 的 PEAK USB（peak_usb）双通道接口（当前 ${#peak_ifaces[@]} 个）。" >&2
    if lsusb 2>/dev/null | grep -qi '0c72:'; then
      echo "USB 上已有 PEAK 设备，但 SocketCAN 通道不足。请检查 peak_usb 是否被占用，或重新插拔适配器。" >&2
    else
      echo "USB 总线上看不到 PEAK 设备（Vendor 0c72）。这不是 udev 问题：规则已安装。" >&2
      echo "请插入 PCAN-USB Pro FD（或 OpenArm 双通道 USB-CANFD 卡）后再运行菜单 2。" >&2
    fi
    print_can_diagnostics >&2
    return 1
  fi

  # 优先按约定名称；否则按枚举顺序映射。
  if ip link show can0 &>/dev/null && [[ "$(basename "$(readlink -f /sys/class/net/can0/device/driver)")" == "peak_usb" ]]; then
    right=can0
  else
    right="${peak_ifaces[0]}"
    ensure_name "${right}" can0 peak_usb
    right=can0
  fi

  if ip link show can1 &>/dev/null && [[ "$(basename "$(readlink -f /sys/class/net/can1/device/driver)")" == "peak_usb" ]]; then
    left=can1
  else
    # 重新收集，避免 rename 后索引过期
    peak_ifaces=()
    shopt -s nullglob
    for iface in /sys/class/net/can*; do
      [[ -e "${iface}/device/driver" ]] || continue
      driver="$(basename "$(readlink -f "${iface}/device/driver")")"
      if [[ "${driver}" == "peak_usb" && "$(basename "${iface}")" != "can0" ]]; then
        peak_ifaces+=("$(basename "${iface}")")
      fi
    done
    shopt -u nullglob
    left="${peak_ifaces[0]:-}"
    [[ -n "${left}" ]] || {
      echo "[错误] 未找到第二个 peak_usb 通道。" >&2
      return 1
    }
    ensure_name "${left}" can1 peak_usb
    left=can1
  fi

  if [[ -x "${openarm_cfg}" ]]; then
    "${openarm_cfg}" can0 -fd -b "${ARM_BITRATE}" -d "${ARM_DBITRATE}"
    "${openarm_cfg}" can1 -fd -b "${ARM_BITRATE}" -d "${ARM_DBITRATE}"
  else
    ip link set can0 down || true
    ip link set can0 type can bitrate "${ARM_BITRATE}" dbitrate "${ARM_DBITRATE}" fd on
    ip link set can0 up
    ip link set can1 down || true
    ip link set can1 type can bitrate "${ARM_BITRATE}" dbitrate "${ARM_DBITRATE}" fd on
    ip link set can1 up
    echo "✓ can0 / can1 已按 CAN-FD 启动"
  fi
}

bringup_chassis() {
  local iface=""
  local driver=""

  if ip link show can2 &>/dev/null; then
    driver="$(basename "$(readlink -f /sys/class/net/can2/device/driver)" 2>/dev/null || true)"
    if [[ "${driver}" == "gs_usb" || -z "${driver}" ]]; then
      iface=can2
    else
      echo "[错误] can2 当前驱动为 ${driver}，不是底盘 gs_usb。请检查 USB 插接和 udev 绑定。" >&2
      return 1
    fi
  else
    echo "[INFO] 等待底盘 gs_usb 接口（USB 已插入时可能仍在重新绑定）..."
    iface="$(wait_for_gs_usb_iface || true)"
    if [[ -z "${iface}" ]]; then
      if chassis_usb_present; then
        echo "[错误] USB 上已有底盘适配器（如 1d50:606f），但内核尚未生成 gs_usb 网卡。" >&2
        echo "请稍后重跑菜单 2，或重新插拔底盘 USB-CAN；不要把 peak_usb 改名为 can2。" >&2
      else
        echo "[错误] 未找到底盘 USB-CAN（gs_usb）。" >&2
        echo "请插入底盘适配器；不要再把 peak_usb 的 can0 改名为 can2。" >&2
      fi
      print_can_diagnostics >&2
      return 1
    fi
    ensure_name "${iface}" can2 gs_usb
    iface=can2
  fi

  ip link set can2 down 2>/dev/null || true
  ip link set can2 up type can bitrate "${CHASSIS_BITRATE}"
  echo "✓ can2 is up (bitrate ${CHASSIS_BITRATE}, gs_usb)"
}

do_arm=false
do_chassis=false

if (($# == 0)); then
  do_arm=true
  do_chassis=true
fi

while (($#)); do
  case "$1" in
    --arm) do_arm=true ;;
    --chassis) do_chassis=true ;;
    --all)
      do_arm=true
      do_chassis=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数：$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ "${EUID}" -ne 0 ]]; then
  echo "请使用 sudo 运行本脚本。" >&2
  exit 1
fi

modprobe gs_usb 2>/dev/null || true
modprobe peak_usb 2>/dev/null || true

if $do_arm; then
  bringup_arm
fi

if $do_chassis; then
  bringup_chassis
fi

echo
echo "当前 CAN 接口："
ip -br link show type can 2>/dev/null || ip -br link | grep -E '^can' || true
