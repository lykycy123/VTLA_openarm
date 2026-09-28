#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rules_src="${script_dir}/99-rbm-openarm-can.rules"
rules_dst="/etc/udev/rules.d/99-rbm-openarm-can.rules"

if [[ "${EUID}" -ne 0 ]]; then
  echo "请使用 sudo 运行：" >&2
  echo "  sudo ${script_dir}/install_can_bindings.sh" >&2
  exit 1
fi

if [[ ! -f "${rules_src}" ]]; then
  echo "未找到规则文件：${rules_src}" >&2
  exit 1
fi

install -m 644 "${rules_src}" "${rules_dst}"
udevadm control --reload-rules
udevadm trigger --subsystem-match=net --action=add

# 不要 modprobe -r：菜单 11 会与「Bringup All CAN」并行，卸载 gs_usb
# 会拆掉 can2，bringup 侧只能看到 USB 设备、看不到网卡。
modprobe peak_usb 2>/dev/null || true
modprobe gs_usb 2>/dev/null || true
udevadm trigger --subsystem-match=net --action=add

sleep 1

echo "已安装 CAN 绑定规则：${rules_dst}"
echo
echo "当前 CAN 接口："
ip -br link show type can 2>/dev/null || ip -br link | grep -E '^can' || true
echo
echo "约定："
echo "  peak_usb channel0 -> can0  (OpenArm 右臂)"
echo "  peak_usb channel1 -> can1  (OpenArm 左臂)"
echo "  gs_usb            -> can2  (Ranger 底盘)"
echo
echo "重新插拔 USB-CAN 后执行："
echo "  ${script_dir}/bringup_can.sh"
