#!/usr/bin/env bash
# 为当前用户配置 CAN 相关脚本的 sudo 免密（写入 /etc/sudoers.d/）。
# 只需执行一次；不会把登录密码写入仓库。

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_scripts="$(cd "${script_dir}/.." && pwd)"
repo_root="$(cd "${script_dir}/../../.." && pwd)"

target_user="${SUDO_USER:-${USER}}"
if [[ "${target_user}" == "root" ]]; then
  echo "[错误] 请用普通用户执行：bash ${script_dir}/install_passwordless_sudo.sh" >&2
  exit 1
fi

install_can="${script_dir}/install_can_bindings.sh"
bringup_can="${script_dir}/bringup_can.sh"
configure_can="${repo_root}/openarm_ws/src/arm/openarm_can/setup/configure_socketcan.sh"
sudoers_dst="/etc/sudoers.d/rbm-openarm-can-${target_user}"

for f in "${install_can}" "${bringup_can}" "${configure_can}"; do
  if [[ ! -x "${f}" && ! -f "${f}" ]]; then
    echo "[错误] 未找到：${f}" >&2
    exit 1
  fi
  chmod +x "${f}" 2>/dev/null || true
done

tmp_file="$(mktemp)"
trap 'rm -f "${tmp_file}"' EXIT

cat >"${tmp_file}" <<EOF
# RBM OpenArm — passwordless sudo for CAN bringup (user: ${target_user})
# Managed by openarm_ws/scripts/can/install_passwordless_sudo.sh
Defaults:${target_user} !requiretty

${target_user} ALL=(root) NOPASSWD: ${install_can}
${target_user} ALL=(root) NOPASSWD: ${bringup_can}
${target_user} ALL=(root) NOPASSWD: ${bringup_can} *
${target_user} ALL=(root) NOPASSWD: ${configure_can}
${target_user} ALL=(root) NOPASSWD: ${configure_can} *
EOF

chmod 440 "${tmp_file}"

if ! visudo -cf "${tmp_file}"; then
  echo "[错误] sudoers 语法校验失败，未安装。" >&2
  exit 1
fi

if [[ "${EUID}" -eq 0 ]]; then
  install -m 440 "${tmp_file}" "${sudoers_dst}"
else
  echo "[INFO] 需要一次管理员密码以安装免密规则（之后 CAN 启动不再提示）..."
  sudo install -m 440 "${tmp_file}" "${sudoers_dst}"
fi

echo "[INFO] 已写入 ${sudoers_dst}"
echo "[INFO] 验证免密："
echo "  sudo -n ${bringup_can} --help"
if sudo -n "${bringup_can}" --help >/dev/null 2>&1; then
  echo "[INFO] 免密 sudo 已生效。"
else
  echo "[警告] 当前 shell 仍无法 sudo -n；请新开终端后再试，或检查 ${sudoers_dst}。" >&2
fi
