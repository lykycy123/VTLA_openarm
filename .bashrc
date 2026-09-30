# 按本文件自身位置定位工作空间，clone 到任何路径都能用
_medibot_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${_medibot_root}/openarm_ws/install/setup.bash" ]; then
  source "${_medibot_root}/openarm_ws/install/setup.bash"
fi
unset _medibot_root
