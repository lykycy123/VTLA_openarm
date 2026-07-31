#!/bin/bash
# 全向移动底盘 ROS2 节点启动脚本
# 
# 使用方法:
#   1. 先启动向话题 /exo/gamepad_keys 发布摇杆数据的节点（与您的遥操作栈一致）
#
#   2. 然后运行此脚本启动底盘节点:
#      ./omni_mover_ros_start.sh
#
# 参数说明:
#   - serial_port: 串口设备 (默认: /dev/ttyACM0)
#   - speed_multiplier: 速度乘数 (默认: 6.0)
sudo chmod 777 /dev/ttyACM0

echo "========================================"
echo "  全向移动底盘 ROS2 节点"
echo "========================================"
echo ""
echo "订阅话题: /exo/gamepad_keys"
echo "控制方式: 外骨骼摇杆"
echo "  - 左摇杆: 平移控制"
echo "  - 右摇杆X: 旋转控制"
echo "  - buttons[1]=1: 启用控制"
echo ""
echo "启动节点..."
echo ""

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 运行节点
python3 "${SCRIPT_DIR}/omni_mover_ros_node.py"

