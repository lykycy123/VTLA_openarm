#!/bin/bash
# 脚本路径：/home/wbk/omni_mover/omni_mover_start.sh

# 1. 给串口设备赋运行权限
sudo chmod 777 /dev/ttyACM0

# 2. 等待串口设备稳定
sleep 2

# 3. 手动加载conda环境配置（关键：替换为你的conda安装路径）
# 先确认你的conda安装路径（通常是miniconda3或anaconda3）
# 例如：如果conda安装在 /home/wbk/miniconda3，就用下面这行
source /home/wbk/miniconda3/etc/profile.d/conda.sh
# 如果是anaconda3，替换为：source /home/wbk/anaconda3/etc/profile.d/conda.sh

# 4. 激活openarm环境
conda activate openarm

# 5. 切换到程序目录并运行
cd /home/wbk/omni_mover/
python xbox.py

