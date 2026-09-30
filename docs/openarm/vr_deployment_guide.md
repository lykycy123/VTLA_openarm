# VR 遥操作 OpenArm 部署与复现指南

**文档用途**：说明如何将 VR 控制 OpenArm 机械臂的环境迁移到**新设备**，或为团队提供可复现的部署包。用于在新机器上复现完整 VR 遥操作环境，包括 conda-pack 打包、openarm_ws 迁移、deb/APK 安装等。

---

## 一、依赖概览

| 组件 | 说明 | 提供方式 |
|------|------|----------|
| 系统 | Ubuntu 22.04、ROS 2 Humble | 目标机自行安装 |
| XRoboToolkit PC Service | VR 数据服务，含 `libPXREARobotSDK.so` | deb 安装 |
| openarm 环境 | Python 3.10 + xrobotoolkit_sdk + placo | conda-pack 打包 |
| openarm_ws | OpenArm 源码及 VR 相关包 | 复制文件夹 |
| PICO APK | VR 头显端 | apk 安装 |

---

## 二、源设备：准备导出包

### 2.1 安装 conda-pack

```bash
conda install -n base -c conda-forge conda-pack
```

### 2.2 导出 openarm 环境

```bash
conda activate openarm
conda pack -n openarm -o openarm.tar.gz
```

生成 `openarm.tar.gz`，内含完整 Python 环境（含 xrobotoolkit_sdk、placo 等），无需目标机联网安装。

### 2.3 打包 openarm_ws

将以下内容打包（例如 `openarm_ws_vr.tar.gz`）：

- `src/`：源码
- `install/`：已编译产物
- `build/`：可选，可不复制以减小体积（目标机需重新 `colcon build`）
- `openarm_vr_topics.md`、`openarm_setup_guide.md`、`vr_openarm_implementation.md` 等文档

建议目录结构保持与源机一致，可放置于任意路径（如 `~/Medibot_openarm/openarm_ws`），脚本会按相对路径解析。

### 2.4 需一并提供的文件

| 文件 | 用途 |
|------|------|
| `XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb` | PC Service 安装包 |
| `XRoboToolkit-PICO-1.1.1.apk` | PICO 头显应用 |
| `openarm.tar.gz` | conda 环境包 |
| `openarm_ws` 目录或压缩包 | 工作空间 |

---

## 三、目标设备：环境搭建

### 3.1 系统要求

- Ubuntu 22.04
- ROS 2 Humble（`/opt/ros/humble`）
- Conda 或 Miniconda

### 3.2 安装 XRoboToolkit PC Service

```bash
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

安装后 PC Service 位于 `/opt/apps/roboticsservice/`。启动：

```bash
bash /opt/apps/roboticsservice/runService.sh
```

### 3.3 解压 openarm_ws

```bash
# 解压到目标路径（示例为 ~/Medibot_openarm/openarm_ws）
tar -xzf openarm_ws_vr.tar.gz -C ~/projects/
# 或直接复制目录
```

若未包含 `build/`，需重新编译：

```bash
cd ~/Medibot_openarm/openarm_ws  # 或你的实际路径
source /opt/ros/humble/setup.bash
colcon build
```

### 3.4 恢复 openarm 环境

```bash
mkdir -p ~/openarm_env
tar -xzf openarm.tar.gz -C ~/openarm_env
```

每次使用前激活：

```bash
source ~/openarm_env/bin/activate
```

或创建 conda 软链接（可选）：

```bash
conda create -n openarm --clone ~/openarm_env  # 若 conda 支持
# 更简单：直接使用 source ~/openarm_env/bin/activate
```

### 3.5 安装 PICO APK

将 `XRoboToolkit-PICO-1.1.1.apk` 安装到 PICO 头显，确保头显与 PC 处于同一网络。

---

## 四、运行与验证

### 4.1 一键启动（推荐）

`openarm_ws/scripts/teleop/launch_vr_teleop.sh` 可一次性启动 bringup + xr_bridge + ik_teleop。**脚本不包含编译**，修改 ik_teleop、xr_bridge、openarm_description 等源码后需先编译：

```bash
cd ~/Medibot_openarm/openarm_ws
# 修改源码后先编译（在 source 之前）
source /opt/ros/humble/setup.bash
colcon build --packages-select openarm_xr_bridge openarm_vr_ik_teleop openarm_description openarm_bringup

# 再激活环境并启动
conda activate openarm
./scripts/teleop/launch_vr_teleop.sh
```

编译需在 **source install/setup.bash 之前** 完成，以便启动时加载最新产物。

**参数**：`--fake`（默认）/ `--real`；`--bimanual`（默认）/ `--unimanual`；`--skip-check` 跳过启动前确认。示例：`./scripts/teleop/launch_vr_teleop.sh --real`（在 `openarm_ws` 下）。启动前会提示 PC Service、头显、环境等准备工作，确认后继续。按 `Ctrl+C` 结束全部。

### 4.2 分终端启动（可选）

设工作空间路径为 `$WS`。

**终端 1 — bringup**

```bash
source $WS/install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=true robot_controller:=forward_position_controller
```

**终端 2 — xr_bridge**

```bash
source ~/openarm_env/bin/activate   # 或 conda activate openarm
source $WS/install/setup.bash
$WS/install/openarm_xr_bridge/share/openarm_xr_bridge/scripts/run_xr_bridge.sh
```

脚本会根据自身位置自动解析 install 路径，无需写死 `$WS`。

**终端 3 — ik_teleop**

```bash
source ~/openarm_env/bin/activate
source $WS/install/setup.bash
ros2 launch openarm_vr_ik_teleop ik_teleop.launch.py
```

### 4.3 验证项

| # | 检查项 | 预期 |
|---|--------|------|
| 1 | PC Service 已启动 | `runService.sh` 运行无报错 |
| 2 | xr_bridge 启动 | 输出 `XRoboToolkit SDK initialized.` |
| 3 | IK 节点启动 | 输出 `IK teleop node started` |
| 4 | 握把激活 | 握把 > 0.9 时手臂随手柄移动 |
| 5 | 扳机夹爪 | 扳机：0=夹爪开，1=夹爪闭 |

---

## 五、故障排查

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `libPXREARobotSDK.so` 找不到 | deb 未安装或路径错误 | 确认 deb 已安装，`run_xr_bridge.sh` 已设置 `LD_LIBRARY_PATH` |
| `rclpy._rclpy_pybind11` 加载失败 | Python 版本非 3.10 | 确认 `openarm` 为 Python 3.10 |
| `import xrobotoolkit_sdk` 失败 | 环境未激活 | 使用 `source ~/openarm_env/bin/activate` |
| source setup.bash 报错 | 路径错误或未编译 | 确认 `$WS/install/setup.bash` 存在 |

---

## 六、相关文档

- `openarm_vr_topics.md` — 命令话题与消息格式
- `vr_openarm_implementation.md` — 实现方案与分阶段验证
- `openarm_setup_guide.md` — OpenArm 系统配置与启动说明
