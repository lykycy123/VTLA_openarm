# VR → OpenArm 实现方案（分阶段可验证，是通过agent实现本项目的过程）

**文档用途**：VR 遥操作 OpenArm 的**分阶段实现与验证方案**。将任务拆为 5 个阶段，每阶段可独立验证。用于开发时按阶段推进、排查问题，以及理解整体架构与依赖关系。

---

## Python 3.10 要求（必读）

**ROS 2 Humble 的 rclpy 仅支持 Python 3.10**。若使用 Python 3.13（如未改造前的 pico_vr），会报 `rclpy._rclpy_pybind11` 加载失败。

**解决**：将 pico_vr 改为 Python 3.10：
```bash
conda install -n pico_vr python=3.10
# 然后重跑 XRoboToolkit 的 setup_conda.sh --install
```

xrobotoolkit_teleop 的 `requires-python` 为 `>=3.10`，无组件要求 3.13。

---

## 方案可行性复核

| 依赖项 | 状态 |
|--------|------|
| OpenArm bringup 假硬件 | ✓ |
| forward_position_controller 接口 | ✓ 见 `openarm_vr_topics.md` |
| XRoboToolkit VR 数据源 | ✓ |
| Placo IK | ✓ 有参考实现 |
| OpenArm URDF | ✓ |
| xrobotoolkit_sdk | ⚠ 需 Python 3.10 环境 |

---

## 阶段 0：环境预检

### 0.1 bringup 假硬件

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=true robot_controller:=forward_position_controller
```

预期：RViz 打开，双臂模型显示；无报错退出。

### 0.2 控制器话题存在

新终端执行（bringup 保持运行）：

```bash
ros2 topic list | grep forward
```

预期输出包含：`/left_forward_position_controller/commands`、`/right_forward_position_controller/commands`。

### 0.3 RViz 双臂模型

目视确认 RViz 中双臂模型正常显示。

### 0.4 XRobo MuJoCo 可用

先启动 PC Service，再执行：

```bash
conda activate pico_vr
cd ~/ros2_ws/XRoboToolkit-Teleop-Sample-Python
python scripts/simulation/teleop_dual_ur5e_mujoco.py
```

预期：VR 能控制 MuJoCo 中 UR5e。按需 Ctrl+C 退出。

### 0.5 Python 3.10 与 SDK

```bash
conda activate pico_vr
python --version
python -c "import xrobotoolkit_sdk; print('OK')"
```

预期：`python --version` 为 `Python 3.10.x`；第二行打印 `OK` 无报错。

---

## 修改点汇总（供您核对）

| 文件 | 修改内容 |
|------|----------|
| `openarm_vr_ik_teleop/ik_teleop_node.py` | VR 专用关节限位：左右 joint4 lower 从 0.0 收紧至 0.15 rad（仅 VR 遥操作生效，防 IK 跳变） |
| `openarm_vr_ik_teleop/geometry_utils.py` | R_HEADSET_TO_WORLD 改为 Unity→ROS 映射（VR +Z 前→world +X 前），修正向前/向右偏转 |
| `openarm_xr_bridge/xr_bridge_node.py` | 新增 `/vr/left_gripper`、`/vr/right_gripper`（扳机→夹爪）；新增 `/vr/controller_input`（sensor_msgs/Joy，按钮 ABXY + 摇杆）；移除原 `left_trigger`/`right_trigger`，改用 gripper 命名 |
| `openarm_xr_bridge/README.md` | 更新话题表；精简 Python 3.10 说明 |
| `openarm_vr_topics.md` | 新增「openarm_xr_bridge 发布的话题」小节 |
| `vr_openarm_implementation.md` | 合并 Python 3.10 说明；新增阶段 1 详细运行/检查步骤；新增「后续」夹爪与摇杆预留 |

---

## 阶段 1：`openarm_xr_bridge`（已完成 ✓）

**目的**：将 VR 手柄数据发布为 ROS 2 话题。

**发布话题**：

| 话题 | 类型 | 用途 |
|------|------|------|
| `/vr/left_controller/pose` | `geometry_msgs/PoseStamped` | IK 目标 |
| `/vr/right_controller/pose` | `geometry_msgs/PoseStamped` | IK 目标 |
| `/vr/left_grip` | `std_msgs/Float32` | 握把，使能手臂运动 |
| `/vr/right_grip` | `std_msgs/Float32` | 握把，使能手臂运动 |
| `/vr/left_gripper` | `std_msgs/Float32` | 扳机，映射夹爪 |
| `/vr/right_gripper` | `std_msgs/Float32` | 扳机，映射夹爪 |
| `/vr/controller_input` | `sensor_msgs/Joy` | 按钮 ABXY、摇杆，预留 |

### 1.1 编译（可直接复制执行）

```bash
conda activate pico_vr
cd ~/ros2_ws
colcon build --packages-select openarm_xr_bridge
```

预期：无报错，`Summary: 1 package finished`。

### 1.2 运行节点（可直接复制执行）

先启动 XRoboToolkit PC Service，再在新终端执行：

```bash
conda activate pico_vr
source ~/ros2_ws/install/setup.bash
~/ros2_ws/install/openarm_xr_bridge/share/openarm_xr_bridge/scripts/run_xr_bridge.sh
```

预期：打印 `XRoboToolkit SDK initialized.`、`XR Bridge node started, publishing at 50.0 Hz`，无崩溃。

### 1.3 检查话题存在（新终端，节点保持运行）

```bash
ros2 topic list | grep vr
```

预期输出应包含：
- `/vr/left_controller/pose`
- `/vr/right_controller/pose`
- `/vr/left_grip`
- `/vr/right_grip`
- `/vr/left_gripper`
- `/vr/right_gripper`
- `/vr/controller_input`

### 1.4 检查发布频率

```bash
ros2 topic hz /vr/left_controller/pose
```

预期：约 50 Hz（`average rate: 50.xxx`）。

### 1.5 检查位姿数据随 VR 变化

```bash
ros2 topic echo /vr/left_controller/pose
```

移动左手柄时，`position` 的 x/y/z 应有变化。按 Ctrl+C 退出。

### 1.6 检查握把（使能）

```bash
ros2 topic echo /vr/left_grip
```

握住左手柄握把时，`data` 应接近 1.0；松开接近 0。按 Ctrl+C 退出。

### 1.7 检查扳机（夹爪映射）

```bash
ros2 topic echo /vr/left_gripper
```

扣动左手柄扳机时，`data` 在 0~1 间变化。按 Ctrl+C 退出。

### 1.8 检查按钮与摇杆（预留）

```bash
ros2 topic echo /vr/controller_input
```

按 A/B/X/Y、推动摇杆时，`axes` 与 `buttons` 应有变化。按 Ctrl+C 退出。

---

## 阶段 2：`openarm_vr_ik_teleop` 骨架与 URDF 加载（已完成 ✓）

**目的**：创建包 2，验证 Placo 能加载 OpenArm 双臂 URDF；**不依赖 ROS、不依赖 VR**。

### 2.1 安装 Placo（pico_vr 环境）

```bash
conda activate pico_vr
pip install placo
```

若已有 xrobotoolkit_teleop 完整安装可跳过。安装可能需数分钟（含 pinocchio 等依赖）。

### 2.2 编译

```bash
cd ~/ros2_ws
colcon build --packages-select openarm_vr_ik_teleop
```

预期：`Summary: 1 package finished`，无报错。

### 2.3 运行 URDF 加载验证

```bash
conda activate pico_vr
source ~/ros2_ws/install/setup.bash
~/ros2_ws/install/openarm_vr_ik_teleop/share/openarm_vr_ik_teleop/scripts/run_check_urdf_placo.sh
```

预期输出包含：
- `[OK] Placo 成功加载 OpenArm 双臂 URDF`
- Joint 列表含 `openarm_left_joint1`…7、`openarm_right_joint1`…7
- 末端 link：`openarm_left_link7`、`openarm_right_link7`

### 2.4 检查项汇总

| # | 检查项 | 预期 |
|---|--------|------|
| 2.1 | 编译 | 无报错 |
| 2.2 | URDF 加载 | 打印 14 个 arm joint（左 7 + 右 7） |
| 2.3 | 末端 link | `openarm_left_link7`、`openarm_right_link7` 存在 |

---

## 阶段 3：IK 节点 + 仿真联调（假数据）（已完成 ✓）

**目的**：实现 IK 节点，订阅 VR 话题，发布 commands；用 `ros2 topic pub` 模拟 VR，**不需真 VR**。

### 3.1 编译

```bash
cd ~/ros2_ws
colcon build --packages-select openarm_vr_ik_teleop
```

### 3.2 启动 bringup（终端 1）

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=true robot_controller:=forward_position_controller
```

预期：RViz 打开，双臂显示。

### 3.3 启动 IK 节点（终端 2）

```bash
conda activate pico_vr
source ~/ros2_ws/install/setup.bash
~/ros2_ws/install/openarm_vr_ik_teleop/share/openarm_vr_ik_teleop/scripts/run_ik_teleop.sh
```

预期：`IK teleop node started, scale=0.80, rate=50.0 Hz`。

### 3.4 用 pub 模拟 grip 与 pose（终端 3 + 4，需同时运行）

**grip 与 pose 必须同时发布**：grip 使能手臂控制，pose 提供目标位姿，缺一不可。

**终端 3** — 激活左手臂（握把 > 0.9）：

```bash
ros2 topic pub -r 50 /vr/left_grip std_msgs/msg/Float32 "{data: 1.0}"
```

**终端 4** — 发布左手柄位姿（头显世界系，单位 m，四元数 wxyz）：

```bash
ros2 topic pub -r 50 /vr/left_controller/pose geometry_msgs/msg/PoseStamped "header: {frame_id: 'headset_world'}
pose:
  position: {x: 0.2, y: 0.0, z: 0.5}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

修改 `position.x`、`position.y`、`position.z` 可观察左臂跟随。松开握把时，终端 3 改为 `"{data: 0.0}"` 再 pub。

### 3.5 检查 commands 频率

```bash
ros2 topic hz /left_forward_position_controller/commands
```

预期：约 50 Hz。

### 3.6 检查项汇总

| # | 检查项 | 预期 |
|---|--------|------|
| 3.1 | bringup 已启动 | RViz 双臂 |
| 3.2 | pub 模拟 grip 与 pose | 话题有数据 |
| 3.3 | IK 节点启动 | 无报错 |
| 3.4 | commands 频率 | 约 50 Hz |
| 3.5 | 手臂随 pose 变化 | 左臂跟随左手柄位姿 |

---

## 阶段 4：全链路验证（真 VR）

**目的**：bringup + xr_bridge + ik_teleop 同时运行，真 VR 控制仿真双臂。

### 前置条件

- XRoboToolkit PC Service 已启动
- VR 头显已连接并正常工作
- conda 环境 `pico_vr` 已安装 placo、xrobotoolkit_sdk

### 4.1 启动方式

**一键启动**（推荐）：`./scripts/launch_vr_teleop.sh`，支持 `--fake`/`--real`、`--bimanual`/`--unimanual`。详见 `vr_deployment_guide.md`。

**分终端启动**：

**终端 1 — bringup**

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=true robot_controller:=forward_position_controller
```

预期：RViz 打开，双臂模型显示。

**终端 2 — xr_bridge**（发布 VR 手柄数据）

```bash
conda activate pico_vr
source <workspace>/install/setup.bash
<workspace>/install/openarm_xr_bridge/share/openarm_xr_bridge/scripts/run_xr_bridge.sh
```

预期：`XRoboToolkit SDK initialized.`、`XR Bridge node started, publishing at 50.0 Hz`。脚本会根据自身位置解析 install 路径。

**终端 3 — ik_teleop**（IK 解算并发布 commands）

```bash
conda activate pico_vr
source <workspace>/install/setup.bash
ros2 launch openarm_vr_ik_teleop ik_teleop.launch.py
# 若抖动明显，可加参数：scale_factor:=0.6
```

预期：`IK teleop node started, scale=0.80, rate=50 Hz`。

### 4.2 握把激活手臂跟随

戴上头显，握住左手柄握把（grip > 0.9），移动左手；左臂应跟随。松开握把，左臂保持。右手同理。

### 4.3 双臂独立控制

左手握把控制左臂，右手握把控制右臂，互不干扰。

### 4.4 无明显抖动

手臂跟随应平滑。**冗余自由度处理**：仅使用 kinetic_energy 正则（1e-6）；manipulability、joints_task 与 OpenArm 双臂模型组合易触发 QP NaN，不予启用。若有关节跳变，可降低 scale_factor：

**VR 专用关节限位（仅 ik_teleop 生效）**：为减轻 IK 解算跳变，对左右 joint4 施加更严格 lower：从 0.0 收紧至 0.15 rad。测试后可考虑增加其他关节的 VR 专用限制。
```bash
ros2 launch openarm_vr_ik_teleop ik_teleop.launch.py scale_factor:=0.6
```

### 4.5 关节顺序正确

左臂 7 关节、右臂 7 关节顺序与 `openarm_vr_topics.md` 一致；手臂运动方向与手柄移动方向合理对应。

### 4.6 检查项汇总

| # | 检查项 | 预期 |
|---|--------|------|
| 4.1 | 按序启动三端 | 无报错，三进程稳定运行 |
| 4.2 | 握把激活手臂跟随 | 握把 > 0.9 时手臂随手柄移动 |
| 4.3 | 双臂独立控制 | 左右手独立控制对应手臂 |
| 4.4 | 无明显抖动 | 运动平滑 |
| 4.5 | 关节顺序正确 | 方向与幅度符合预期 |

---

## 后续（可选）

- **夹爪**：订阅 `/vr/left_gripper`、`/vr/right_gripper`，转发到 `left_gripper_controller` / `right_gripper_controller` 的 Action
- **摇杆/按钮**：`/vr/controller_input` 用于底盘、急停、模式切换等，按需扩展

---

## 实现顺序汇总

| 阶段 | 内容 | 依赖 |
|------|------|------|
| 0 | 环境预检 | 无 |
| 1 | openarm_xr_bridge | 0、PC Service、Python 3.10 |
| 2 | openarm_vr_ik_teleop 骨架 | 0 |
| 3 | IK 节点 + 假数据联调 | 1、2、bringup |
| 4 | 全链路真 VR | 1、2、3 |

---

## 附录：参考文件

| 用途 | 路径 |
|------|------|
| VR 接口 | `XRoboToolkit-Teleop-Sample-Python/xrobotoolkit_teleop/common/xr_client.py` |
| Placo IK | `xrobotoolkit_teleop/common/base_teleop_controller.py` |
| 几何 | `xrobotoolkit_teleop/utils/geometry.py` |
| 双臂硬件 | `xrobotoolkit_teleop/hardware/dual_arm_ur_controller.py` |
| 命令格式 | `openarm_vr_topics.md` |
