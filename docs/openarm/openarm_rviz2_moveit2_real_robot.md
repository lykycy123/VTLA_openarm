# 通过 RViz2 + MoveIt2 控制 OpenArm 实体机械臂

本文说明**在不使用 VR** 的前提下，如何通过 **RViz2** 与 **MoveIt2** 驱动 **实体 OpenArm（CAN 实物）**：涉及启动命令、ROS 2 侧控制接口（控制器 / Action）、以及在 RViz 中的人机接口。

---

## 1. 控制链路概览（实体臂）

```
RViz2（MotionPlanning 插件）
    → move_group（规划与碰撞检查）
        → FollowJointTrajectory / GripperCommand（ROS 2 Action）
            → ros2_control（joint_trajectory_controller、gripper_controller）
                → 实体机械臂（CAN 硬件接口）
```

- **人机侧接口**：RViz2 中 **MotionPlanning** 面板、末端/关节 **Interactive Marker**、**Plan** / **Plan and Execute**。
- **机器人侧接口**：MoveIt 通过 `moveit_simple_controller_manager` 将轨迹下发到下列 **控制器名称**（与包内 `moveit_controllers.yaml` 一致）：

  | 类型 | 控制器名称 | Action 类型 | `action_ns` |
  |------|------------|-------------|-------------|
  | 左臂 7 关节轨迹 | `left_joint_trajectory_controller` | `control_msgs/action/FollowJointTrajectory` | `follow_joint_trajectory` |
  | 右臂 7 关节轨迹 | `right_joint_trajectory_controller` | 同上 | `follow_joint_trajectory` |
  | 左夹爪 | `left_gripper_controller` | `control_msgs/action/GripperCommand` | `gripper_cmd` |
  | 右夹爪 | `right_gripper_controller` | 同上 | `gripper_cmd` |

因此：**在实体模式下，必须通过 `joint_trajectory_controller` 双臂接口**（除非自行修改 MoveIt 配置以适配 `forward_position_controller`）。本仓库 MoveIt 默认绑定的是 **关节轨迹控制器**，不是 VR 遥操里用的 forward position 命令话题。

---

## 2. 推荐：一键启动（ros2_control + move_group + RViz2）

一体化启动文件：`openarm_bimanual_moveit_config` 包中的 `demo.launch.py`。  
它会启动：

- `ros2_control_node` + 实物参数（`use_fake_hardware:=false`）
- `joint_state_broadcaster`
- 左右臂 `joint_trajectory_controller`、左右 `gripper_controller`
- `move_group`
- `rviz2`，并加载包内 `config/moveit.rviz`

### 命令示例

```bash
source $HOME/Medibot_openarm/openarm_ws/install/setup.bash

ros2 launch openarm_bimanual_moveit_config demo.launch.py \
  use_fake_hardware:=false \
  robot_controller:=joint_trajectory_controller \
  right_can_interface:=can0 \
  left_can_interface:=can1
```

### 参数说明

| 参数 | 含义 |
|------|------|
| `use_fake_hardware:=false` | 使用真实 CAN 硬件，而非仿真假硬件 |
| `robot_controller:=joint_trajectory_controller` | 与 MoveIt 的 `FollowJointTrajectory` 接口一致（**实体 + MoveIt 时保持此值**） |
| `right_can_interface` / `left_can_interface` | 右臂 / 左臂 CAN 口名，按实际接线修改（默认 `can0` / `can1`） |

---

## 3. 在 RViz2 中操作实体臂（人机接口）

启动成功后：

1. 打开 **MotionPlanning** 显示/面板（随 `moveit.rviz` 配置应已加载）。
2. 选择 **Planning Group**（左臂 / 右臂等，由 SRDF 定义）。
3. 使用 **Interactive Marker** 拖动末端，或在 **Planning** 页设置目标。
4. 点击 **Plan**，确认轨迹合理后点击 **Plan and Execute**，将轨迹发送到上述 **joint trajectory / gripper Action**，由 `ros2_control` 在实体上执行。

**安全建议**：首次上电先在小幅、低速下验证方向与限位；确保急停与现场人员安全距离符合规范。

---

## 4. 可选：分两终端（先 bringup，后仅 MoveIt）

若需先单独拉起控制栈、再调试 MoveIt：

**终端 A — 仅机器人控制栈（关闭自带 RViz 避免重复）**

```bash
source $HOME/Medibot_openarm/openarm_ws/install/setup.bash

ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=false \
  robot_controller:=joint_trajectory_controller \
  launch_rviz:=false \
  right_can_interface:=can0 \
  left_can_interface:=can1
```

**终端 B — MoveIt（`move_group`）+ RViz2**

需与本工作空间中的 MoveIt 启动方式一致（例如 `move_group.launch.py` 与带 MoveIt 的 RViz）。注意：**仅**启动 RViz 而不启动 `ros2_control` 时，无法驱动实体臂。  
多数情况下，直接使用第 2 节的 **`demo.launch.py` 一条命令更简单**。

---

## 5. 与 VR 遥操的区别（避免混用接口）

| 场景 | 典型接口 |
|------|----------|
| **RViz2 + MoveIt2（本文）** | `move_group` → `joint_trajectory_controller` / `*_gripper_controller`（Action） |
| **VR 遥操（openarm_vr_ik_teleop）** | `/left_forward_position_controller/commands` 等 **话题** + 夹爪 Action |

实体臂可以只开 MoveIt 栈，不必运行 `openarm_xr_bridge` 或 `openarm_vr_ik_teleop`。

---

## 6. 参考与源码位置

- 一体化 launch：`src/arm/openarm_ros2/openarm_bimanual_moveit_config/launch/demo.launch.py`
- MoveIt 控制器映射：`src/arm/openarm_ros2/openarm_bimanual_moveit_config/config/moveit_controllers.yaml`
- 双臂 bringup：`src/arm/openarm_ros2/openarm_bringup/launch/openarm.bimanual.launch.py`
- 官方文档：<https://docs.openarm.dev/software/ros2/install>

---

*文档对应工作空间：`$HOME/Medibot_openarm/openarm_ws`。若安装路径不同，请将 `source` 与 `ros2 launch` 中的路径改为本地 `install/setup.bash`。*
