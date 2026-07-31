# openarm_vr_ik_teleop

**功能包作用**：将 VR 手柄位姿、握把与扳机状态转换为 OpenArm 双臂关节目标与夹爪命令，通过 Placo IK 解算后发布到 `forward_position_controller` 与夹爪 Action，实现 VR 遥操作机械臂。握把激活时，手柄相对位移/旋转映射为末端位移/旋转；**扳机**：按下(1)=夹爪闭合，松开(0)=夹爪全开。

## 输入 / 输出话题

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `/joint_states` | `sensor_msgs/JointState` | 当前关节状态，用于同步 Placo 模型与 IK 初始解 |
| 订阅 | `/vr/left_controller/pose` | `geometry_msgs/PoseStamped` | 左手柄位姿（frame_id: headset_world） |
| 订阅 | `/vr/right_controller/pose` | `geometry_msgs/PoseStamped` | 右手柄位姿 |
| 订阅 | `/vr/left_grip` | `std_msgs/Float32` | 左手柄握把，> 0.9 激活左臂跟随 |
| 订阅 | `/vr/right_grip` | `std_msgs/Float32` | 右手柄握把，> 0.9 激活右臂跟随 |
| 订阅 | `/vr/left_gripper` | `std_msgs/Float32` | 左手柄扳机：0=夹爪开，1=夹爪闭 |
| 订阅 | `/vr/right_gripper` | `std_msgs/Float32` | 右手柄扳机：0=夹爪开，1=夹爪闭 |
| 发布 | `/left_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 左臂 7 关节目标位置（弧度） |
| 发布 | `/right_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 右臂 7 关节目标位置（弧度） |
| Action | `left_gripper_controller/gripper_cmd` | `control_msgs/GripperCommand` | 左夹爪目标开口 (m) |
| Action | `right_gripper_controller/gripper_cmd` | `control_msgs/GripperCommand` | 右夹爪目标开口 (m) |

## 参数

- `scale_factor` (float): VR 位移到末端的缩放，默认 0.8
- `control_rate` (float): 控制频率 Hz，默认 50
- `grip_threshold` (float): 握把激活阈值，默认 0.9
- `gripper_open_m` (float): 夹爪全开时的关节位置 (m)，默认 0.044
- `gripper_closed_m` (float): 夹爪闭合时的关节位置 (m)，默认 0.0
- `gripper_update_throttle_s` (float): 夹爪 Action 发送节流间隔 (s)，默认 0.1

可通过环境变量覆盖：`IK_SCALE_FACTOR`、`IK_GRIP_THRESHOLD`。

## 夹爪映射

扳机值 0~1 与夹爪开口的对应关系：

| 扳机状态 | 话题值 | 夹爪状态 |
|----------|--------|----------|
| 松开 | 0 | 全开 (0.044 m) |
| 按下 | 1 | 闭合 (0 m) |

中间值线性插值。按下扳机即闭合夹爪，松开即打开，符合抓取直觉。

## 依赖与运行

需 conda 环境 `pico_vr`（含 placo）。bringup 与 xr_bridge 需先启动。

```bash
conda activate pico_vr
source <workspace>/install/setup.bash
ros2 launch openarm_vr_ik_teleop ik_teleop.launch.py
# 或脚本: <workspace>/install/openarm_vr_ik_teleop/share/openarm_vr_ik_teleop/scripts/run_ik_teleop.sh
```

## URDF 加载验证（阶段 2）

验证 Placo 能否正确加载 OpenArm 双臂 URDF：

```bash
conda activate pico_vr
source <workspace>/install/setup.bash
<workspace>/install/openarm_vr_ik_teleop/share/openarm_vr_ik_teleop/scripts/run_check_urdf_placo.sh
```

预期：`[OK] Placo 成功加载 OpenArm 双臂 URDF`，Joint 与 link7 列表正确。
