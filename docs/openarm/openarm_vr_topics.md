# OpenArm ROS 2 工作区说明：双臂 `forward_position_controller`

**文档用途**：说明在**双臂**配置下，使用 **`forward_position_controller`** 时的 ROS 话题、消息类型与数据格式，便于 VR 等高频流式位置控制对接。用于查阅命令话题、关节顺序和消息格式时参考。

## 何时用 `forward_position_controller`

- **`joint_trajectory_controller`**：适合带时间戳的轨迹（`FollowJointTrajectory` action），例如 MoveIt、示教回放。VR 每帧一个新目标时，反复取消/重发 action 开销大，语义也不匹配。
- **`forward_position_controller`**（底层类型为 `forward_command_controller/ForwardCommandController`）：每个控制周期把**当前一帧位置指令**交给硬件接口，适合**固定频率流式更新**（例如经滤波后的 50–100 Hz）。

若 VR 输出的是**关节空间目标位置**（弧度），通常选 **`forward_position_controller`**。若输出的是**速度/增量**，可考虑配置中的 **`forward_velocity_controller`**，或在上层做 IK/积分后再发位置。

**注意**：应对原始指令做滤波、限速与工作空间限制；真机还需急停、限位等安全策略。

## 如何启动（双臂 + 该控制模式）

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  arm_type:=v10 \
  use_fake_hardware:=true \
  robot_controller:=forward_position_controller
```

`robot_controller` 为 launch 参数，会决定加载哪一对手臂控制器（与 `can_interface` 等无关）。配置定义见：

`src/arm/openarm_ros2/openarm_bringup/config/v10_controllers/openarm_v10_bimanual_controllers.yaml`

## 话题与消息类型（双臂）

左、右臂各有一个控制器，因此需要**两个命令话题**（不是单话题里发 14 个数）：

| 臂 | 话题（默认无命名空间） | 消息类型 |
|----|------------------------|----------|
| 左 | `/left_forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray` |
| 右 | `/right_forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray` |

若启动时设置了 `arm_prefix`（命名空间），话题形如：

`<prefix>/left_forward_position_controller/commands`  
`<prefix>/right_forward_position_controller/commands`

## `Float64MultiArray` 数据格式

- 字段 **`data`**：长度必须为 **7**，单位为**弧度 (rad)**。
- 元素顺序与 yaml 中该控制器的 `joints` 列表一致：
  - **左臂**：`openarm_left_joint1` … `openarm_left_joint7`
  - **右臂**：`openarm_right_joint1` … `openarm_right_joint7`
- **`layout`** 一般可忽略，控制器主要使用 `data` 的长度与顺序。

## 双臂同时控制时的发布方式

- 使用**两个 publisher**，分别发布到左、右 `commands`；或
- 在一个节点内**依次/并行**向上述两个话题各发布一条消息。

## 夹爪

左右手臂的 7 关节 forward 命令**不包含夹爪**。双臂 launch 会另外加载 `left_gripper_controller` / `right_gripper_controller`（`GripperActionController`），需通过 **Action** 控制，而不是上述 `Float64MultiArray` 话题。

## 自检命令

```bash
ros2 topic list | grep forward
ros2 topic info /left_forward_position_controller/commands
ros2 interface show std_msgs/msg/Float64MultiArray
```

若话题名与上文不一致，以 `ros2 topic list` 实际输出为准。

## 相关代码位置

- Launch 参数与控制器加载：`src/arm/openarm_ros2/openarm_bringup/launch/openarm.bimanual.launch.py`
- 控制器与关节名：`src/arm/openarm_ros2/openarm_bringup/config/v10_controllers/openarm_v10_bimanual_controllers.yaml`

更完整的启动与仿真说明见同目录下的 `openarm_setup_guide.md`。

---

## openarm_xr_bridge 发布的话题（VR → 各控制节点）

| 话题 | 类型 | 用途 |
|------|------|------|
| `/vr/left_controller/pose` | `geometry_msgs/PoseStamped` | 左手柄位姿，IK 节点订阅 |
| `/vr/right_controller/pose` | `geometry_msgs/PoseStamped` | 右手柄位姿 |
| `/vr/left_grip` | `std_msgs/Float32` | 握把 (0~1)，使能手臂运动 |
| `/vr/right_grip` | `std_msgs/Float32` | 握把，使能手臂运动 |
| `/vr/left_gripper` | `std_msgs/Float32` | 扳机：0=夹爪开，1=夹爪闭 |
| `/vr/right_gripper` | `std_msgs/Float32` | 扳机：0=夹爪开，1=夹爪闭 |
| `/vr/controller_input` | `sensor_msgs/Joy` | 按钮 ABXY、摇杆，预留。axes: [左x,左y,右x,右y]；buttons: [A,B,X,Y,左菜单,右菜单,左摇杆点击,右摇杆点击] |
