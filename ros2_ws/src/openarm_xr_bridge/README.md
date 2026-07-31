# openarm_xr_bridge

**功能包作用**：从 XRoboToolkit PC Service 读取 PICO VR 手柄/头显数据，发布为 ROS 2 话题，供 `openarm_vr_ik_teleop` 等节点订阅。无 ROS 订阅，仅发布；数据来源于 `xrobotoolkit_sdk` 对接 PC Service。

## 输入 / 输出话题

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 发布 | `/vr/left_controller/pose` | `geometry_msgs/PoseStamped` | 左手柄位姿（位置 m，四元数 xyzw，frame_id: headset_world） |
| 发布 | `/vr/right_controller/pose` | `geometry_msgs/PoseStamped` | 右手柄位姿 |
| 发布 | `/vr/left_grip` | `std_msgs/Float32` | 左手柄握把 (0~1)，用于激活手臂跟随 |
| 发布 | `/vr/right_grip` | `std_msgs/Float32` | 右手柄握把 (0~1) |
| 发布 | `/vr/left_gripper` | `std_msgs/Float32` | 左手柄扳机：0=夹爪开，1=夹爪闭 |
| 发布 | `/vr/right_gripper` | `std_msgs/Float32` | 右手柄扳机：0=夹爪开，1=夹爪闭 |
| 发布 | `/vr/controller_input` | `sensor_msgs/Joy` | 按钮 ABXY、摇杆，预留。axes: [左摇杆x,y, 右摇杆x,y]；buttons: [A,B,X,Y,左/右菜单,左/右摇杆点击] |

无 ROS 订阅；输入来自 XRoboToolkit PC Service（通过 `xrobotoolkit_sdk`）。

## 参数

- `publish_rate` (float): 发布频率 Hz，默认 50.0
- `frame_id` (string): pose 的 frame_id，默认 `headset_world`

## 依赖与运行

- ROS 2 Humble（rclpy 需 Python 3.10）
- XRoboToolkit PC Service 已启动
- conda 环境 `pico_vr`（含 `xrobotoolkit_sdk`）

**务必先启动 PC Service**：`bash /opt/apps/roboticsservice/runService.sh`

```bash
conda activate pico_vr
source <workspace>/install/setup.bash
<workspace>/install/openarm_xr_bridge/share/openarm_xr_bridge/scripts/run_xr_bridge.sh
```

## 相关文档

- [openarm_vr_topics.md](../openarm_vr_topics.md) - OpenArm forward_position_controller 接口
- [vr_openarm_implementation.md](../vr_openarm_implementation.md) - VR 遥操作实现方案
