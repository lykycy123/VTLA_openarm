# VR → ROS 通路完整检验方法

在 **OpenArm bringup、IK、RViz 全部关闭** 的前提下，只验证 **PICO / XRoboToolkit → PC Service → `openarm_xr_bridge` → ROS 2 话题** 这一段是否正常。

---

## 一、检验前环境（一次性确认）

| 项 | 说明 |
|----|------|
| 系统 | Ubuntu 22.04 + ROS 2 Humble（`/opt/ros/humble`） |
| 工作空间 | 已 `colcon build` 且存在 `install/setup.bash`（下文以 `$WS` 表示，例：`export WS=~/ros2_ws`） |
| Python | `pico_vr` 或 `pico_vr_env` 为 **Python 3.10**，且能 `import xrobotoolkit_sdk` |
| 网络 | PICO 头显与 PC **同一局域网**，头显已装 XRoboToolkit APK 并已连接 |
| PC Service | XRoboToolkit PC Service **必须先于** `xr_bridge` 启动 |

关闭旧进程（可选）：

```bash
# 若有残留 ros2 launch / python 节点，可先结束对应终端或：
pkill -f xr_bridge_node 2>/dev/null || true
```

---

## 二、最小链路：只启动 PC Service + xr_bridge

### 步骤 1：启动 PC Service（单独终端，保持运行）

```bash
bash /opt/apps/roboticsservice/runService.sh
```

若安装路径不同，以本机实际 `runService.sh` 为准。

### 步骤 2：启动 XR Bridge（第二个终端）

```bash
conda activate pico_vr
# 若用解压的 env：  source ~/pico_vr_env/bin/activate

source /opt/ros/humble/setup.bash
source $WS/install/setup.bash

$WS/install/openarm_xr_bridge/share/openarm_xr_bridge/scripts/run_xr_bridge.sh
```

**成功标志（终端日志）**：

- `XRoboToolkit SDK initialized.`
- `XR Bridge node started, publishing at 50.0 Hz`（默认 `publish_rate` 为 50）
- 无持续报错；可能出现设备连接、串流相关 INFO。

此时 **不要** 启动 `openarm_bringup`、`ik_teleop`。

---

## 三、ROS 侧检验命令（第三个终端）

同样先加载环境：

```bash
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
```

### 3.1 节点是否存在

```bash
ros2 node list | grep xr_bridge
```

预期：存在 `xr_bridge_node`（或类似名称）。

### 3.2 话题是否存在

```bash
ros2 topic list | grep '^/vr/'
```

预期至少包含（与 `xr_bridge_node.py` 一致）：

| 话题 | 类型 |
|------|------|
| `/vr/left_controller/pose` | `geometry_msgs/msg/PoseStamped` |
| `/vr/right_controller/pose` | `geometry_msgs/msg/PoseStamped` |
| `/vr/left_grip` | `std_msgs/msg/Float32` |
| `/vr/right_grip` | `std_msgs/msg/Float32` |
| `/vr/left_gripper` | `std_msgs/msg/Float32` |
| `/vr/right_gripper` | `std_msgs/msg/Float32` |
| `/vr/controller_input` | `sensor_msgs/msg/Joy` |

### 3.3 发布频率（通路是否“活”）

```bash
ros2 topic hz /vr/left_controller/pose
ros2 topic hz /vr/left_grip
```

- **正常**：接近节点参数 `publish_rate`（默认 **约 50 Hz**），且较稳定。
- **异常**：长期无输出、频率极低 → 多为 PC Service / SDK / 头显连接问题，而非下游 IK。

按 `Ctrl+C` 退出 `hz`。

### 3.4 内容是否随手柄变化（核心）

**左手位姿**（缓慢平移、旋转手柄，数值应明显变化）：

```bash
ros2 topic echo /vr/left_controller/pose --no-arr
```

**右手**：

```bash
ros2 topic echo /vr/right_controller/pose --no-arr
```

关注：

- `header.frame_id`：默认 **`headset_world`**（可用参数 `frame_id` 修改）。
- `header.stamp`：随时间更新。
- `pose.position`、`pose.orientation`：运动时持续变化。

**侧握把（手臂使能逻辑会用到）**：

```bash
ros2 topic echo /vr/left_grip
```

未捏时接近 0，用力捏一侧握把应升高，**接近 1.0** 为理想。

**扳机（夹爪映射用）**：

```bash
ros2 topic echo /vr/left_gripper
```

松开约 0，扣到底接近 1。

### 3.5 可选：看发布者与类型

```bash
ros2 topic info -v /vr/left_controller/pose
```

确认 **Publisher** 为 `xr_bridge_node`，类型为 `geometry_msgs/msg/PoseStamped`。

---

## 四、判定标准汇总

| 检查项 | 通过标准 |
|--------|----------|
| SDK 日志 | 出现 `XRoboToolkit SDK initialized` |
| 话题 | `/vr/*` 完整且能 `ros2 topic list` 看到 |
| 频率 | 主位姿话题 `ros2 topic hz` 约为 `publish_rate`（默认 ~50 Hz） |
| 位姿 | 动手柄时 `pose` 的 position/orientation **明显变化** |
| 握把/扳机 | `*_grip`、`*_gripper` 在 0~1 范围内随输入变化 |

**以上全部满足** → **VR → ROS 这一段正常**，问题若在机械臂不动，应转查 **bringup / 控制器 / `ik_teleop` / `grip_threshold`** 等下游。

**任一项不满足** → 保持本最小配置继续排查（下表），**不要**先加 IK 与 Gazebo。

---

## 五、常见问题对照

| 现象 | 可能原因 | 建议 |
|------|----------|------|
| `import xrobotoolkit_sdk` 失败 | 未激活 pico_vr / 路径不对 | 确认 `conda activate pico_vr` 后再起节点 |
| `SDK init failed` | PC Service 未起或端口被占 | 先起 `runService.sh`，再看防火墙 |
| 无 `/vr/*` 或 hz 极低 | 节点未起或崩溃 | 看 `xr_bridge` 终端报错 |
| 有话题但 pose 不变 | 头显未连、追踪关闭、或只连了 PC 未进应用 | 查 PICO 侧与 WiFi |
| grip 最大值 < 0.9 | 设备量程差异 | IK 侧可调低 `grip_threshold`（见 VR 遥操文档） |
| `rclpy` / Python 版本错误 | 非 3.10 | Humble 需 Python 3.10 的 ROS 环境 |

---

## 六、检验结束后

- 在 `xr_bridge` 终端按 **Ctrl+C** 结束节点。
- 再停 PC Service 终端（若有说明要求按特定方式退出则按其文档）。

---

*本流程仅覆盖 **XR Bridge**；不包含 `openarm_vr_ik_teleop` 与 `ros2_control`。*
