<h2 style="text-align: center;">功能启动说明</h2>

**文档用途**：OpenArm 机械臂的系统配置、CAN 接口、启动与仿真说明。用于首次搭建环境、配置真机通信、以及日常启动与调试时参考。

### 简介

OpenArm是一个开源的7自由度类人机械臂，专为物理AI研究和接触丰富环境中的部署而设计。具有高反向驱动性和柔顺性，在提供实际载荷能力的同时，优异于安全的人机交互

### 系统要求

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.8+

### 设置接口波特率,使用canfd脚本一键激活

``` bash
cd openarm_ws
sudo ./src/arm/openarm_can/setup/configure_socketcan.sh can0 -fd
sudo ./src/arm/openarm_can/setup/configure_socketcan.sh can1 -fd
```

**电机通信测试**

监控 CAN 流量：

```bash
candump can0
```

保持此运行以观察所有 CAN 消息。

在终端 2 中，向电机 #1 发送电机启用命令(右臂can0，左臂can1)

```bash
cansend can0 001#FFFFFFFFFFFFFFFC
# 禁用电机
cansend can0 001#FFFFFFFFFFFFFFFD
```

脚本测试

``` b
cd openarm_can/build
./motor-check 1 17 can0
```

### 编译

```bash
colcon build
source ~/Downloads/WorkSpace/hao/Medibot_openarm/openarm_ws/install/setup.bash
```

若编译时出现 `AMENT_PREFIX_PATH` 指向已删除路径（例如旧的 `qnbot_teleoperator`）的警告，请新开终端，或先执行 `source /opt/ros/humble/setup.bash` 再 `source ~/Downloads/WorkSpace/hao/Medibot_openarm/openarm_ws/install/setup.bash`，并检查 `~/.bashrc` 是否仍 source 了旧的 `install` 配置。

#### 设置环境变量

```bash
# 建议添加到 ~/.bashrc 中
echo "source ~/Downloads/WorkSpace/hao/Medibot_openarm/openarm_ws/install/setup.bash" >> ~/.bashrc
```

### 启动基础控制

#### 使用假硬件启动OpenARM

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=true
```

**启动文件**

- `openarm.launch.py`- 单臂配置
- `openarm.bimanual.launch.py`- 双臂配置

**关键参数**

- `arm_type`- 手臂类型（默认：v10）
- `use_fake_hardware`- 使用假硬件而不是真硬件（默认：false）
- `can_interface`- 要使用的 CAN 接口（默认：can0）
- `robot_controller`- 控制器类型：joint_trajectory_controller 或 forward_position_controller

当您运行启动文件时，将启动机器人状态发布器、控制器管理器等。

成功启动控制器后，可以通过检查可用作来验证它们是否正常工作：

```sh
ros2 action list
```

要测试关节运动，请发送一个简单的轨迹命令：

```sh
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{trajectory: {joint_names: ["openarm_joint1", "openarm_joint2", "openarm_joint3", "openarm_joint4", "openarm_joint5", "openarm_joint6", "openarm_joint7"], points: [{positions: [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15], time_from_start: {sec: 3, nanosec: 0}}]}}'
```

此命令在 3 秒内将所有手臂关节移动到 0.15 弧度位置。

#### RViz配置

启动后RViz会自动打开，需要进行以下配置：

1. **修改Fixed Frame设置**：
   - 展开左侧 `Global Options`
   - 将 `Fixed Frame` 从 `map` 改为 `openarm_link0`

2. **添加机器人模型显示**：
   - 点击左下角的 `Add` 按钮
   - 选择 `RobotModel`
   - 确保 `Robot Description` 设置为 `/robot_description`

3. **验证显示**：
   - 机械臂模型应该正确显示在RViz中
   - 可以看到7个关节和夹爪
   - 第五步：MoveIt2集成

#### 安装MoveIt2(已安装忽略)

**选项1：核心包安装（~100MB，推荐）**

```bash
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit ros-humble-moveit-ros-perception
```

**选项2：完整套件安装（~1.5GB）**


```bash
sudo apt install ros-humble-moveit-* ros-humble-moveit-configs-utils
```

#### 启动MoveIt

``` bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py
```

注意：如果没有连接真实机械臂必须运行`ros2 launch openarm_bimanual_moveit_config demo.launch.py arm_type:=v10` 会显示显示坐标转换错误，注意添加`arm_type:=v10 use_fake_hardware:=true`

#### 验证MoveIt2功能

- 启动后应该看到MoveIt2的运动规划界面
- 可以通过拖拽交互式标记来规划机械臂运动
- 支持双臂协调控制

### 重力补偿

```bash
# 重力补偿启动(单臂)
cd openarm_teleop
./script/launch_grav_comp.sh <arm_side> <can_interface> <urdf_path>
# 默认参数：
# arm_side: right_arm
# can_interface: can0
# arm_type: v10
```

### 双臂力反馈启动

```bash
# right_arm（单臂、默认 CAN 总线为 can0 can2）
./script/launch_unilateral.sh right_arm can0 can2

# left_arm（默认 CAN 总线为 can1 can3）
./script/launch_bilateral.sh left_arm can1 can3
./script/launch_bilateral.sh right_arm can0 can2
```

1. 主程序openarm_bilateral_control.cpp

   - 初始化主从两个OpenArm实例
   - 加载配置文件参数（[leader.yaml](file:///E:/ROS_project/11111/openarm_ws/src/arm/openarm_teleop/config/leader.yaml)和[follower.yaml](file:///E:/ROS_project/11111/openarm_ws/src/arm/openarm_teleop/config/follower.yaml)）
   - 创建多个线程：
     - LeaderArmThread：控制主臂
     - FollowerArmThread：控制从臂
     - AdminThread：管理主从臂状态同步

2. 控制线程

   - [LeaderArmThread](file:///E:/ROS_project/11111/openarm_ws/src/arm/openarm_teleop/control/openarm_bilateral_control.cpp#L29-L55)和[FollowerArmThread](file:///E:/ROS_project/11111/openarm_ws/src/arm/openarm_teleop/control/openarm_bilateral_control.cpp#L57-L83)：分别调用[Control::bilateral_step()](file:///E:/ROS_project/11111/openarm_ws/src/arm/openarm_teleop/src/controller/control.hpp#L83-L83)执行控制逻辑
   - [AdminThread](file:///E:/ROS_project/11111/openarm_ws/src/arm/openarm_teleop/control/openarm_bilateral_control.cpp#L85-L114)：同步主从臂状态

3. 核心控制逻辑

   Control::bilateral_step()

   - 获取电机状态并转换为关节状态
   - 计算重力、科里奥利力和摩擦力补偿
   - 根据主从关系计算控制扭矩
   - 发送MIT控制指令

两个控制模式都使用YAML配置文件：

- [leader.yaml](file:///E:/ROS_project/11111/openarm_ws/src/arm/openarm_teleop/config/leader.yaml)：主臂控制参数
- [follower.yaml](file:///E:/ROS_project/11111/openarm_ws/src/arm/openarm_teleop/config/follower.yaml)：从臂控制参数

配置包括：

- Kp/Kd：位置和速度控制增益
- 摩擦模型参数（Fc、k、Fv、Fo）

核心类和组件

1. **[Control](file:///E:/ROS_project/11111/openarm_ws/src/arm/openarm_teleop/src/controller/control.hpp#L31-L100)**：主要控制类，实现重力补偿和双边控制逻辑
2. **[Dynamics](file:///E:/ROS_project/11111/openarm_ws/src/arm/openarm_teleop/src/controller/dynamics.hpp#L32-L70)**：动力学计算类，使用KDL库计算重力、科里奥利力等
3. **[RobotSystemState](file:///E:/ROS_project/11111/openarm_ws/src/arm/openarm_teleop/src/robot_state.hpp#L73-L165)**：机器人状态管理类
4. **[PeriodicTimerThread](file:///E:/ROS_project/11111/openarm_ws/src/arm/openarm_teleop/src/periodic_timer_thread.hpp#L25-L121)**：定时控制线程基类

### 双臂启动

``` bash
# 轨迹控制
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10
```

- `arm_type`- 手臂类型（默认：v10）
- `use_fake_hardware`- 使用假硬件而不是真硬件（默认：false）
- `can_interface`- 要使用的 CAN 接口（默认：can0）
- `robot_controller`- 控制器类型：joint_trajectory_controller 或 forward_position_controller

### 网页预设动作测试

在Ubuntu终端运行

``` bash
# 启动websocket连接
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 硬件控制启动
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 robot_controller:=forward_position_controller
```

浏览器打开

``` bash
http://visual.svtrobot.com/

# 输入工控机ip地址，点击连接，点右下角边开始运动
```

