# `launch_vr_teleop.sh` 使用说明（仿真 / 实物）

本文说明如何用 `openarm_ws/scripts/teleop/launch_vr_teleop.sh` 分别启动 **假硬件仿真** 与 **实物机械臂** 的 VR 遥操作，以及启动前必须完成的 VR 连接与 CAN 接线对应关系。

---

## 重要：先完成 VR 连接，再运行脚本

脚本会依次启动 **bringup**、**xr_bridge**、**ik_teleop**。其中 `xr_bridge` 依赖 XRoboToolkit 与头显数据，若 VR 未就绪，容易出现连接失败或无手柄数据。

**建议在运行本脚本之前完成：**

1. **启动 PC 端服务**（单独终端保持运行）  
   `bash /opt/apps/roboticsservice/runService.sh`

2. **PICO 头显**：开机、已安装并打开对应 APK，与 PC **处于同一局域网**。

3. **Python 环境**：已配置 `openarm`（`conda activate openarm` 或 `source ~/openarm_env/bin/activate`）。脚本会尝试自动激活；若环境缺失会报错退出。

4. **（仅实物）** SocketCAN 已配置，左右臂 CAN 与线缆一致（见下文「实物机械臂 CAN 对应关系」），电机已按现场流程使能。

上述就绪后，再执行下面的启动命令。

---

## 工作目录与基本命令

在 **仓库根目录**（`~/Medibot_openarm`）执行（将路径换成你的机器上的实际路径）：

```bash
cd $HOME/Medibot_openarm
./openarm_ws/scripts/teleop/launch_vr_teleop.sh [选项...]
```

查看全部选项：

```bash
./openarm_ws/scripts/teleop/launch_vr_teleop.sh --help
```

启动过程中若未加 `--skip-check`，脚本会列出检查项并要求 **按 Enter 继续**；结束进程请在该终端 **Ctrl+C**。

---

## 控制假硬件（仿真）机械臂

默认即为假硬件，以下两种写法等价：

```bash
./openarm_ws/scripts/teleop/launch_vr_teleop.sh
./openarm_ws/scripts/teleop/launch_vr_teleop.sh --fake
```

双臂（默认）：

```bash
./openarm_ws/scripts/teleop/launch_vr_teleop.sh --fake --bimanual
```

单臂仿真：

```bash
./openarm_ws/scripts/teleop/launch_vr_teleop.sh --fake --unimanual
```

**说明**：`--fake` 下不连接真实 CAN 电机，用于在没有实物或调试 VR 链路时使用。

---

## 控制实物机械臂

必须显式指定 **`--real`**（否则会按 `--fake` 启动）：

双臂实物：

```bash
./openarm_ws/scripts/teleop/launch_vr_teleop.sh --real
```

与下面等价：

```bash
./openarm_ws/scripts/teleop/launch_vr_teleop.sh --real --bimanual
```

单臂实物：

```bash
./openarm_ws/scripts/teleop/launch_vr_teleop.sh --real --unimanual
```

无需交互确认时（例如自动化脚本内调用）：

```bash
./openarm_ws/scripts/teleop/launch_vr_teleop.sh --real --skip-check
```

**说明**：修改过 `openarm_bringup`、`openarm_description`、`openarm_xr_bridge`、`openarm_vr_ik_teleop` 等包后，需先在工作空间内 `colcon build` 再启动，否则可能仍加载旧的 install 产物。

---

## 实物机械臂 CAN 通道对应关系

本仓库双臂 bringup 的**默认** SocketCAN 与机械臂对应关系为：

| 机械臂 | CAN 接口 | 线缆标识（现场） |
|--------|----------|------------------|
| 右臂   | **can0** | 套**绿色**热缩管 |
| 左臂   | **can1** | — |

请在上电、使能前确认 **套绿色热缩管的线缆接 can0（右臂）**，左臂接 **can1**，与系统内 `ip link` / `slcand` 等配置一致，避免左右接反。

---

## 参数小结

| 参数 | 含义 |
|------|------|
| `--fake` | 假硬件（默认） |
| `--real` | 真硬件 |
| `--bimanual` | 双臂（默认） |
| `--unimanual` | 单臂 |
| `--skip-check` | 跳过启动前「按 Enter 继续」的交互提示 |

---

## 相关文档

- `vr_deployment_guide.md` — 环境迁移、依赖与分终端启动方式  
- `docs/vr_to_ros_pipeline_verification.md` — VR 到 ROS 链路验证  
