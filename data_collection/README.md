# RBM OpenArm 数据采集

参考 `ARX_A5_PLAY/data_collection`，面向本仓库话题做 OpenArm + Ranger 示教采集。

## 依赖

```bash
conda activate openarm
pip install h5py opencv-python pyyaml
```

## 启动前

推荐控制菜单快速启动 **13**（采集）/ **14**（回放）。手动时建议：`2 → 5 → 8 → 9`（CAN、VR 臂、底盘 VR、相机），或至少保证以下话题有数据：

- `/camera/camera_{l,h,r}/color/image_rect_raw/compressed`
- `/joint_states`
- TF：`world` → `openarm_left_hand_tcp` / `openarm_right_hand_tcp`
- `/odom`、`/cmd_vel`

## 运行

```bash
cd ~/Downloads/WorkSpace/hao/Medibot_openarm
source source_all.sh
conda activate openarm
python data_collection/data_collection.py
# 或指定配置/输出目录：
# python data_collection/data_collection.py --config data_collection/config.yaml --datasets-dir ~/Downloads/WorkSpace/hao/Medibot_openarm/datasets
```

- **SPACE**：开始 / 停止录制（停止时写入 `datasets/episode_N.hdf5`）
- **ESC**：退出

## 回放

需先启动带 `forward_position_controller` 的机械臂驱动（不要同时开 VR IK）。回放底盘时再开底盘驱动：

```bash
# 示例：真机双臂 forward_position（勿与菜单 5 的 IK 抢控制器）
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  arm_type:=v10 use_fake_hardware:=false \
  robot_controller:=forward_position_controller launch_rviz:=false

source ~/Downloads/WorkSpace/hao/Medibot_openarm/source_all.sh
conda activate openarm
python data_collection/replay.py
# python data_collection/replay.py --replay-hdf5-path datasets/episode_0.hdf5 --no-replay-base
```

- 启动后终端会列出 `datasets/episode_*.hdf5`，输入列表序号或 episode 编号选择；直接 Enter 用最新
- **SPACE**：开始 / 暂停回放  
- **ESC**：退出（默认回 home）  
- 发布：左右臂 `forward_position`、夹爪 Action、可选 `/cmd_vel`  
- 跳过选择：`--latest` 或 `--replay-hdf5-path datasets/episode_0.hdf5`

## HDF5 字段

| 路径 | 说明 |
|------|------|
| `data/demo_0/timestamp` | 采样时间 |
| `data/demo_0/observations/qpos` | 左右臂 7+夹爪+7+夹爪（16） |
| `data/demo_0/observations/qvel` | 同上速度 |
| `data/demo_0/observations/eef` | 左右末端位姿各 7：`xyz + qxqyqzqw`（14） |
| `data/demo_0/observations/odom` | 底盘里程计 13 维 |
| `data/demo_0/observations/cmd_vel` | 底盘速度命令 6 维 |
| `data/demo_0/observations/images/{head,left_wrist,right_wrist}` | JPEG 图像 |
| `data/demo_0/actions*` | 与观测对齐的动作副本 |
