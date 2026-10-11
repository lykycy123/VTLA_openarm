# 千觉传感器实时展示

适用于 USB 直连的千觉传感器。当前已在 G1-WS、Xense SDK 2.2.1、`vtla_medibot` 环境验证。

## 启动

在仓库根目录、已登录图形桌面的终端运行。先查询设备序列号，再用查询结果替换 `YOUR_SENSOR_SERIAL`：

```bash
conda activate vtla_medibot
python -c "from xensesdk import Sensor; print(Sensor.scanSerialNumber())"
python tools/xense_live_view.py --serial YOUR_SENSOR_SERIAL
```

所有信息显示在一个仪表盘中：

- 左侧 16 个图形面板：原始图像、校正视触觉、差分与增强差分、深度 / 精细深度 / 粗深度、力分布模长 / 法向力模长、网格形变模长、标记点与位移箭头、光流，以及浮点图像、去标记图像、标记增强和物体增强图像。
- 右上角：`ForceResultant` 和 `Force6DTuned` 的六维数值，以及 `ForceResultant` 三轴力最近 300 帧的曲线。
- 右下角：SDK 每一个输出枚举的形状、当前最小 / 最大值或时间戳；设备不支持的输出明确显示 `unavailable`，不会伪装成零。
- 顶部：序列号、帧数、帧率和固定显示范围。按 `Q`、`Esc` 或关闭窗口退出并释放设备。

深度默认固定映射到 0–0.2 mm，低值显示为深色、高值显示为红色；超过上限会饱和。该默认值比原来的 2 mm 更适合观察微小形变。需要更大的颜色范围时，启动前指定：

```bash
python tools/xense_live_view.py --serial YOUR_SENSOR_SERIAL --depth-range-mm 0.5
```

力分布图显示三维向量的模长，并非只显示某一个方向；精确的正负合力与力矩看右侧数值。官方当前输出文档明确深度单位为 mm，但没有明确力和力矩单位，因此界面保留 SDK 原始值，并标注单位未确认，不能把数值直接当作经校验的 N 或 N·m。`Force6DTuned` 也没有在此重新标定或去除空载偏置。

力分布色标默认 0–0.02 SDK 单位；三轴力曲线默认 ±1 SDK 单位。这些只是显示范围，不改变传感器灵敏度。可以在启动时固定其他范围：

```bash
python tools/xense_live_view.py --serial YOUR_SENSOR_SERIAL \
  --depth-range-mm 0.2 --force-range 0.01 --trace-range 0.1
```

## 数据流与不变量

`SDK 扫描 → 固定序列号创建 Sensor → 启动时逐项探测全部输出 → 锁定可用输出集合 → 每帧一次调用读取全部可用数据 → 图像 / 数值 / 摘要 / 曲线 → OpenCV 单窗口`。

- 序列号在启动时锁定，循环不重新选择相机编号。
- 启动探测逐项记录真实支持状态；正常帧来自同一次 SDK 调用，避免分别读取各面板。每项分别验证数据非空且有限；可选输出异常显示状态，核心 Rectify / Depth 异常则退出，防止以旧画面冒充实时数据。
- 色标在启动时锁定，不逐帧自动归一化，防止空载噪声被渲染成强接触。
- JSONL 默认追加到 `/tmp/vtla_xense_dashboard.jsonl`，记录每项探测结果、固定显示范围、每帧所有输出的形状 / 最小 / 最大值 / 小向量原始值、读取耗时、帧率和退出原因。可通过 `--log /tmp/my_xense.jsonl` 更换位置。临时目录文件不是长期存档；需要留存时指定自己的日志路径。SDK 自身运行信息仍输出到终端。
- 窗口退出或 Python 异常时释放 Sensor；不依赖 SDK 的 GLFW / OpenGL 查看器。

## 本次验收

2026-10-11：G1-WS 实际返回 26 类可用输出，逐项验证连续 20 帧均非空且有限；两种六维力输出均为 6 元向量。`Marker3D`、`Marker3DFlow`、`Marker2DFlip` 在当前设备不可读取，明确标为不可用。桌面窗口状态为 `IsViewable`，读取实际窗口像素确认所有图形、力数值、曲线和输出摘要都显示，帧率约 27 FPS。向已确认的窗口发送真实 Q 按键后，进程退出码为 0，JSONL 记录按键退出，再次启动成功连接设备。

人工按压的物理动作需由使用者完成，本次不宣称已自动验证力单位、接触精度或硬件最小可检测力。
