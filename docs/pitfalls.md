# Pitfalls

## 2026-10-11：输出枚举存在不等于设备能返回数据

- 症状：SDK 枚举包含 `Marker3D`、`Marker3DFlow`、`Marker2DFlip`，但 G1-WS 实际读取抛 `KeyError`；`ForceResultant` 等其他输出可正常返回。
- 根因：枚举描述 SDK 的能力集合，不保证当前设备 / 标定 / 处理流水线生成每一种字段。
- 修法与教训：启动时逐项向真实设备探测并记录支持情况，锁定可用集合后同一次调用读取；不可用字段在窗口明确标记，不造零值或反复尝试。验收对每个可用输出逐项断言，不能以任意一个输出成功代替所有输出成功。
- Ref：[实时展示指南](xense_live_view.md)、[入口](../tools/xense_live_view.py)。

## 2026-10-11：关闭最后一个 OpenCV Qt 窗口后可见性查询抛异常

- 症状：实时显示正常，关闭窗口后 `getWindowProperty` 报 `NULL guiReceiver`。
- 根因：Qt 在最后一个窗口关闭后销毁 GUI receiver，可见性查询会抛异常而非返回不可见。
- 修法与教训：在窗口可见性查询处处理 `cv2.error`，结束循环并释放传感器；不要让正常关闭变成采集故障。
- Ref：[入口](../tools/xense_live_view.py)。

## 2026-10-11：SDK 可采集不代表自带查看器可显示

- 症状：Xense SDK 2.2.1 导入、USB 扫描和 GPU 深度采集成功，`ExampleView.show()` 却报 `RuntimeError: GLFW window create failed`。
- 定位：失败在 SDK 查看器的 `App.create_window` 图形边界；同一桌面上 OpenCV 窗口创建和渲染独立通过。具体 GLFW / OpenGL 底层故障未进一步诊断，不把它归因于传感器或 CUDA。
- 修法与教训：本次实时图像需求使用已验证的 OpenCV 显示；验收既读取传感器返回数据，也检查桌面窗口 `IsViewable` 并读取窗口像素。不能用“采集日志成功”替代“窗口实际显示”。Wayland 下根窗口截图可能报 `X get_image failed`，直接读取已显示的 XWayland 窗口可验证像素。
- Ref：[实时展示指南](xense_live_view.md)、[入口](../tools/xense_live_view.py)。
