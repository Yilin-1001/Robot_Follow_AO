# GUI Sim 设计文档

## 1. 目标

`GUI_sim` 是 `robot_follow.core` 跟随 pipeline 的可视化测试台。
它不是机器人控制界面，也不能导入任何真实硬件相关模块。

第一版需要回答一个核心问题：

```text
当鼠标控制的目标持续移动时，core pipeline 是否能输出合理的跟随、
转向、保持、停止命令，并且这些命令是否能让一个模拟机器人点产生
符合预期的运动？
```

GUI 负责提供直观画面，终端日志负责提供逐帧证据，方便调试和测试复核。

## 2. 范围

第一版包含：

- 鼠标位置表示被跟随的人。
- 一个点或小圆表示机器人。
- 机器人有朝向。
- 在机器人前方显示 120 度检测范围。
- 鼠标目标只有在 120 度检测范围内、且没有超过最大检测距离时才有效。
- 每一帧都转换成 `TargetSample`。
- 每一帧都调用 `FollowPipeline.process()`。
- 使用返回的 `MotionCommand` 更新模拟机器人状态。
- GUI 显示人、机器人、朝向、检测范围和基础状态。
- 终端打印结构化逐帧日志和事件日志。
- 键盘支持暂停/继续、重置等基本控制。

第一版不包含：

- 真实 ALX-G100 输入。
- 真实 Unitree G1 运动输出。
- 障碍物或地图。
- 多目标跟踪。
- 录制和回放。
- 复杂物理仿真。
- Web 前端或浏览器界面。

## 3. 包边界

后续开发文件建议放在：

```text
src/robot_follow/sim/GUI_sim/
  DESIGN.md
  __init__.py
  geometry.py
  realtime_state.py
  logger.py
  gui_app.py
```

推荐测试文件：

```text
tests/sim/GUI_sim/
  test_geometry.py
  test_realtime_state.py
  test_logger.py
```

说明：`GUI_sim` 是当前指定的文件夹名称。它可以作为 Python 包导入，
但 Python 包名通常推荐使用小写。如果后续需要对外作为稳定 API，
可以考虑增加一个小写别名，或迁移为 `gui_sim`。

## 4. 架构

运行时循环：

```text
GUI 鼠标位置
-> geometry.mouse_to_target_sample(...)
-> core.FollowPipeline.process(sample)
-> MotionCommand
-> realtime_state.SimRobotState.apply(command, dt)
-> GUI 重绘
-> 终端日志
```

`core` pipeline 是被测试对象。GUI 不能重复实现跟随逻辑。
GUI 只负责：

- 把屏幕坐标转换为距离和方位角；
- 判断目标是否处于模拟传感器检测范围内；
- 可视化当前状态；
- 根据 `MotionCommand` 对模拟机器人位姿做简单积分。

设计决策：第一版采用严格分层方案。GUI 代码禁止根据距离或角度直接生成
`MotionCommand`，所有运动命令都必须来自：

```python
step = pipeline.process(sample)
command = step.command
```

## 5. 坐标模型

使用简单的 2D 俯视世界。

世界坐标：

```text
x_m: 单位为米，向右为正
y_m: 单位为米，向上为正
heading_rad: 机器人在世界坐标中的朝向，单位为弧度
```

屏幕坐标：

```text
x_px: 单位为像素，向右为正
y_px: 单位为像素，向下为正
```

GUI 使用固定比例尺，例如：

```text
pixels_per_meter = 120
```

坐标转换逻辑必须集中在 `geometry.py` 中，保证不打开 GUI 也可以通过单元测试验证。

## 6. 120 度检测范围

模拟传感器检测范围定义为机器人朝向前方的扇形区域：

```text
fov_deg = 120.0
half_fov_deg = 60.0
```

每一帧计算：

```text
relative_angle_deg = 鼠标目标相对机器人朝向的夹角
distance_m = 机器人到鼠标目标的距离
in_fov = abs(relative_angle_deg) <= 60.0
in_range = distance_m <= max_detection_range_m
valid = in_fov and in_range
```

当 `valid` 为 false 时，生成的 `TargetSample` 仍然保留测得的距离和方位角，
用于日志展示，但必须设置：

```python
valid=False
```

预期 core 行为：

- 目标在检测范围内：pipeline 可以输出 `move`。
- 目标离开检测范围：pipeline 应输出 `stop`，原因通常为 `data_invalid`。
- 目标距离过近：pipeline 应输出 `stop`，原因为 `too_close`。
- 目标突然大幅跳变：pipeline 可能输出 `stop`，原因为 `distance_jump`
  或 `angle_jump`，具体取决于滤波器和安全参数。

## 7. 模拟机器人状态

`realtime_state.py` 建议定义一个小型状态对象：

```text
SimRobotState
  x_m
  y_m
  heading_rad
```

对于 `move` 命令：

```text
heading_rad += command.omega * dt
x_m += cos(heading_rad) * command.vx * dt
y_m += sin(heading_rad) * command.vx * dt
```

对于 `stop` 命令：

```text
位姿保持不变
```

第一版可以忽略 `vy`，因为当前 `FollowController` 始终输出 `vy=0.0`。

## 8. 时序与刷新率

真实接入背包的信号基站刷新率为 40 Hz，因此 GUI_sim 默认按 40 Hz
向 `core` pipeline 输入样本。

```text
input_hz = 40.0
fixed_dt_s = 1 / 40 = 0.025 s
```

第一版采用固定步长设计：

```text
每 25 ms:
  采样鼠标位置
  计算 distance / azimuth / valid
  调用 FollowPipeline.process()
  使用 dt=0.025 更新模拟机器人状态
```

GUI 绘制频率可以等于或低于 40 Hz，但 core pipeline 的调用频率默认保持
40 Hz。终端逐帧日志不按 40 Hz 全量打印，而是限频到约 10 Hz；事件日志
仍在状态变化时立即打印。

这样做的目的：

- 模拟输入节奏贴近真实基站；
- 避免不同机器 GUI 刷新耗时不同导致仿真结果漂移；
- 保持终端日志可读。

## 9. GUI 行为

第一版推荐使用 `tkinter.Canvas`。

原因：

- Python 标准库自带；
- 不需要新增运行时依赖；
- 足够绘制点、箭头、扇形检测范围和状态文本；
- 鼠标和键盘事件处理简单。

默认视图：

- 机器人：显示为当前位姿处的小圆。
- 朝向：从机器人中心画一段短箭头。
- 人：显示为鼠标位置处的小圆。
- 检测范围：显示机器人前方 120 度扇形区域。
- 目标连线：从机器人连到人，有效和无效状态使用不同颜色。
- 状态区：显示 distance、azimuth、valid、command kind、reason、vx、omega。

键盘控制：

```text
Space: 暂停/继续模拟
R: 重置机器人位姿和 pipeline 滤波状态
L: 切换逐帧日志详细程度
Q 或 Escape: 退出
```

## 10. 终端日志

GUI 应打印两类日志。

逐帧日志限制到约 10 Hz。注意：这只是日志输出频率，不是 core pipeline
输入频率；core pipeline 仍默认按 40 Hz 调用。

```text
FRAME t=12.35 dt=0.050 target=(2.10,-0.40)m distance=2.14m azimuth=-10.8deg valid=True in_fov=True command=move reason=tracking_forward vx=0.13 omega=-0.002 robot=(0.42,0.01,-1.2deg)
```

事件日志，在状态变化时打印：

```text
EVENT target_left_fov azimuth=68.4deg distance=2.20m
EVENT target_reacquired azimuth=42.0deg distance=2.10m
EVENT command_changed from=move/tracking_forward to=stop/data_invalid
EVENT command_changed from=move/tracking_forward to=stop/too_close
```

日志逻辑应独立于 GUI 绘制逻辑，方便不打开窗口也能进行单元测试。

## 11. 测试策略

单元测试不应打开 GUI 窗口。

`test_geometry.py`：

- 目标在机器人正前方时，`azimuth_deg == 0` 且 `valid=True`。
- 目标在 +60 度边界时有效。
- 目标在 -60 度边界时有效。
- 目标略微超过 +60 度时无效。
- 目标超过最大检测距离时无效。
- 角度归一化能正确处理 -180 / +180 度边界。

`test_realtime_state.py`：

- `stop` 命令不会移动机器人。
- 前进 `move` 命令会沿机器人朝向改变位置。
- 正 `omega` 会改变机器人朝向。
- reset 会恢复初始位姿。
- 使用固定 `dt=0.025` 时，位姿积分结果稳定可复现。

`test_logger.py`：

- 逐帧日志包含目标、命令、原因和机器人位姿。
- 命令原因变化时输出事件日志。
- 连续未变化的帧不会重复刷事件日志。

手动 GUI 测试流程：

1. 启动 GUI。
2. 将鼠标移动到机器人前方 120 度检测范围内。
3. 确认终端日志显示 `valid=True`。
4. 确认机器人会转向并靠近鼠标目标。
5. 将鼠标移动到检测范围外。
6. 确认终端日志显示 `valid=False` 和 `stop/data_invalid`。
7. 将鼠标移动到机器人非常近的位置。
8. 确认终端日志显示 `stop/too_close`。
9. 确认 core pipeline 调用频率为 40 Hz，终端逐帧日志约为 10 Hz。

## 12. 验收标准

第一版满足以下条件即可验收：

- 不需要 ALX-G100 硬件即可运行。
- 不需要 Unitree SDK 即可运行。
- 每一帧都使用 `FollowPipeline.process()`。
- GUI 能显示机器人、目标和 120 度检测范围。
- 目标离开 120 度检测范围时，机器人停止。
- 终端日志足够诊断 core pipeline 的决策过程。
- core pipeline 默认按 40 Hz 调用，固定步长为 `dt=0.025s`。
- 几何计算和机器人状态更新逻辑有单元测试。

## 13. 风险与规避

风险：GUI 逻辑不小心重复实现 core 跟随行为。

规避方案 A：采用严格分层。所有跟随决策都放在 `FollowPipeline` 中。
GUI 只做坐标转换、有效性模拟、显示、日志和位姿积分。GUI 禁止根据距离
或角度直接生成 `MotionCommand`。

风险：120 度检测范围边界行为不明确。

规避方案 A：采用闭区间边界。明确规定 `-60 deg <= azimuth_deg <= +60 deg`
属于检测范围内，并写测试覆盖 `-60`、`0`、`+60`、`60.1`、`-60.1`。

风险：GUI 帧率较高，导致终端日志过多。

规避方案 A：逐帧日志限频输出，默认约 10 Hz；事件日志在有意义的状态变化时
立即输出。core pipeline 调用频率不受日志限频影响，仍默认保持 40 Hz。

风险：现有 `run_simulation()` 是批处理 runner，不适合实时鼠标输入。

规避方案 A：GUI 每一帧直接调用 `FollowPipeline.process()`，不要用
`run_simulation()` 包一层。现有 `run_simulation()` 继续只服务离线 scenario
和 CSV 回放。

风险：`GUI_sim` 文件夹名不符合 Python 小写包名习惯。

规避方案 A：保留该名称，因为这是当前明确指定的目录。若未来导入工具或风格
检查出现问题，再增加小写兼容包或迁移到 `gui_sim`。

风险：GUI 运行环境可能没有显示能力，例如 SSH、Docker 或无桌面的 Jetson。

规避方案 A：核心逻辑必须能脱离 GUI 单元测试。`geometry.py`、
`realtime_state.py` 和 `logger.py` 不依赖窗口环境；`gui_app.py` 作为手动
可视化测试入口。无显示环境下不强制运行 GUI，只运行非 GUI 单元测试。
