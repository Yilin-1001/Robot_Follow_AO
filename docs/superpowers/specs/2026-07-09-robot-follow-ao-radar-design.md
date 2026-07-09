# Robot Follow AO Radar Design

> 状态：初步设计已确认  
> 日期：2026-07-09  
> 目标环境：`robot_followao_radar`  
> 选定方案：方案 2，AO follow 主进程独占同一个 USB 串口，按需管理 Livox / FAST-LIO

## 1. 背景

当前仓库是 `Robot_Follow` 的副本，用于重构失败版本
`/home/yilin/Code/Robot_FollowAO_Radar`。旧版本 `g1_mid360_ao`
同时包含 Livox / FAST-LIO、AO 避障、Web viewer、手环 supervisor、
Robot Follow 接入和 G1 SDK 输出，运行链路过长，延迟来源难以定位。

新版本保留旧 AO 中有价值的 radar / avoidance 核心逻辑，但不继承旧
supervisor 抢占串口和大范围编排的结构。由于手环信号和 ALX 位置信息来自同一个
USB 串口，新架构必须保证同一个串口只有一个读取者。

## 2. 目标

- 使用 `robot_followao_radar` Python 环境运行。
- 直接接入 Livox 和 FAST-LIO，使用 `/cloud_registered_body` 点云做 AO 避障。
- 手环 ON 后直接跑 `ALX 跟随命令 + 雷达 AO 避障 + G1 输出`。
- 手环 OFF 后立即停止 G1 运动，并在 10 秒无有效运动需求后停止 FAST-LIO 和 Livox，释放雷达。
- 保持当前 Robot Follow 的 `MotionCommand` 语义，不把历史 MotionCommand 排队处理。
- 直接复制旧版 AO 的 radar / avoidance 核心模块到当前项目，避免运行时依赖失败版本目录。
- 为每帧记录足够的 age / timing 字段，便于定位延迟。

## 3. 非目标

- 第一版不重建旧 Web viewer。
- 第一版不让 supervisor 独占手环串口。
- 第一版不通过 socket / pipe 转发 ALX 位置信息。
- 第一版不把 Robot Follow core 改成依赖 Livox / FAST-LIO。
- 第一版不改变现有 `robot_follow.core.types.MotionCommand` 的字段结构。

## 4. 核心架构

选定架构是单主运行链路：

```text
USB serial
  -> AO follow main process
      -> wristband ON/OFF parser
      -> ALX position parser
      -> Robot Follow pipeline
      -> RadarServiceManager
      -> FAST-LIO latest point cloud
      -> AO radar / avoidance filter
      -> G1MotionBridge
```

同一个 USB 串口只由 AO follow 主进程读取。手环状态不再由独立 supervisor 读取，也不需要转发给 AO follow。

### 4.1 主数据流

```text
ALX position
  -> TargetSample
  -> Robot Follow TargetFilter / SafetyManager / FollowController
  -> raw MotionCommand
  -> CommandEnvelope

FAST-LIO /cloud_registered_body
  -> latest PointCloudFrame
  -> transform / ROI / self filter
  -> occupancy zones
  -> ObstacleReport

CommandEnvelope + ObstacleReport
  -> old AO stateful planner / limiter
  -> safe MotionCommand
  -> G1MotionBridge
```

### 4.2 手环 ON 时序

```text
1. AO follow 主进程持续读取同一个 USB 串口。
2. 读到 wristband ON。
3. ALX 数据有效时，Robot Follow pipeline 生成 raw MotionCommand。
4. 输出前调用 RadarServiceManager.ensure_running()。
5. 如果 Livox / FAST-LIO 未运行：
   - 启动 Livox ROS2 driver。
   - 启动 FAST-LIO。
   - 等待 /cloud_registered_body ready。
6. 在 FAST-LIO 点云 ready 且未过期之前，G1 必须保持 stop。
7. 点云 ready 后，raw MotionCommand 进入 AO 判断层。
8. AO 输出 safe MotionCommand 到 G1。
```

### 4.3 手环 OFF / 串口失效时序

```text
1. 读到 wristband OFF，或手环心跳超时，或串口/ALX 长期无效。
2. 立即覆盖 latest raw command 为 stop。
3. G1 立即收到 stop(reason="wristband_off" 或对应错误原因)。
4. 不再放行 move MotionCommand。
5. RadarServiceManager 开始 idle 计时。
6. 10 秒内没有新的有效运动需求时：
   - 停止 FAST-LIO。
   - 停止 Livox ROS2 driver。
   - 释放雷达。
7. AO follow 主进程继续读取 USB 串口，等待下一次 wristband ON。
```

## 5. MotionCommand 兼容性

当前 Robot Follow 的 `MotionCommand` 保持不变：

```python
@dataclass(frozen=True)
class MotionCommand:
    kind: Literal["move", "stop"]
    vx: float
    vy: float
    omega: float
    reason: str
```

实时性元数据放在外层 envelope 中，不修改 core 类型：

```text
CommandEnvelope
  command: MotionCommand
  generated_at: float
  source: "robot_follow"
```

Robot Follow 现有逻辑已经是 latest snapshot + per-frame dispatch：

- `ALXLiveSource` 后台读取串口，只保存最新 ALX snapshot。
- 主循环每帧读取 `source.snapshot()`。
- `FollowPipeline.process()` 每次处理一个 `TargetSample`。
- pipeline 输出一个 `MotionCommand` 后立即 dispatch。
- 当前实现没有 MotionCommand 队列。

新 AO 链路必须保持这个语义。

## 6. 延迟帧与积压处理

设计硬约束：

- MotionCommand 不进入无界队列。
- raw command 只保存最新一条，新命令覆盖旧命令。
- point cloud 只保存最新一帧，新点云覆盖旧点云。
- AO tick 不消费历史 command，不追赶延迟帧。
- 任何过期输入都转成 stop。

延迟处理规则：

```text
ALX 位置帧过期:
  FollowPipeline 输出 stop(reason="data_timeout")

MotionCommand 过期:
  AO/output 层输出 stop(reason="command_timeout")

FAST-LIO 点云过期:
  AO/output 层输出 stop(reason="radar_timeout")

Livox / FAST-LIO 未 ready:
  AO/output 层输出 stop(reason="radar_not_ready")

手环 OFF:
  立即覆盖 latest command 为 stop(reason="wristband_off")

G1 dispatch 过慢或 SDK timeout:
  G1 bridge watchdog 输出 stop / fault
```

每帧必须记录：

- `alx_age_ms`
- `command_age_ms`
- `cloud_age_ms`
- `loop_dt_ms`
- `ao_process_ms`
- `g1_dispatch_ms`
- `raw_command_kind`
- `raw_command_reason`
- `safe_command_kind`
- `safe_command_reason`
- `radar_state`
- `wristband_state`

## 7. RadarServiceManager

`RadarServiceManager` 只负责 Livox 和 FAST-LIO 生命周期，不读取手环，不解析 ALX，不做 AO 决策。

职责：

```text
ensure_running()
  确保 Livox 和 FAST-LIO 进程已启动。

cloud_ready()
  检查 /cloud_registered_body 是否已有发布者或最新点云。

mark_motion_demand(now)
  记录最近一次有效运动需求时间。

tick_idle(now)
  如果超过 idle_timeout_s，则停止 FAST-LIO 和 Livox。

stop()
  按顺序停止 FAST-LIO 和 Livox。
```

确认参数：

```text
idle_timeout_s = 10.0
```

停止顺序：

```text
1. 先确保 G1 已收到 stop。
2. 停 FAST-LIO。
3. 停 Livox ROS2 driver。
```

## 8. AO 避障策略

第一版直接复制旧版 `g1_mid360_ao` 的 radar / avoidance 核心逻辑到当前项目，例如：

```text
src/robot_follow/ao/
  core/
    types.py
  radar/
    transform.py
    preprocessing.py
    occupancy.py
    pipeline.py
  avoidance/
    zones.py
    limiter.py
    planner.py
    decision.py
```

保留旧版 stateful planner 行为：

- `avoid_left`
- `avoid_right`
- `return_forward`
- slowdown
- stop
- limiter 限速

但不复制旧版 Web viewer 和旧 supervisor 的串口控制逻辑。

AO 是 G1 输出前的最后安全过滤层：

```text
raw MotionCommand + ObstacleReport
  -> AO planner / limiter
  -> safe MotionCommand
  -> G1MotionBridge
```

如果 radar invalid、radar timeout、cloud not ready，AO 必须输出 stop。

## 9. 真实 G1 输出

第一阶段默认目标是真实 G1 输出，不是 Mock。

真实输出仍必须保留保护：

- 配置中必须允许真实输出。
- CLI 必须显式确认真实运动。
- G1 bridge 必须限速。
- G1 bridge 必须有 watchdog / timeout stop。
- AO runtime 关闭时必须先下发 stop。

在 Livox / FAST-LIO 未 ready 或点云过期时，真实 G1 输出必须保持 stop。

## 10. 模块边界

建议新增模块边界：

```text
src/robot_follow/ao/
  live/
    runtime.py
      AO follow 主运行入口。
    fast_lio_source.py
      ROS2 latest point cloud source，queue size = 1。
    radar_service_manager.py
      Livox / FAST-LIO 子进程生命周期管理。
    command_envelope.py
      MotionCommand 外层实时性元数据。

  serial/
    combined_source.py
      同一个 USB 串口读取者，解析 wristband + ALX。

  radar/
    ...

  avoidance/
    ...
```

现有 Robot Follow 模块继续承担：

- ALX protocol parsing where reusable。
- `TargetSample`
- `MotionCommand`
- `FollowPipeline`
- `G1MotionBridge`

## 11. 错误处理

### 11.1 串口错误

串口错误或长期无数据时：

```text
latest command = stop(reason="serial_error" 或 "data_timeout")
G1 立即 stop
RadarServiceManager 进入 idle 计时
```

### 11.2 雷达启动失败

Livox 或 FAST-LIO 启动失败时：

```text
safe command = stop(reason="radar_start_failed")
不放行 move
记录 radar_state 和错误原因
```

### 11.3 点云超时

点云超过配置阈值：

```text
safe command = stop(reason="radar_timeout")
不消费旧点云
```

### 11.4 G1 输出异常

G1 dispatch 异常或 SDK timeout：

```text
safe command = stop(reason="sdk_error" 或 "sdk_timeout")
记录 g1_dispatch_ms
必要时退出，让外部服务重启
```

## 12. 验收标准

第一版设计验收标准：

1. 同一个 USB 串口只有 AO follow 主进程打开。
2. 手环 ON 后，系统可以读取 ALX 位置信息并生成 raw MotionCommand。
3. 手环 ON 后，如果 Livox / FAST-LIO 未运行，系统会按需启动它们。
4. `/cloud_registered_body` 未 ready 前，G1 必须保持 stop。
5. 点云 ready 后，raw MotionCommand 进入 AO planner，输出 safe MotionCommand。
6. 手环 OFF 后，G1 立即收到 stop。
7. 手环 OFF 后，10 秒无有效运动需求时 FAST-LIO 和 Livox 停止。
8. MotionCommand 不排队，过期 command 不补处理。
9. 点云不排队，过期点云不补处理。
10. 日志能定位 ALX、command、cloud、AO、G1 dispatch 的延迟来源。

## 13. 待实现前必须确认的接口细节

以下属于实现计划阶段需要细化的内容，不改变本设计方向：

- 同一个 USB 串口中手环帧和 ALX 帧的实际解析规则。
- Livox ROS2 driver 启动命令。
- FAST-LIO 启动命令和 config 路径。
- `/cloud_registered_body` 的 frame_id 和坐标约定。
- G1 网卡名、速度上限和 yaw sign。
- 真实输出 CLI 确认参数命名。

