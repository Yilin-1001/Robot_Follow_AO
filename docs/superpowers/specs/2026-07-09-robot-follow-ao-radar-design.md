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
- 解析到新鲜 ALX 位置帧后视为手环已打开，并直接跑 `ALX 跟随命令 + 雷达 AO 避障 + G1 输出`。
- 解析到显式 OFF 帧或位置帧超时后停止 G1 运动，并在 5 秒无新鲜 position frame 后停止 FAST-LIO 和 Livox，释放雷达。
- 保持当前 Robot Follow 的 `MotionCommand` 语义，不把历史 MotionCommand 排队处理。
- 直接复制旧版 AO 的 radar / avoidance 核心模块到当前项目，避免运行时依赖失败版本目录。
- 为每帧记录足够的 age / timing 字段，便于定位延迟。
- 通过 YAML 配置文件修改现场参数，不为了调参修改 `src/`。

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
      -> ALX position parser
      -> wristband gate from ALX position freshness / explicit OFF frame
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
2. 解析到新鲜 ALX position frame，视为 wristband enabled。
3. ALX 数据有效时，Robot Follow pipeline 生成 raw MotionCommand。
4. 新鲜 ALX position frame 同时作为雷达服务 ON 判定，输出前调用 RadarServiceManager.ensure_running()。
5. 如果 Livox / FAST-LIO 未运行：
   - 启动 Livox ROS2 driver。
   - 启动 FAST-LIO。
   - 等待 /cloud_registered_body ready。
6. 在 FAST-LIO 点云 ready 且未过期之前，G1 必须保持 stop。
7. 点云 ready 后，raw MotionCommand 进入 AO 判断层。
8. AO 输出 safe MotionCommand 到 G1。
```

### 4.3 手环 OFF / 位置帧超时 / 串口失效时序

```text
1. 读到显式 wristband OFF 帧，或超过 stale_timeout_s 未解析到新鲜 ALX position frame，或串口长期无效。
2. 立即覆盖 latest raw command 为 stop。
3. G1 立即收到 stop(reason="wristband_off" / "data_timeout" / "serial_error")。
4. 不再放行 move MotionCommand。
5. RadarServiceManager 开始 idle 计时；idle 依据是“没有新鲜 ALX position frame”，不是 MotionCommand 队列。
6. 连续 5 秒没有新鲜 ALX position frame 时：
   - 停止 FAST-LIO。
   - 停止 Livox ROS2 driver。
   - 释放雷达。
7. AO follow 主进程继续读取 USB 串口，等待下一次新鲜 ALX position frame。
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

AO planner 必须适配当前 Robot Follow 的 `MotionCommand + CommandEnvelope`。旧版
`g1_mid360_ao` 的 planner 可以复制迁移，但需要做接口适配：

- 不修改当前 `robot_follow.core.types.MotionCommand`。
- 不要求 Robot Follow `MotionCommand` 增加 `timestamp` 或 `source` 字段。
- AO planner 需要时间戳时，从 `CommandEnvelope.generated_at` 读取。
- AO planner 需要 command source 时，从 `CommandEnvelope.source` 或决策上下文读取。
- 避障核心规则保持旧版逻辑，不因为类型适配改变避障行为。

不要长期保留两套可混用的 `MotionCommand` 类型。AO 输出给 G1 的最终命令也应是当前 Robot Follow 的 `MotionCommand`。

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
- RadarServiceManager 启动 Livox / FAST-LIO 期间不缓存 move command。
- radar ready 后第一帧必须重新读取 latest raw command，并重新通过 command age 检查。

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
  如果解析到显式 OFF 帧，立即覆盖 latest command 为 stop(reason="wristband_off")

ALX 位置帧停止刷新:
  超过 stale_timeout_s 后覆盖 latest command 为 stop(reason="data_timeout")

G1 dispatch 过慢或 SDK timeout:
  G1 bridge watchdog 输出 stop / fault
```

safe command 输出前必须满足输入年龄预算：

```text
command_age_s <= max_command_age_s
cloud_age_s <= max_cloud_age_s
```

AO 计算耗时在计算完成后检查。如果 `ao_process_ms > max_ao_process_ms`，本次 AO 结果必须丢弃，并输出 `stop(reason="ao_timeout")`，不能把超时计算得到的历史 safe command 发给 G1。

第一版建议预算：

```text
ao_output_loop_hz = 20.0
max_command_age_s = 0.2
max_cloud_age_s = 0.5
max_ao_process_ms = 20.0
max_g1_dispatch_ms = 30.0
warn_loop_dt_ms = 75.0
hard_loop_dt_ms = 150.0
```

超时降级规则：

```text
AO 处理超过 max_ao_process_ms:
  safe command = stop(reason="ao_timeout")

G1 dispatch 超过 max_g1_dispatch_ms:
  记录 g1_dispatch_ms
  连续超时后进入 stop / fault

loop_dt 超过 hard_loop_dt_ms:
  下一帧不得追赶历史输入
  只能读取最新 command / 最新点云，或输出 stop
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
- `wristband_gate_state`

## 7. RadarServiceManager

`RadarServiceManager` 只负责 Livox 和 FAST-LIO 生命周期，不读取手环，不解析 ALX，不做 AO 决策。

雷达服务 ON/OFF 判定标准：

```text
ON:
  成功解析到新鲜 ALX position frame。

保持 ON:
  持续在 alx.stale_timeout_s 内解析到新鲜 ALX position frame。

立即停止 G1:
  解析到显式 OFF 帧，或超过 alx.stale_timeout_s 没有新鲜 ALX position frame。

雷达服务 OFF:
  连续 5 秒没有新鲜 ALX position frame 后，停止 FAST-LIO 和 Livox。
```

该判定沿用当前 Robot Follow 实测正确的串口解析方式，不新增第二套 wristband 解析规则。

职责：

```text
ensure_running()
  确保 Livox 和 FAST-LIO 进程已启动。

cloud_ready()
  检查 /cloud_registered_body 是否已有发布者或最新点云。

mark_position_frame_seen(now)
  记录最近一次解析到新鲜 ALX position frame 的时间。

tick_idle(now)
  如果距离最近一次新鲜 ALX position frame 超过 idle_timeout_s，则停止 FAST-LIO 和 Livox。

stop()
  按顺序停止 FAST-LIO 和 Livox。
```

确认参数：

```text
idle_timeout_s = 5.0
```

停止顺序：

```text
1. AO runtime 先向 G1 dispatch stop。
2. AO runtime 调用 RadarServiceManager.stop()。
3. RadarServiceManager 停 FAST-LIO。
4. RadarServiceManager 停 Livox ROS2 driver。
```

`RadarServiceManager` 本身不直接控制 G1。G1 stop、G1 dispatch、G1 shutdown 由 AO runtime / G1MotionBridge 负责。

## 8. AO 避障策略

第一版复制旧版 `g1_mid360_ao` 的 radar / avoidance 核心逻辑到当前项目，并做类型/接口适配。例如：

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

`ao/core/types.py` 只应保留 AO 需要的雷达/避障类型，例如：

```text
PointCloudFrame
ZoneResult
ObstacleReport
AvoidanceDecision
DecisionKind
Direction
```

不要从旧版长期复制并使用旧 AO 的 `MotionCommand` / `CommandKind` 作为主命令类型。旧 planner 中依赖这些类型的地方应适配到当前 Robot Follow `MotionCommand + CommandEnvelope`。适配只改变接口形状，不改变旧版 AO 的避障状态机和规则。

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

  radar/
    ...

  avoidance/
    ...
```

现有 Robot Follow 模块继续承担：

- ALX protocol parsing where reusable。
- `ALXG100SerialReader`
- `ALXLiveSource`
- `TargetSample`
- `MotionCommand`
- `FollowPipeline`
- `G1MotionBridge`

第一版不新增 `combined_source.py`。同一个 USB 串口继续由现有 `ALXG100SerialReader` / `ALXLiveSource` 路径读取；后续只扩展它们，把当前实测正确的 position frame / 显式 OFF 帧解析结果暴露为结构化 `wristband_gate` 状态。

## 11. 配置策略

第一版主配置文件：

```text
configs/jetson_g1_ao_radar.yaml
```

该文件必须从现有 `configs/jetson_g1_safe.yaml` 扩展而来，保留现有 Robot Follow 配置语义：

```text
alx
g1
safety
follow
filter
run
```

AO / Radar 只追加新 section，不替换现有 section 的含义：

```text
wristband_gate
radar_service
fast_lio
transform
roi
self_filter
zones
avoidance
ao
logging
```

第一版不使用 YAML include。配置文件应是一个完整文件，便于现场复制、审查和修改。

配置原则：

- `src/` 只定义默认值、dataclass/schema、解析和校验逻辑。
- `configs/jetson_g1_ao_radar.yaml` 承担现场参数。
- CLI 参数只做临时覆盖。
- 未知配置 key 必须报错，避免拼写错误静默失效。
- 修改跟随距离、速度上限、G1 网卡、Livox/FAST-LIO 启动命令、点云 topic、ROI、self filter、zone、avoidance、超时预算，不需要改 `src/`。

关键配置形状：

```yaml
alx:
  port: /dev/ttyUSB0
  baudrate: 115200
  hz: 40.0
  stale_timeout_s: 0.5

g1:
  net_iface: eth0
  max_vx: 0.05
  max_vy: 0.15
  max_omega: 0.10
  yaw_sign: 1

safety:
  data_timeout_s: 0.5
  min_safe_distance_m: 0.5

follow:
  target_distance_m: 1.2

filter:
  alpha: 0.35

run:
  hz: 20.0
  command_timeout_s: 0.2
  print_every_s: 0.5

wristband_gate:
  enabled: true
  # 第一版 gate 规则沿用当前 Robot Follow 实测解析：
  # 新鲜 ALX position frame 表示 enabled；显式 OFF 帧优先表示 disabled；
  # 如果没有显式 OFF 帧，则由 alx.stale_timeout_s 触发 data_timeout。
  infer_on_from_position_frame: true
  explicit_off_frame_enabled: true

radar_service:
  idle_timeout_s: 5.0
  cloud_ready_timeout_s: 10.0
  livox_command: []
  fast_lio_command: []

fast_lio:
  pointcloud_topic: /cloud_registered_body
  expected_frame_id: body
  max_cloud_age_s: 0.5

ao:
  max_ao_process_ms: 20.0
  max_g1_dispatch_ms: 30.0
  warn_loop_dt_ms: 75.0
  hard_loop_dt_ms: 150.0
```

## 12. 错误处理

### 12.1 串口错误

串口错误或长期无数据时：

```text
latest command = stop(reason="serial_error" 或 "data_timeout")
G1 立即 stop
RadarServiceManager 进入 idle 计时
```

### 12.2 雷达启动失败

Livox 或 FAST-LIO 启动失败时：

```text
safe command = stop(reason="radar_start_failed")
不放行 move
记录 radar_state 和错误原因
```

### 12.3 点云超时

点云超过配置阈值：

```text
safe command = stop(reason="radar_timeout")
不消费旧点云
```

### 12.4 G1 输出异常

G1 dispatch 异常或 SDK timeout：

```text
safe command = stop(reason="sdk_error" 或 "sdk_timeout")
记录 g1_dispatch_ms
必要时退出，让外部服务重启
```

## 13. 验收标准

第一版设计验收标准：

1. 同一个 USB 串口只有 AO follow 主进程打开。
2. 解析到新鲜 ALX position frame 后，系统可以生成 raw MotionCommand。
3. 有新鲜 ALX position frame 后，如果 Livox / FAST-LIO 未运行，系统会按需启动它们。
4. `/cloud_registered_body` 未 ready 前，G1 必须保持 stop。
5. 点云 ready 后，raw MotionCommand 进入 AO planner，输出 safe MotionCommand。
6. 解析到显式 OFF 帧后，G1 立即收到 stop。
7. 没有新鲜 ALX position frame 超过 `alx.stale_timeout_s` 后，G1 收到 stop(reason="data_timeout")。
8. 显式 OFF 或 data_timeout 后，5 秒无新鲜 ALX position frame 时 FAST-LIO 和 Livox 停止。
9. MotionCommand 不排队，过期 command 不补处理。
10. 点云不排队，过期点云不补处理。
11. 日志能定位 ALX、command、cloud、AO、G1 dispatch 的延迟来源。
12. Livox / FAST-LIO 启动期间不缓存 move command，radar ready 后只处理最新未过期 command。
13. safe command 输出前检查 command age 和 cloud age；AO 处理完成后检查 AO 耗时，任一超预算都必须 stop 或 fault。
14. 存在完整主配置 `configs/jetson_g1_ao_radar.yaml`，并从 `configs/jetson_g1_safe.yaml` 扩展现有 alx/g1/safety/follow/filter/run 参数。
15. 修改 AO/Radar/G1/Follow 现场参数不需要修改 `src/`。

## 14. 待实现前必须确认的接口细节

以下属于实现计划阶段需要细化的内容，不改变本设计方向：

- 当前 Robot Follow 实测解析规则如何暴露为结构化 wristband gate 状态。
- Livox ROS2 driver 启动命令。
- FAST-LIO 启动命令和 config 路径。
- `/cloud_registered_body` 的 frame_id 和坐标约定。
- G1 网卡名、速度上限和 yaw sign。
- 真实输出 CLI 确认参数命名。

## Implementation Plan

Implementation plan: `docs/superpowers/plans/2026-07-09-robot-follow-ao-radar.md`

The implementation keeps Robot Follow core unchanged, exposes only structured wristband gate state from the existing parser, adds AO runtime under `robot_follow.ao`, and keeps the command/point-cloud paths latest-only.
