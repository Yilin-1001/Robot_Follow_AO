# 三层隔离重构计划（core / sim / real）

> 状态：**架构已确认，待执行** | 日期：2026-06-11
>
> 本文档是三层架构重构计划。目标是在不依赖 ALX-G100 和 Unitree G1 的情况下先完成离线仿真链路，并为后续真实硬件测试建立清晰隔离边界。

## 1. 重构目标

当前项目中，协议解析、串口读取、滤波、安全判断和跟随控制器平铺在 `src/robot_follow/` 下。

本次重构目标是拆成三层：

```text
core = 硬件无关算法层
sim  = 离线仿真层
real = 真实硬件接入层
```

核心原则：

```text
输入源可以替换。
输出桥可以替换。
中间 core pipeline 必须复用同一套。
```

对应验证链路为：

```text
离线仿真:
SimDataSource / CsvReplaySource
-> core pipeline
-> MockMotionBridge

真实 ALX dry-run:
ALXSerialDataSource
-> core pipeline
-> MockMotionBridge

G1 输出接口验证:
手动/固定小速度命令
-> G1MotionBridge

真实低速闭环:
ALXSerialDataSource
-> core pipeline
-> G1MotionBridge
```

完整验证流程见 `docs/alx_g1_verification_chain.md`。

## 2. 当前状态

当前已有文件：

```text
src/robot_follow/
  __init__.py
  alx_g100_protocol.py
  serial_reader.py
  filtering.py
  safety.py
  follow_controller.py

tests/
  test_alx_g100_protocol.py
  test_follow_controller.py
  test_safety.py

docs/
  g1_alx_g100_follow_mvp.md
  jetson_backpack_test_guide.md
  alx_g1_verification_chain.md
```

关键问题：

```text
main.py 不存在，但 pyproject.toml 已声明 robot-follow = robot_follow.main:main。
filtering.py / safety.py / follow_controller.py 属于硬件无关逻辑，应归入 core。
SafetyManager 当前直接使用 time.monotonic()，不利于确定性仿真测试。
当前测试只有单模块测试，还没有完整 pipeline 验证。
```

## 3. 目标目录结构

```text
src/robot_follow/
  __init__.py
  main.py

  core/
    __init__.py
    types.py
    filtering.py
    safety.py
    follow_controller.py
    pipeline.py

  sim/
    __init__.py
    scenarios.py
    data_source.py
    mock_motion_bridge.py
    runner.py

  real/
    __init__.py
    alx_data_source.py
    g1_motion_bridge.py
    alx_dry_run_runner.py
    g1_check_runner.py
    runner.py

  alx_g100_protocol.py
  serial_reader.py

  filtering.py
  safety.py
  follow_controller.py

tests/
  core/
    test_follow_controller.py
    test_safety.py
    test_filtering.py
    test_pipeline.py

  sim/
    test_scenarios.py
    test_sim_data_source.py
    test_csv_replay_source.py
    test_mock_motion_bridge.py
    test_sim_runner.py

  real/
    test_alx_g100_protocol.py
    test_serial_reader.py
    test_alx_data_source.py
    test_g1_motion_bridge_import.py

  hardware/
    test_real_alx_serial.py
    test_real_g1_motion_bridge.py

docs/
  g1_alx_g100_follow_mvp.md
  jetson_backpack_test_guide.md
  alx_g1_verification_chain.md
  offline_simulation_architecture.md
```

顶层兼容文件保留：

```text
src/robot_follow/filtering.py
src/robot_follow/safety.py
src/robot_follow/follow_controller.py
```

它们只做 re-export，保证旧导入路径继续可用。

## 4. core 层设计

`core/` 只放硬件无关逻辑。

```text
core/types.py
  TargetSample
  MotionCommand

core/filtering.py
  TargetFilter
  FilteredTarget

core/safety.py
  SafetyConfig
  SafetyInput
  SafetyVerdict
  SafetyManager

core/follow_controller.py
  FollowConfig
  FollowOutput
  FollowController

core/pipeline.py
  FollowPipeline
  PipelineStepResult
```

`core/types.py` 建议只放基础类型，避免循环依赖：

```python
@dataclass(frozen=True)
class TargetSample:
    timestamp: float
    distance_m: float
    azimuth_deg: float
    valid: bool = True

@dataclass(frozen=True)
class MotionCommand:
    kind: Literal["move", "stop"]
    vx: float
    vy: float
    omega: float
    reason: str
```

`PipelineStepResult` 放在 `core/pipeline.py`，因为它会引用 `FilteredTarget`、`SafetyVerdict` 和 `FollowOutput`。

pipeline 顺序必须是：

```text
TargetSample
-> if sample.valid is False: stop，不更新 filter
-> TargetFilter.update()
-> SafetyManager.check()
-> if unsafe: stop
-> FollowController.compute()
-> MotionCommand(kind="move")
```

注意：

```text
无效数据不能先进入 TargetFilter。
stop 和 holding 必须区分，不能只靠 vx=0 判断。
MotionCommand.kind 必须明确区分 move / stop。
```

`SafetyManager` 需要支持注入 clock：

```python
SafetyManager(config=None, clock=time.monotonic)
```

这样 pipeline 测试和 dropout 测试可以不用 sleep。

## 5. sim 层设计

`sim/` 只负责离线仿真。

```text
sim/scenarios.py
  定义内置场景:
    straight_approach
    angle_alignment
    too_close
    jump
    dropout

sim/data_source.py
  BuiltinScenarioSource
  CsvReplaySource

sim/mock_motion_bridge.py
  MockMotionBridge

sim/runner.py
  run_simulation()
```

场景要求：

```text
straight_approach:
  距离 2.5m -> 1.3m，角度 0deg

angle_alignment:
  距离约 2.0m，角度 30deg -> 0deg

too_close:
  距离进入 min_safe_distance 以下

jump:
  距离或角度突然大幅跳变

dropout:
  不能只是停止迭代。
  必须使用 invalid sample，或 runner 在无新数据时继续 tick，确保能触发 data_timeout。
```

`MockMotionBridge` 只记录命令：

```text
move(command)
stop(reason)
history
```

它不能 import Unitree SDK，不能打开串口，不能控制真实机器人。

## 6. real 层设计

`real/` 只放真实硬件相关适配。

```text
real/alx_data_source.py
  ALXSerialDataSource
  将 PositionSample 转为 TargetSample

real/g1_motion_bridge.py
  G1MotionBridge
  当前阶段只做占位
  不在模块级 import unitree_sdk2_python

real/alx_dry_run_runner.py
  真实 ALX 输入
  输出到 MockMotionBridge
  G1 不运动

real/g1_check_runner.py
  单独验证 G1 Init / StopMove / 小速度 Move
  不接 ALX 闭环

real/runner.py
  真实闭环 runner
  必须显式 enable_motion 和确认参数
```

真实运动启动必须强保护：

```bash
robot-follow real-run \
  --port /dev/ttyUSB0 \
  --enable-motion \
  --i-understand-this-moves-robot
```

没有确认参数时，不允许调用 G1 `Move()`。

当前阶段不实现真实 G1 控制，只保留接口边界和安全占位。

## 7. CLI 设计

```text
robot-follow simulate
robot-follow simulate --scenario angle_alignment
robot-follow simulate --csv path/to/samples.csv

robot-follow alx-dry-run --port /dev/ttyUSB0
robot-follow g1-check --stop-only
robot-follow real-run --port /dev/ttyUSB0 --enable-motion --i-understand-this-moves-robot
```

当前阶段必须至少实现：

```text
simulate
simulate --scenario
simulate --csv
```

`alx-dry-run`、`g1-check`、`real-run` 可以先占位，但不能误触发真实运动。

## 8. 测试结构

默认测试：

```bash
pytest -q
```

必须只验证不依赖硬件的内容。

```text
tests/core/
  验证过滤、安全、控制器、pipeline

tests/sim/
  验证内置场景、CSV 回放、MockMotionBridge、simulate runner

tests/real/
  只允许做无硬件测试:
    协议解析
    mock 串口 reader
    ALX sample 转 TargetSample
    import g1_motion_bridge 不需要 SDK

tests/hardware/
  真实硬件测试
  默认 skip 或使用 pytest marker 隔离
```

建议硬件测试使用 marker：

```text
@pytest.mark.hardware
```

默认不跑硬件测试。真实硬件测试单独运行：

```bash
pytest -q tests/hardware
```

或：

```bash
pytest -q -m hardware
```

## 9. 分阶段执行计划

### Step 1: 基线验证

```bash
pytest -q
```

确认现有测试通过。

### Step 2: 创建 core 层

```text
移动 filtering.py -> core/filtering.py
移动 safety.py -> core/safety.py
移动 follow_controller.py -> core/follow_controller.py
新增 core/types.py
新增 core/pipeline.py
顶层旧文件改为 wrapper
```

### Step 3: 完成 core pipeline 测试

覆盖：

```text
正常远距离 -> move
大角度 -> move with omega, vx 为 0 或降低
too_close -> stop
distance_jump -> stop
angle_jump -> stop
invalid sample -> stop，且不更新 filter
data_timeout -> stop
```

### Step 4: 创建 sim 层

```text
新增内置场景
新增 CSV 回放
新增 MockMotionBridge
新增 run_simulation()
新增 simulate CLI
```

### Step 5: 完成离线仿真测试

覆盖：

```text
straight_approach
angle_alignment
too_close
jump
dropout
CSV replay
```

### Step 6: 创建 real 层占位

```text
新增 ALXSerialDataSource
新增 G1MotionBridge 占位
新增 alx-dry-run runner
新增 g1-check runner
新增 real-run runner 占位
```

当前阶段不打开真实串口，不 import Unitree SDK，不控制 G1。

### Step 7: 更新文档

保留并引用：

```text
docs/alx_g1_verification_chain.md
```

新增：

```text
docs/offline_simulation_architecture.md
```

说明：

```text
core/sim/real 职责
simulate 使用方式
CSV 格式
真实硬件测试为什么隔离
```

## 10. 验收标准

```bash
python3 -m pip install -e ".[dev]"
pytest -q
robot-follow simulate
robot-follow simulate --scenario angle_alignment
robot-follow simulate --csv path/to/samples.csv
```

通过标准：

```text
普通测试不需要 ALX-G100。
普通测试不需要 Unitree G1。
普通测试不需要 unitree_sdk2_python。
simulate 不打开串口。
simulate 不控制真实机器人。
MockMotionBridge 能记录 move / stop。
危险场景必须产生 stop。
正常场景必须产生合理 vx / omega。
```

## 11. 明确不做的事

```text
不安装 unitree_sdk2_python。
不在模块级 import unitree_sdk2_python。
不打开 /dev/ttyUSB*。
不调用真实 G1 Move / StopMove。
不改 STM32F1-Usart 或其他 vendor 代码。
不删除已有逻辑。
不覆盖未提交改动。
```

## 12. 与验证链路的对应关系

```text
阶段 1: pytest 单元测试
对应:
  tests/core
  tests/real/test_alx_g100_protocol.py

阶段 2: 离线 simulate
对应:
  sim source
  core pipeline
  MockMotionBridge

阶段 3: 真实 ALX dry-run
对应:
  real/ALXSerialDataSource
  core pipeline
  MockMotionBridge

阶段 4: G1 输出接口验证
对应:
  real/G1MotionBridge
  g1_check_runner

阶段 5: 真实低速闭环
对应:
  real/ALXSerialDataSource
  core pipeline
  real/G1MotionBridge
```

最终架构必须保证：

```text
仿真和实测隔离。
真实输入和真实输出可以分阶段替换。
核心算法只有一套，不复制。
任何真实运动都需要显式确认。
```
