# Unitree G1 + ALX-G100 跟随系统验证链路

## 1. 目标

本文档定义项目从离线代码验证到真实硬件闭环测试的完整验证流程。

验证目标不是直接让机器人运动，而是分阶段确认：

```text
代码模块正确
-> 离线完整链路正确
-> 真实 ALX 输入正确
-> G1 输出接口正确
-> 最后才进入低速真实闭环
```

核心原则：

```text
输入源可以替换。
输出桥可以替换。
中间 core pipeline 必须复用同一套。
```

## 2. 总体链路

系统最终真实链路为：

```text
ALX-G100 手环 / 基站
-> ALX 串口数据源
-> core pipeline
-> G1 motion bridge
-> Unitree G1
```

离线仿真链路为：

```text
模拟数据源 / CSV 回放
-> core pipeline
-> MockMotionBridge
```

半实物验证链路为：

```text
真实 ALX 串口数据源
-> core pipeline
-> MockMotionBridge
```

三条链路的差异只在输入源和输出桥，中间算法链路保持一致。

## 3. core pipeline

`core pipeline` 是仿真和真实运行共同复用的核心链路。

推荐结构：

```text
TargetSample
-> 基础有效性判断
-> TargetFilter
-> SafetyManager
-> FollowController
-> MotionCommand / PipelineStepResult
```

其中：

```text
TargetSample:
  timestamp
  distance_m
  azimuth_deg
  valid

TargetFilter:
  负责距离和角度平滑
  负责计算 distance_jump / angle_jump

SafetyManager:
  负责判断是否必须停止
  例如数据无效、数据超时、距离过近、跳变异常

FollowController:
  负责根据距离和角度计算 vx / vy / omega

MotionCommand:
  表示本周期的运动输出
  应明确区分 move 和 stop
```

注意：

```text
无效数据不应先进入滤波器。
sample.valid=False 时应直接进入 safety stop 逻辑，避免污染 filter 状态。
```

## 4. 阶段 1：单元测试

### 4.1 目标

验证单个模块是否按预期工作。

### 4.2 输入

手写模拟数据。

例如：

```text
模拟 ALX 原始字节
模拟 distance_m / azimuth_deg
模拟 SafetyInput
```

### 4.3 链路

协议解析测试：

```text
模拟 ALX 原始字节
-> alx_g100_protocol.parse_frame()
-> 检查 distance / azimuth / tag_status
```

控制器测试：

```text
模拟 distance_m / azimuth_deg
-> FollowController.compute()
-> 检查 vx / omega / reason
```

安全测试：

```text
模拟 SafetyInput
-> SafetyManager.check()
-> 检查 safe / stop reason
```

### 4.4 运行方式

```bash
pytest -q
```

### 4.5 通过标准

```text
所有不依赖硬件的单元测试通过。
测试过程中不需要 ALX-G100。
测试过程中不需要 Unitree G1。
测试过程中不需要 unitree_sdk2_python。
```

### 4.6 本阶段能证明什么

```text
协议解析单独正确。
控制器单独正确。
安全判断单独正确。
```

### 4.7 本阶段不能证明什么

```text
完整主循环是否能连续运行。
真实 ALX 数据是否稳定。
真实 G1 控制是否可用。
```

## 5. 阶段 2：离线仿真验证

### 5.1 目标

在没有 ALX-G100 和 G1 的情况下，验证完整软件链路。

### 5.2 输入

```text
SimDataSource
CsvReplaySource
```

推荐内置场景：

```text
straight_approach
angle_alignment
too_close
dropout
jump
```

### 5.3 链路

```text
SimDataSource / CsvReplaySource
-> TargetSample
-> core pipeline
-> MockMotionBridge
-> 日志 / 测试断言
```

### 5.4 场景预期

`straight_approach`：

```text
输入:
  距离 2.5m -> 1.3m
  角度 0deg

预期:
  远距离时输出前进命令
  接近目标距离后进入 holding
```

`angle_alignment`：

```text
输入:
  距离约 2.0m
  角度 30deg -> 0deg

预期:
  大角度时优先转向
  角度变小后允许低速前进
```

`too_close`：

```text
输入:
  距离小于 min_safe_distance

预期:
  stop
  reason=too_close
```

`jump`：

```text
输入:
  距离或角度突然大幅跳变

预期:
  stop
  reason=distance_jump 或 angle_jump
```

`dropout`：

```text
输入:
  数据中断、无效样本，或 runner 持续 tick 但没有新数据

预期:
  stop
  reason=data_timeout 或 data_invalid
```

注意：

```text
如果 dropout 场景只是让数据源停止迭代，pipeline 不会继续运行，也就无法触发 timeout。
因此 dropout 场景应使用 invalid sample，或者 runner 在无新数据时继续产生 tick。
```

### 5.5 运行方式

```bash
robot-follow simulate
robot-follow simulate --scenario angle_alignment
robot-follow simulate --csv path/to/samples.csv
```

### 5.6 通过标准

```text
simulate 可以在无硬件环境运行。
MockMotionBridge 记录 move / stop。
危险场景必须输出 stop。
正常场景必须输出合理 vx / omega。
日志包含 raw、filtered、safety、controller、command 信息。
```

### 5.7 本阶段能证明什么

```text
完整软件链路可以连续运行。
滤波、安全、控制器组合后行为合理。
模拟异常能触发 stop。
```

### 5.8 本阶段不能证明什么

```text
真实 ALX 串口是否稳定。
真实距离和角度方向是否符合安装方向。
Unitree G1 SDK 是否可用。
机器人运动是否稳定。
```

## 6. 阶段 3：真实 ALX 输入 dry-run

### 6.1 目标

硬件到位后，先验证真实 ALX 数据输入和控制输出，不让 G1 运动。

### 6.2 输入

```text
ALXSerialDataSource
```

### 6.3 输出

```text
MockMotionBridge
```

### 6.4 链路

```text
ALXSerialDataSource
-> core pipeline
-> MockMotionBridge
```

### 6.5 观察日志

应记录：

```text
raw distance
raw azimuth
filtered distance
filtered azimuth
distance_jump
angle_jump
safety_safe
safety_reason
vx
vy
omega
controller_reason
command kind
```

### 6.6 测试动作

```text
佩戴手环站在机器人正前方。
缓慢远离和靠近。
左右偏移。
短暂遮挡或断开 ALX。
观察距离、角度、控制输出和 stop reason。
```

### 6.7 通过标准

```text
真实 ALX 数据能稳定解析。
距离变化方向符合实际动作。
角度正负方向能被确认。
滤波后数值平滑。
正常移动不会频繁误触发 jump。
过近、断连、异常跳变能触发 stop。
G1 不发生任何运动。
```

### 6.8 本阶段能证明什么

```text
真实 ALX 输入可以驱动 core pipeline。
真实数据下控制输出基本合理。
```

### 6.9 本阶段不能证明什么

```text
G1 控制接口是否可用。
真实闭环运动是否稳定。
```

## 7. 阶段 4：G1 输出接口验证

### 7.1 目标

单独验证 Unitree G1 控制接口，不进入完整跟随闭环。

### 7.2 推荐链路

Stop-only 验证：

```text
手动触发
-> G1MotionBridge
-> StopMove
```

小速度运动验证：

```text
固定小速度命令
-> G1MotionBridge
-> Move(vx, vy, omega)
-> StopMove
```

或者：

```text
ALXSerialDataSource
-> core pipeline
-> G1MotionBridge(stop-only)
```

### 7.3 安全要求

```text
G1 原厂遥控器必须在线。
测试人员必须能随时接管。
测试场地必须空旷。
首次只验证 Init 和 StopMove。
Move 测试必须使用极低速度。
Move 测试必须有持续时间限制。
Move 后必须调用 StopMove。
```

### 7.4 通过标准

```text
unitree_sdk2_python 可以加载。
G1 LocoClient 可以初始化。
StopMove 调用有效。
小速度 Move 可控。
遥控器可以接管。
异常时能立即 stop。
```

### 7.5 本阶段能证明什么

```text
G1 运动输出通道可用。
StopMove 安全路径可用。
```

### 7.6 本阶段不能证明什么

```text
ALX 输入和 G1 输出组成闭环后是否稳定。
```

## 8. 阶段 5：真实低速闭环验证

### 8.1 目标

验证真实 ALX 输入和真实 G1 输出组成低速跟随闭环。

### 8.2 链路

```text
ALXSerialDataSource
-> core pipeline
-> G1MotionBridge
-> Unitree G1
```

### 8.3 启动要求

真实运动命令必须要求显式确认参数，例如：

```bash
robot-follow real-run \
  --port /dev/ttyUSB0 \
  --enable-motion \
  --i-understand-this-moves-robot
```

没有显式确认参数时，只允许 dry-run，不允许调用 G1 Move。

### 8.4 初始参数建议

```text
max_vx: 0.10 ~ 0.20 m/s
max_omega: 较小值
min_safe_distance: 保守设置
data_timeout_s: 保守设置
vy: 0
```

### 8.5 通过标准

```text
机器人能在低速下保持朝向目标。
距离过近时停止。
数据丢失时停止。
角度过大时优先转向。
遥控器可随时接管。
无异常加速、无持续误动作。
```

### 8.6 失败处理

如果出现异常：

```text
立即 StopMove。
使用遥控器接管。
保存日志。
回退到阶段 3 或阶段 4 单独排查。
不要直接在真实闭环中连续试错。
```

## 9. 推荐验证顺序

```text
阶段 1: pytest 单元测试
阶段 2: 离线 simulate
阶段 3: 真实 ALX 输入 + MockMotionBridge
阶段 4: G1MotionBridge 单独验证
阶段 5: 真实 ALX 输入 + G1MotionBridge
```

每进入下一阶段，只替换一个变量：

```text
阶段 2 -> 阶段 3:
  替换输入源
  SimDataSource -> ALXSerialDataSource
  输出仍然是 MockMotionBridge

阶段 3 -> 阶段 4:
  单独验证输出桥
  不进入跟随闭环

阶段 4 -> 阶段 5:
  替换输出桥
  MockMotionBridge -> G1MotionBridge
```

## 10. 关键安全边界

必须避免：

```text
未经 dry-run 验证，直接让 G1 运动。
同时首次接入真实 ALX 和真实 G1。
无遥控器接管时测试。
在障碍物、坡道、台阶或人员密集区域测试。
数据异常时继续发送 Move。
```

必须保证：

```text
任何 data_invalid 都 stop。
任何 data_timeout 都 stop。
任何 too_close 都 stop。
任何 serial_error 都 stop。
任何 sdk_error 都 stop。
任何 manual_stop 都 stop。
```

## 11. 总结

完整验证链路的核心思想是：

```text
仿真先验证软件逻辑。
真实 ALX 先只验证输入。
真实 G1 先只验证输出。
最后才组合成真实闭环。
```

最终目标链路是：

```text
ALXSerialDataSource
-> core pipeline
-> G1MotionBridge
```

但在此之前，必须先经过：

```text
SimDataSource
-> core pipeline
-> MockMotionBridge
```

以及：

```text
ALXSerialDataSource
-> core pipeline
-> MockMotionBridge
```

这样可以最大限度降低调试复杂度和真实机器人运动风险。
