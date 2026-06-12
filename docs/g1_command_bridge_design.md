# Unitree G1 指令转换桥技术方案

## 1. 目标

G1 指令转换桥负责把项目内部的 `MotionCommand` 转换为 Unitree
`unitree_sdk2_python` 的 G1 高层运动控制调用。它只承担输出适配职责，不重新实现
ALX-G100 解析、滤波、安全判断或跟随控制算法。

首版目标：

```text
core MotionCommand
-> G1CommandBridge / G1MotionBridge
-> unitree_sdk2py.g1.loco.LocoClient
-> Unitree G1 sport/loco 高层速度控制
```

首版明确不做：

```text
不直接控制关节电机。
不使用低层 LowCmd/MotorCmd。
不做路径规划、避障或视觉融合。
不在导入模块时初始化 DDS 或连接机器人。
```

## 2. SDK 依据

根据 Unitree 官方 `unitree_sdk2_python`：

```text
依赖:
  Python >= 3.8
  cyclonedds == 0.10.2
  numpy
  opencv-python

通信:
  SDK 通过 DDS 进行请求-响应和 topic 发布/订阅。
  示例运行前需要配置机器人网络连接，并传入连接 G1 的网卡名。

高层控制:
  G1 示例使用 unitree_sdk2py.g1.loco.g1_loco_client.LocoClient。
  初始化顺序为 ChannelFactoryInitialize(0, network_interface)
  -> LocoClient()
  -> SetTimeout(...)
  -> Init()

LocoClient 关键接口:
  Move(vx, vy, vyaw, continous_move=False)
  StopMove()
  SetVelocity(vx, vy, omega, duration=1.0)

Move 的内部行为:
  continous_move=False 时 duration=1。
  continous_move=True 时 duration=864000.0。
  StopMove 等价于 SetVelocity(0, 0, 0)。

底层服务:
  LOCO_SERVICE_NAME = "sport"
  速度 API id = 7105
```

因此本项目首版应使用 G1 高层 `LocoClient`，并将内部速度命令映射为
`Move(vx, vy, omega)` 或 `StopMove()`。

参考链接：

```text
https://github.com/unitreerobotics/unitree_sdk2_python
https://raw.githubusercontent.com/unitreerobotics/unitree_sdk2_python/master/unitree_sdk2py/g1/loco/g1_loco_client.py
https://raw.githubusercontent.com/unitreerobotics/unitree_sdk2_python/master/unitree_sdk2py/g1/loco/g1_loco_api.py
```

## 3. 输入输出契约

项目内部统一输出为：

```python
MotionCommand(
    kind="move" | "stop",
    vx=float,       # m/s, 正值前进
    vy=float,       # m/s, 首版固定为 0
    omega=float,    # rad/s, yaw angular velocity
    reason=str,
)
```

转换桥对外提供：

```python
class G1MotionBridge:
    def __init__(self, network_interface: str, config: G1BridgeConfig | None = None): ...
    def init(self) -> None: ...
    def dispatch(self, command: MotionCommand) -> None: ...
    def move(self, command: MotionCommand) -> None: ...
    def stop(self, reason: str | MotionCommand = "stop") -> None: ...
    def close(self) -> None: ...
```

调用约定：

```text
kind="stop":
  必须立即调用 StopMove。

kind="move":
  先做桥内二次校验和限幅。
  通过后调用 LocoClient.Move(vx, vy, omega, continous_move=False)。

异常:
  任何 SDK 初始化、Move、StopMove 调用异常都记录为 sdk_error。
  Move 异常后必须尝试 StopMove。
  异常状态反馈给主循环，使下一帧进入 SafetyManager 的 sdk_error stop。
```

## 4. 命令映射规则

### 4.1 坐标与符号

项目当前约定：

```text
vx > 0: 前进
vx < 0: 后退
vy = 0: 不横移
omega: 绕竖直轴转向角速度，单位 rad/s
```

G1 `LocoClient.Move(vx, vy, vyaw)` 与该接口天然匹配。唯一需要现场确认的是
`omega` 正方向是否与 ALX-G100 方位角正方向一致。桥层不应硬编码算法逻辑，但应提供
一个可配置符号项：

```text
yaw_sign = +1 或 -1
```

实际下发：

```text
sdk_vx = vx
sdk_vy = 0
sdk_omega = yaw_sign * omega
```

如果现场发现“目标在左侧但 G1 右转”，只调整 `yaw_sign`，不改
`FollowController`。

### 4.2 停止与保持

必须区分：

```text
MotionCommand(kind="stop"):
  安全停止，调用 StopMove。

MotionCommand(kind="move", vx=0, vy=0, omega=0, reason="holding"):
  正常保持，可调用 Move(0,0,0) 或降频发送。
```

首版建议：

```text
stop 命令:
  每次都调用 StopMove，优先保证安全语义。

holding move:
  可调用 Move(0,0,0)，但允许做重复命令抑制。
```

不要把所有零速度都视为 `StopMove`，否则日志和状态机会混淆“正常保持”和“安全停止”。

### 4.3 Move duration

SDK `Move(..., continous_move=False)` 内部使用 `duration=1`。本项目主循环建议
20-40 Hz 连续刷新速度命令，因此首版使用默认非 continuous 模式。

不建议首版使用 `continous_move=True`，因为它会设置非常长的持续时间。若主循环崩溃或
DDS 链路异常，机器人可能保持旧速度更久。真实运行时必须依赖本项目侧看门狗和
StopMove 做短周期刷新。

## 5. 桥内安全策略

`SafetyManager` 是主安全门，转换桥是最后防线。桥内必须再做一次防御式校验：

```text
1. 未 init 时拒绝 Move，只允许 StopMove 尝试。
2. 非有限数值 NaN/inf 直接 StopMove，并标记 sdk_error。
3. kind 非 move/stop 直接拒绝。
4. vy 首版强制归零。
5. vx、omega 进入 SDK 前再次限幅。
6. 发送频率过低或长时间未收到新命令时触发本地 StopMove。
```

建议默认限幅：

```text
max_abs_vx: 0.20 m/s
max_abs_vy: 0.00 m/s
max_abs_omega: 0.60 rad/s
command_timeout_s: 0.30 ~ 0.50 s
sdk_timeout_s: 2.0 ~ 10.0 s
```

现场第一次运动验证建议更保守：

```text
max_abs_vx: 0.05 ~ 0.10 m/s
max_abs_omega: 0.20 ~ 0.30 rad/s
```

## 6. 状态机

转换桥内部建议维护轻量状态：

```text
CREATED
  尚未初始化 SDK。

INITIALIZED
  DDS 和 LocoClient 初始化完成。

ACTIVE
  已经发送过 move。

STOPPED
  最近一次命令为 StopMove 或安全停止。

ERROR
  SDK 调用异常或桥内校验失败。
```

状态转移：

```text
CREATED -> INITIALIZED:
  init() 成功。

INITIALIZED/STOPPED -> ACTIVE:
  收到合法 move 并 Move 调用成功。

ACTIVE/INITIALIZED -> STOPPED:
  收到 stop 或 watch dog 超时，StopMove 调用成功。

任意状态 -> ERROR:
  SDK 调用异常、非法数值、初始化失败。

ERROR -> STOPPED:
  stop() 调用成功并清除运动输出。
```

主循环不要在 `ERROR` 状态继续发送 Move，应将 `sdk_error=True` 送入
`FollowPipeline.process(...)`，让 pipeline 输出 stop。

## 7. 建议实现结构

新增文件：

```text
src/robot_follow/g1_motion.py
tests/test_g1_motion.py
```

`src/robot_follow/g1_motion.py`：

```python
@dataclass
class G1BridgeConfig:
    max_abs_vx: float = 0.20
    max_abs_vy: float = 0.0
    max_abs_omega: float = 0.60
    yaw_sign: int = 1
    sdk_timeout_s: float = 10.0
    command_timeout_s: float = 0.5
    continuous_move: bool = False
```

导入策略：

```text
模块顶层不要 import unitree_sdk2py。
init() 内部延迟 import。
这样无 SDK 的开发机仍可运行单元测试、GUI sim 和 dry-run。
```

初始化伪代码：

```python
def init(self) -> None:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

    ChannelFactoryInitialize(0, self.network_interface)
    self._client = LocoClient()
    self._client.SetTimeout(self.config.sdk_timeout_s)
    self._client.Init()
    self._initialized = True
```

分发伪代码：

```python
def dispatch(self, command: MotionCommand) -> None:
    if command.kind == "stop":
        self.stop(command)
        return
    self.move(command)
```

Move 伪代码：

```python
def move(self, command: MotionCommand) -> None:
    self._require_initialized()
    self._validate_move_command(command)

    vx = clamp(command.vx, -cfg.max_abs_vx, cfg.max_abs_vx)
    vy = 0.0
    omega = clamp(cfg.yaw_sign * command.omega, -cfg.max_abs_omega, cfg.max_abs_omega)

    try:
        self._client.Move(vx, vy, omega, cfg.continuous_move)
    except Exception:
        self._sdk_error = True
        self._best_effort_stop()
        raise
```

Stop 伪代码：

```python
def stop(self, reason="stop") -> None:
    if self._client is None:
        return
    try:
        self._client.StopMove()
    except Exception:
        self._sdk_error = True
        raise
```

Watchdog：

```text
桥对象记录 last_dispatch_time。
主循环每个 tick 调用 bridge.tick(now)。
若 now - last_dispatch_time > command_timeout_s 且最近不是 STOPPED:
  调用 StopMove。
```

## 8. 主循环集成

真实运行链路：

```text
ALXLiveSource / serial reader
-> TargetSample
-> FollowPipeline.process(sample, sdk_error=bridge.sdk_error)
-> MotionCommand
-> G1MotionBridge.dispatch(command)
-> bridge.tick(now)
```

CLI 必须提供安全开关：

```bash
robot-follow real-run \
  --serial-port /dev/ttyUSB0 \
  --net-iface enp2s0 \
  --dry-run
```

真实运动必须显式确认：

```bash
robot-follow real-run \
  --serial-port /dev/ttyUSB0 \
  --net-iface enp2s0 \
  --enable-motion \
  --i-understand-this-moves-robot
```

没有 `--enable-motion` 和 `--i-understand-this-moves-robot` 时，输出桥必须使用
`MockMotionBridge`，不能实例化或调用真实 G1 Move。

## 9. 测试方案

### 9.1 无 SDK 单元测试

用 fake client 注入测试，不依赖 `unitree_sdk2py`：

```text
move command -> fake_client.Move(vx, 0, omega, False)
stop command -> fake_client.StopMove()
holding move -> fake_client.Move(0, 0, 0, False)
NaN/inf -> StopMove + sdk_error
超限速度 -> clamp 后下发
yaw_sign=-1 -> omega 符号翻转
未 init move -> raise 或返回错误，不调用 Move
```

### 9.2 SDK 导入测试

```text
python -c "from robot_follow.g1_motion import G1MotionBridge; print(G1MotionBridge)"
```

通过标准：

```text
无 unitree_sdk2py 环境也能导入。
导入时不初始化 DDS。
导入时不访问网卡。
```

### 9.3 G1 单独输出验证

按照 `docs/alx_g1_verification_chain.md` 阶段 4 执行：

```text
1. 只 Init。
2. 只 StopMove。
3. 极低速 Move 0.5-1.0 s。
4. Move 后立即 StopMove。
5. 遥控器全程在线。
```

### 9.4 闭环前 dry-run

真实 ALX 输入先接 `MockMotionBridge`：

```text
ALXSerialDataSource
-> core pipeline
-> MockMotionBridge
```

确认日志中 `vx/omega/reason/stop reason` 合理后，才允许替换为 G1MotionBridge。

## 10. 风险与处理

```text
风险: G1 yaw 正方向与 ALX 方位角不一致。
处理: 使用 yaw_sign 参数，不改控制器算法。

风险: Move 默认持续 1 秒，主循环中断后旧命令仍短时有效。
处理: 高频刷新 + bridge watchdog + finally StopMove。

风险: Unitree SDK 初始化失败或 DDS 网卡配置错误。
处理: init 失败直接进入 ERROR，不允许 Move；保留 dry-run 路径。

风险: SDK 包未安装导致开发环境测试失败。
处理: 延迟 import，fake client 单测覆盖转换逻辑。

风险: 把 holding 和 safety stop 混在一起。
处理: `MotionCommand.kind` 保持语义，stop 必须调用 StopMove。
```

## 11. 推荐落地顺序

```text
1. 实现 G1BridgeConfig 和 G1MotionBridge，先支持 fake client 注入。
2. 补充无 SDK 单元测试。
3. 加 CLI dry-run / enable-motion 双开关。
4. 在 Jetson 上安装并验证 unitree_sdk2_python 导入。
5. G1 单独做 Init/StopMove/极低速 Move。
6. 真实 ALX 输入 dry-run 校准 yaw_sign。
7. 最后接入真实 G1 闭环，使用最保守速度参数。
```

## 12. 结论

G1 指令转换桥应被设计成一个非常薄但非常保守的硬件输出适配层。它只把
`MotionCommand` 映射到 `LocoClient.Move/StopMove`，并在 SDK 边界增加延迟导入、
限幅、符号校准、watchdog、异常反馈和显式运动确认。

这样可以保持现有 `core pipeline` 的可测试性，也能让真实 G1 运动输出按阶段逐步验证，
降低首次闭环调试风险。
