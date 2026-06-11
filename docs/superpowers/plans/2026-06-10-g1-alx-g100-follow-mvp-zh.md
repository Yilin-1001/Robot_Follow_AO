# G1 + ALX-G100 低速跟随 MVP 中文执行计划

> **执行说明：** 该计划用于按任务逐步实现 MVP。正式执行代码实现时，建议使用 `superpowers:subagent-driven-development` 或 `superpowers:executing-plans`，按任务逐项完成、测试、提交。

**目标：** 使用 Python 3 在 Jetson 上读取 ALX-G100 定位数据，计算低速跟随控制指令，并通过 `unitree_sdk2_python` 控制 Unitree G1。

**架构：** 项目实现为一个小型 Python 包，按职责拆分为协议解析、串口读取、滤波、安全判断、跟随控制和 G1 控制桥。先用单元测试验证协议解析、控制算法和安全逻辑，再进行 Jetson + ALX-G100 + G1 的硬件闭环联调。

**技术栈：** Python 3、`pyserial`、`numpy`、`unitree_sdk2_python`、`cyclonedds == 0.10.2`、`pytest`。

---

## 1. 文件结构

计划创建如下结构：

```text
pyproject.toml
src/robot_follow/__init__.py
src/robot_follow/alx_g100_protocol.py
src/robot_follow/serial_reader.py
src/robot_follow/filtering.py
src/robot_follow/safety.py
src/robot_follow/follow_controller.py
src/robot_follow/g1_motion.py
src/robot_follow/main.py
tests/test_alx_g100_protocol.py
tests/test_safety.py
tests/test_follow_controller.py
```

各文件职责：

```text
alx_g100_protocol.py
  纯协议解析模块，解析 ALX-G100 的 0x2001 定位帧和 0x2002 心跳帧。

serial_reader.py
  基于 pyserial 读取串口字节流，并输出解析后的定位样本。

filtering.py
  对距离和方位角做轻量滤波，同时计算距离/角度跳变。跳变检测输出供安全急停模块使用。

safety.py
  判断是否需要停止，包括数据超时、距离过近、跳变过大、SDK 异常和人工停止。依赖滤波模块的跳变检测接口。

follow_controller.py
  根据滤波后的距离和方位角计算 vx、vy、omega。

g1_motion.py
  封装 unitree_sdk2_python 的 G1 LocoClient。可与其他模块并行开发。

main.py
  运行入口，连接串口读取、控制算法、安全逻辑和 G1 控制桥。
```

---

## 2. 任务拆分

### 任务 1：Python 项目脚手架

**目标：** 建立 Python 包结构和测试环境。

**创建文件：**

```text
pyproject.toml
src/robot_follow/__init__.py
```

**关键内容：**

```toml
[project]
name = "robot-follow"
version = "0.1.0"
requires-python = ">=3.8"
dependencies = [
  "pyserial>=3.5",
  "numpy>=1.23",
]

[project.optional-dependencies]
dev = [
  "pytest>=7.0",
]

[project.scripts]
robot-follow = "robot_follow.main:main"
```

**验证命令：**

```bash
python3 -m pip install -e ".[dev]"
pytest -q
```

**预期结果：**

```text
包可安装；初始阶段可以没有测试运行。
```

---

### 任务 2：ALX-G100 协议解析器

**目标：** 解析 ALX-G100 的定位帧，得到 `distance_cm`、`azimuth_deg`、`elevation_deg`、`tag_status` 等字段。

**创建文件：**

```text
src/robot_follow/alx_g100_protocol.py
tests/test_alx_g100_protocol.py
```

**协议事实：**

```text
帧头: FF FF FF FF
定位命令: 0x2001
定位帧 PacketLength: 0x0025
定位帧 VersionID: 0x0100
心跳命令: 0x2002
心跳帧 PacketLength: 0x0010
心跳帧 VersionID: 0x0101
Distance: 4 字节，无符号整数，单位 cm
Azimuth: 2 字节，有符号整数，单位度
Elevation: 2 字节，有符号整数，单位度
XorByte: 定位帧最后 1 字节，对该字节前所有字节做异或校验
```

**需要实现的数据结构：**

```python
@dataclass(frozen=True)
class PositionSample:
    command: int
    version_id: int
    anchor_id: int
    tag_id: int
    distance_cm: int
    azimuth_deg: int
    elevation_deg: int
    tag_status: int
    batch_sn: int
    timestamp: float

    @property
    def distance_m(self) -> float:
        return self.distance_cm / 100.0
```

**必须覆盖的测试：**

```text
正角度定位帧解析
负角度定位帧解析
心跳帧解析
错误帧头拒绝
错误 XOR 校验拒绝
错误 PacketLength 拒绝
```

**验证命令：**

```bash
pytest tests/test_alx_g100_protocol.py -q
```

**预期结果：**

```text
6 passed
```

---

### 任务 3：ALX-G100 串口读取器

**目标：** 从 Jetson 的 `/dev/ttyUSB*` 或其他串口设备读取 ALX-G100 字节流。

**创建文件：**

```text
src/robot_follow/serial_reader.py
```

**串口参数：**

```text
波特率: 115200
数据位: 8
停止位: 1
校验: 无
流控: 无
```

这里的“校验: 无”指串口层无奇偶校验；ALX-G100 协议层定位帧仍包含 `XorByte`，软件解析时需要校验。

**核心逻辑：**

```text
打开串口
持续读取字节
查找 FF FF FF FF 帧头
根据命令码判断帧长
校验 PacketLength 字段
调用 parse_frame()
输出 PositionSample 或 HeartbeatSample
```

实现要求：串口读取必须使用短超时轮询接口，不能让主循环永久阻塞在等待样本上；找不到完整帧头时，应保留最后 3 个字节，避免跨串口读取边界的 `FF FF FF FF` 帧头被误删。

**验证命令：**

```bash
python3 -c "from robot_follow.serial_reader import ALXG100SerialReader; print(ALXG100SerialReader)"
```

**预期结果：**

```text
<class 'robot_follow.serial_reader.ALXG100SerialReader'>
```

---

### 任务 4：距离与角度滤波

**目标：** 对 ALX-G100 输出做轻量平滑，并计算跳变量。本任务必须先于安全急停管理完成，因为安全模块需要消费本模块输出的 `distance_jump` 和 `angle_jump`。

**创建文件：**

```text
src/robot_follow/filtering.py
```

**建议实现：**

```text
使用一阶低通滤波
默认 alpha = 0.35
输出 filtered distance、filtered azimuth、distance_jump、angle_jump
```

**验证命令：**

```bash
python3 -c "from robot_follow.filtering import TargetFilter; print(TargetFilter().update(1.2, 3.0))"
pytest -q
```

**预期结果：**

```text
可以打印 FilteredTarget
所有已有测试通过
```

---

### 任务 5：安全急停管理

**目标：** 独立判断是否必须调用 `StopMove()`。距离跳变和角度跳变的检测依赖任务 4 滤波模块输出的 `distance_jump` 和 `angle_jump`。

**创建文件：**

```text
src/robot_follow/safety.py
tests/test_safety.py
```

**急停触发条件：**

```text
定位数据超时
距离小于最小安全距离
距离单帧跳变过大（由滤波模块提供）
角度单帧跳变过大（由滤波模块提供）
串口异常
unitree_sdk2_python 控制异常
程序退出
```

**初始阈值：**

```text
数据超时: 0.5 s
最小安全距离: 0.75 m
最大距离跳变: 0.5 m
最大角度跳变: 45 度
```

**验证命令：**

```bash
pytest tests/test_safety.py -q
```

**预期结果：**

```text
4 passed
```

---

### 任务 6：跟随控制器

**目标：** 将滤波后的 `distance_m` 和 `azimuth_deg` 转换为 G1 的速度控制量。

**创建文件：**

```text
src/robot_follow/follow_controller.py
tests/test_follow_controller.py
```

**输入：**

```text
distance_m（经滤波平滑）
azimuth_deg（经滤波平滑）
```

**输出：**

```text
vx
vy = 0
omega
reason
```

**初始参数：**

```text
目标距离: 1.3 m
最小安全距离: 0.75 m
最大前进速度: 0.2 m/s
最大角速度: 0.3 rad/s
角度死区: ±5 度
大角度阈值: ±20 度
```

**控制逻辑：**

```text
距离过近: 停止
角度偏差大: 原地/低速转向，vx = 0
角度中等: 低速前进，同时转向修正
角度在死区内: 根据距离误差前进
```

**必须覆盖的测试：**

```text
距离过近停止
角度过大原地转向
正前方且距离较远时前进
中等角度时低速前进并转向
```

**验证命令：**

```bash
pytest tests/test_follow_controller.py -q
```

**预期结果：**

```text
4 passed
```

---

### 任务 7：G1 控制桥（可与任务 4-6 并行开发）

**目标：** 封装 `unitree_sdk2_python` 的 G1 `LocoClient`，提供统一的 `init()`、`move()`、`stop()` 接口。

> **说明：** 本模块仅依赖 `unitree_sdk2_python`，与 ALX-G100 数据管线完全独立，可在任务 4-6 开发期间并行推进，提前发现 SDK 集成问题。

**创建文件：**

```text
src/robot_follow/g1_motion.py
```

**核心逻辑：**

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

ChannelFactoryInitialize(0, network_interface)
client = LocoClient()
client.SetTimeout(10.0)
client.Init()
client.Move(vx, vy, omega)
client.StopMove()
```

**验证命令：**

```bash
python3 -c "from robot_follow.g1_motion import G1MotionBridge; print(G1MotionBridge)"
```

**预期结果：**

```text
导入模块时不直接初始化 SDK，不产生网络副作用。
```

---

### 任务 8：运行入口 CLI

**目标：** 提供完整运行命令，把串口、解析、滤波、控制、安全和 G1 控制串起来。

**创建文件：**

```text
src/robot_follow/main.py
```

**CLI 参数：**

```text
--serial-port   ALX-G100 串口，例如 /dev/ttyUSB0
--net-iface     G1 通信网卡，例如 enp2s0
--dry-run       只打印控制结果，不控制 G1
```

**运行逻辑：**

```text
初始化 FollowController
初始化 SafetyManager
初始化 TargetFilter
打开 ALX-G100 串口
读取 0x2001 定位帧
计算滤波后的距离和角度
固定周期检查定位数据是否超时
判断是否需要 StopMove
计算 vx / omega
dry-run 时只打印
非 dry-run 时调用 G1 Move
q + Enter 或 Ctrl+C 时调用 StopMove 后退出
```

**验证命令：**

```bash
python3 -m robot_follow.main --help
pytest -q
```

**预期结果：**

```text
CLI help 正常输出
所有单元测试通过
```

---

## 3. Jetson 硬件联调步骤

### 步骤 1：识别 ALX-G100 串口

插入 ALX-G100 后运行：

```bash
dmesg | tail -n 40
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

预期看到：

```text
/dev/ttyUSB0
```

实际端口名可能是 `/dev/ttyUSB1` 或 `/dev/ttyACM0`，以后续实测为准。

---

### 步骤 2：dry-run 验证串口解析

> **说明：** 本步骤验证 ALX-G100 数据管线，与步骤 4（G1 SDK 验证）互相独立，可并行执行。

运行：

```bash
robot-follow --serial-port /dev/ttyUSB0 --net-iface enp2s0 --dry-run
```

移动手环时预期输出：

```text
distance=1.25m azimuth=...
```

如果没有输出，优先检查：

```text
串口设备名
ALX-G100 是否供电
手环是否进入跟随模式
串口权限
波特率是否为 115200
```

---

### 步骤 3：确认方位角正负方向

将手环分别移动到机器人左侧和右侧，记录：

```text
左侧 azimuth 符号:
右侧 azimuth 符号:
```

如果闭环测试时转向方向反了，修改：

```python
angle_gain: float = -0.02
```

---

### 步骤 4：验证 G1 Python SDK

> **说明：** 本步骤验证 G1 控制链路，与步骤 2-3（ALX-G100 数据验证）互相独立，可并行执行。

先运行宇树官方 Python 示例：

```bash
python3 example/g1/high_level/g1_loco_client_example.py enp2s0
```

预期输出：

```text
Input "list" to list all test option ...
```

只在室内空旷平地低速测试移动和转向选项。

---

### 步骤 5：低速闭环测试

运行：

```bash
robot-follow --serial-port /dev/ttyUSB0 --net-iface enp2s0
```

预期行为：

```text
G1 优先转向面对手环。
当距离大于目标距离时，G1 低速前进。
当距离过近或数据异常时，G1 停止。
```

---

### 步骤 6：软件急停验证

机器人运动时，在 SSH 终端中按：

```text
q，然后按 Enter
```

预期：

```text
程序调用 StopMove，G1 停止。
```

再次运行程序，按：

```text
Ctrl+C
```

预期：

```text
程序捕获中断，先调用 StopMove，再退出。
```

---

## 4. 执行顺序建议

建议顺序：

```text
阶段一：数据管线基础
  1. 项目脚手架
  2. 协议解析器
  3. 串口读取器（紧跟协议解析器）

阶段二：算法与安全
  4. 滤波模块（提供跳变检测，为安全模块的依赖项）
  5. 安全急停管理（依赖滤波模块的 distance_jump / angle_jump）
  6. 跟随控制器（依赖滤波后的平滑数据）

  [并行] 7. G1 控制桥（仅依赖 unitree_sdk2_python，可与 4-6 并行）

阶段三：系统集成
  8. CLI 主程序

阶段四：硬件验证（增量闭环）
  9.  Jetson dry-run 串口验证  ─┐
  10. G1 SDK 独立验证           ─┤← 可并行
  11. 低速闭环联调               ─┘
```

优先保证：

```text
协议解析正确
StopMove 可靠
dry-run 数据稳定
G1 低速控制可控
```

再进行闭环跟随。

---

## 5. 自检

该计划覆盖：

```text
ALX-G100 协议解析
Distance cm 转 m
Azimuth 度转控制角度
Jetson 直连串口
Python 3 主开发语言
unitree_sdk2_python G1 控制
正面对人低速前进
异常自动 StopMove
SSH/q + Enter/Ctrl+C 人工软件急停
不实现自主避障
```

该计划不包含：

```text
客户验收标准
详细日程排期
后续扩展方向
自主避障
多人识别
视觉跟随
```
