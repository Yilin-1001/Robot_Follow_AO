# Unitree G1 + ALX-G100 低速跟随 MVP 技术方案

## 1. 项目目标

本项目目标是在 10 天内完成基于 ALX-G100 单基站手环套件、NVIDIA Jetson Orin NX 和 Unitree G1 的低速跟随 MVP 演示系统。

系统通过 ALX-G100 获取佩戴手环人员相对于机器人基站的距离与方位角，由 Jetson 完成串口数据解析、状态判断、跟随控制计算，并通过 `unitree_sdk2_python` 向 Unitree G1 下发高层运动控制命令，使 G1 在受限场景下实现低速跟随。

首版目标是验证技术链路与低速跟随行为，不定位为完整商用品质的自主跟随系统。

## 2. 应用场景与边界

首版 MVP 的运行场景限定为室内空旷平地。演示区域内应提前清理障碍物，避免台阶、坡道、狭窄通道和人员密集区域。

系统首版不包含自主避障、路径规划或复杂环境导航能力。机器人运动仅基于 ALX-G100 输出的距离与方位角进行低速跟随控制。

首版跟随行为定义为：Unitree G1 在跟随过程中尽量保持机身正面对准佩戴手环人员，并根据距离变化低速前进。当人员偏离正前方时，机器人优先进行转向修正；当距离过近、数据丢失或状态异常时，机器人停止运动。

## 3. 系统总体架构

系统链路如下：

```text
ALX-G100 手环
-> ALX-G100 单基站
-> Jetson Orin NX 串口读取
-> 0x2001 定位数据帧解析
-> 距离/角度滤波与异常检测
-> 跟随控制器
-> unitree_sdk2_python G1 LocoClient
-> Unitree G1 高层运动控制
```

首版优先采用 ALX-G100 直接接入 Jetson 的方式。STM32F1-Usart 示例工程仅作为协议解析参考和备用验证工具，不作为首选运行链路。

## 4. 硬件组成

本项目硬件组成如下：

```text
机器人平台: Unitree G1
板载计算单元: NVIDIA Jetson Orin NX 16GB
操作系统: Ubuntu 22.04.5 LTS
L4T: 36.4.4
内核: 5.15.148-tegra
定位套件: ALX-G100 单基站 + 手环
备用参考: STM32F1-Usart 示例工程
```

Jetson 侧当前环境：

```text
设备型号: NVIDIA Jetson Orin NX Engineering Reference Developer Kit
模块: Jetson Orin NX 16GB
系统: Ubuntu 22.04.5 LTS, jammy
L4T: 36.4.4
功耗模式: 25W
CUDA: 12.6.68
cuDNN: 9.3.0.75
TensorRT: 10.3.0.30
VPI: 3.2.4
OpenCV: 4.5.4
```

首版跟随功能不依赖 CUDA、TensorRT 或视觉模型。

## 5. ALX-G100 接入方案

ALX-G100 单基站属于单基站手环跟随套件。根据资料，基站具备 UART TTL 接口，并可通过 USB 接口连接 PC 进行数据传输。

基站本体接口：

```text
VCC: 3-5V, 推荐 3.3V
RX: 3.3V TTL
TX: 3.3V TTL
GND: 地
```

基站转接板接口：

```text
Pin1: TX
Pin2: RX
Pin3: 5V
Pin4: GND
UART: TTL@115200bps
```

推荐首选接入方式：

```text
ALX-G100 基站/转接板
-> USB 或 USB-TTL
-> Jetson Orin NX
-> /dev/ttyUSB*
```

若使用 Jetson GPIO UART，应确保信号为 3.3V TTL，不能将 5V 信号直接接入 Jetson UART。基站供电电压与 UART 信号电平需要分开确认，GND 必须共地。

STM32 备用路径如下：

```text
ALX-G100 单基站
-> STM32F1 示例板
-> STM32 解析 Distance / Azimuth / TagStatus
-> Jetson 接收简化数据
-> Unitree G1 控制
```

该路径仅在 Jetson 无法直接稳定读取 ALX-G100 数据时启用。

## 6. ALX-G100 通信协议

ALX-G100 基站主动向上位机发送定位信息。串口参数如下：

```text
波特率: 115200
数据位: 8
停止位: 1
串口校验: 无
流控: 无
数据格式: 十六进制收发
```

注意：串口层无奇偶校验，但协议帧末尾包含 `XorByte`，需要在软件解析时做异或校验。

定位数据命令码为 `0x2001`。ALX-G100 单基站定位帧的 `VersionID` 固定为 `0x0100`，示例 `PacketLength` 为 `0x0025`。本项目按规格书示例将该字段作为协议帧长度字段校验。

定位帧字段如下：

```text
MessageHeader: 4 byte, 4 个连续 0xFF 表示消息开始
PacketLength: 2 byte, 协议帧长度字段，定位帧示例为 0x0025
SequenceID: 2 byte, 消息流水号
RequestCommand: 2 byte, 命令码
VersionID: 2 byte, 协议版本，定位帧固定为 0x0100
AnchorID: 4 byte, 基站 ID
TagID: 4 byte, 标签 ID
Distance: 4 byte, 标签与基站间距离, 单位 cm
Azimuth: 2 byte, 标签与基站间方位角, 单位度
Elevation: 2 byte, 标签与基站间仰角, 单位度
TagStatus: 2 byte, 标签状态
BatchSn: 2 byte, 测距序号
Reserve: 4 byte, 预留
XorByte: 1 byte, 前序字节异或校验
```

基站一方不在线时，基站会约每 2 s 主动发送一次心跳帧：

```text
心跳命令码: 0x2002
PacketLength: 0x0010
VersionID: 0x0101
字段: MessageHeader, PacketLength, SequenceID, RequestCommand, VersionID, AnchorID
```

Jetson 端统一将协议数据转换为控制内部单位：

```text
distance_m = distance_cm / 100.0
azimuth_rad = azimuth_deg * pi / 180.0
```

规格书示例显示上传数据自带卡尔曼滤波。Jetson 端仍需要进行基本异常检测和必要的平滑处理，防止单帧跳变直接驱动机器人。

## 7. Jetson 软件架构

首版软件确定采用 Python 3 开发。Python 负责 ALX-G100 串口解析、定位数据处理、跟随控制状态机、安全急停以及 Unitree G1 高层控制调用。

机器人控制通过 `unitree_sdk2_python` 的 G1 `LocoClient` 实现。C++17 仅作为备用方案，在 Python SDK 无法稳定安装或控制链路无法稳定运行时启用。

首版核心依赖：

```text
Python 3
unitree_sdk2_python
cyclonedds == 0.10.2
pyserial
numpy
```

推荐模块划分：

```text
alx_g100_serial_reader
  基于 pyserial 打开串口，读取原始字节流，完成帧头查找、长度判断和校验。

alx_g100_protocol_parser
  解析 0x2001 定位帧，输出 distance_cm、azimuth_deg、elevation_deg、tag_status、timestamp。

follow_state_filter
  对距离、角度和时间戳进行基础滤波、跳变检测和有效性判断。

follow_controller
  根据目标距离和方位角计算 vx、vy、omega。

g1_motion_bridge
  封装 unitree_sdk2_python 的 G1 LocoClient，负责 Init、Move、StopMove 等高层控制命令。

safety_manager
  处理数据超时、距离过近、状态异常、串口异常、Python SDK 异常和人工软件急停。
```

软件主循环建议逻辑：

```text
读取 ALX-G100 数据
-> 解析定位帧
-> 检查数据有效性
-> 更新跟随状态
-> 计算速度命令
-> 安全限幅
-> 发送 Move 或 StopMove
-> 记录日志
```

## 8. Unitree G1 控制方案

Jetson 通过 `unitree_sdk2_python` 控制 Unitree G1。当前 Jetson 与 G1 的通信和控制链路已完成前期验证，因此首版重点不在从零验证机器人控制能力，而在接入 ALX-G100 数据并形成稳定的跟随闭环。

首版使用 G1 高层运动控制接口，不直接控制关节电机。

控制接口抽象如下：

```text
Init()
Move(vx, vy, omega)
StopMove()
```

速度含义：

```text
vx: 前后方向速度, 正值前进
vy: 横向速度, 首版固定为 0
omega: 转向角速度
```

首版不使用横移控制，`vy = 0`。机器人通过转向保持正面对准人员，再根据距离误差低速前进。

## 9. 跟随控制逻辑

首版跟随策略为“优先对准方向，再低速前进”。

控制目标：

```text
G1 正面对准佩戴手环人员
保持约 1.2-1.5 m 的目标跟随距离
仅在室内空旷平地低速运行
```

控制输入：

```text
distance_m: 手环与基站距离, 单位 m
azimuth_deg: 手环相对于基站方位角, 单位度
tag_status: 标签状态
timestamp: 数据时间戳
```

控制输出：

```text
vx: G1 前进速度
vy: 0
omega: G1 转向角速度
```

建议首版参数范围：

```text
目标距离: 1.2-1.5 m
最大前进速度: 0.15-0.20 m/s
最大角速度: 约 0.3 rad/s
角度死区: 约 +/-5 deg
大角度阈值: 约 +/-20 deg
最小安全距离: 0.7-0.8 m
```

行为逻辑：

```text
如果数据无效:
  StopMove

如果距离过近:
  StopMove

如果 |Azimuth| > 大角度阈值:
  vx = 0
  omega = 限幅后的转向速度

如果 |Azimuth| 在中等范围:
  vx = 较低前进速度
  omega = 限幅后的转向修正

如果 |Azimuth| 在角度死区内:
  omega = 0 或小修正
  vx = 根据距离误差计算并限幅
```

首版不建议让 G1 在明显未对准人员时继续前进，避免机器人斜向接近人员或偏离演示区域。

## 10. 安全策略

首版 MVP 采用“原厂遥控器接管 + Jetson 软件急停”的安全策略，不额外接入外部物理急停按钮。

### 10.1 原厂遥控器接管

测试和演示期间，Unitree G1 原厂遥控器保持在线。现场人员可随时通过遥控器中断自动跟随或接管机器人。

### 10.2 异常状态自动软件急停

Jetson 控制程序检测到异常状态后，应立即调用 `StopMove()`。

自动急停触发条件包括：

```text
ALX-G100 串口数据超时
连续一段时间未收到 0x2001 定位帧
Distance 小于最小安全距离
Distance 单帧跳变过大
Azimuth 单帧跳变过大
串口读取异常
unitree_sdk2_python 控制调用异常
程序准备退出
```

建议初始阈值：

```text
数据超时: 300-500 ms
最小安全距离: 0.7-0.8 m
距离跳变阈值: 单帧变化大于 0.5 m
角度跳变阈值: 单帧变化大于 45 deg
```

`TagStatus` 首版应完成解析和日志记录，但在状态码含义未通过资料或实测确认前，不作为强制急停条件。待确认异常状态码后，再将其加入 `safety_manager`。

### 10.3 SSH/键盘人工软件急停

开发人员可通过 SSH 登录 Jetson，在控制程序终端中触发人工软件急停。

建议实现：

```text
q + Enter: 调用 StopMove 并退出程序
Ctrl+C: 捕获 SIGINT，先调用 StopMove，再退出程序
异常退出: 在清理逻辑中调用 StopMove
```

## 11. 技术风险与待确认项

当前主要技术风险如下：

```text
ALX-G100 数据刷新率、稳定性和实际抖动需要实测确认。
Azimuth 正负方向需要通过现场移动手环确认。
TagStatus 字段具体状态含义需要结合实测或进一步协议资料确认。
G1 在低速跟随时的实际运动响应需要现场调参。
室内环境中 UWB/AOA 数据可能受安装高度、人体遮挡和多径影响。
首版无自主避障能力，演示场地必须为空旷平地。
```

已降低风险项：

```text
Jetson Orin NX 性能满足本项目需求。
Jetson 与 Unitree G1 的通信和控制链路已完成前期验证。
ALX-G100 协议格式、距离单位和串口参数已从规格书确认。
```
