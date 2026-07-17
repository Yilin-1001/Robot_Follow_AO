# Robot Follow

Robot Follow 是一个面向 ALX-G100 UWB 定位接收器和 Unitree G1 机器人的低速人员跟随项目。项目目标是在 Jetson 或本地开发环境中读取腕带/标签相对位置，经过滤波、安全判断和跟随控制，输出 G1 可执行的前后速度与旋转速度命令。

当前实现重点是 MVP 闭环验证：先用真实 ALX 数据做 dry-run，再通过保守参数接入 G1 运动桥，最后用带日志的现场测试流程复现实验结果。

## 功能概览

- 解析 ALX-G100 串口定位数据，支持断线/读数异常后的自动重连。
- 独立的核心跟随管线：数据有效性检查、低通滤波、安全停止、距离/角度控制。
- 支持 dry-run 模式，默认只记录运动命令，不驱动真实机器人。
- 支持 Unitree G1 LocoClient 输出，真实运动必须显式双重确认。
- 支持 CSV、控制台日志、元数据和 zip 打包，便于现场测试留档。
- 提供仿真、核心逻辑、ALX 输入、G1 桥接等测试用例。

## 项目结构

```text
src/robot_follow/
  alx_g100_protocol.py       # ALX-G100 协议解析
  serial_reader.py           # 串口读取
  main.py                    # 主运行入口：ALX -> pipeline -> motion bridge
  run_logger.py              # 带日志的测试运行器
  core/
    filtering.py             # 目标距离/角度滤波
    safety.py                # 安全停止条件
    follow_controller.py     # 跟随控制器
    pipeline.py              # 单帧处理链路
    types.py                 # 核心数据类型
  real/
    alx_source.py            # 真实 ALX 数据源
    g1/bridge.py             # Unitree G1 运动桥
  sim/
    mock_motion_bridge.py    # 不驱动硬件的模拟运动桥
    GUI_sim/                 # GUI 仿真相关模块
  lidar_alx_assisted_follow/ # 当前 LiDAR + ALX 跟随主链路
    sensors/                 # ALX、点云和手环状态输入
    tracking/                # 点云预处理与 ALX 位置滤波
    motion/
      avoidance/             # 可插拔避障模式；当前为 APF
      apf.py                 # APF 具体算法（兼容旧导入）
    runtime/                 # 云驱动控制循环、日志和状态
    web/                     # 原型监控与控制界面
  ao/                        # FAST-LIO 点云源、雷达服务与旧 AO viewer 复用层

configs/
  jetson_g1_safe.yaml        # Jetson + G1 现场测试保守配置

docs/
  ...                        # 硬件接线、现场测试、设计和验证文档

tests/
  ...                        # 核心、仿真、真实适配层测试
```

## 环境准备

项目使用 Python 3.8+，推荐使用仓库当前开发环境 `robot-follow`：

```bash
conda create -n robot-follow python=3.10
conda activate robot-follow
pip install -e ".[dev]"
```

基础依赖包括：

- `pyserial`：读取 ALX-G100 串口数据
- `numpy`：数值处理
- `pytest`：开发测试

真实 G1 运动需要额外安装并配置 Unitree SDK。SDK 只在启用真实运动时延迟导入，dry-run 和大多数测试不需要安装 SDK。

## 快速运行

默认运行不会驱动真实机器人，只使用 `MockMotionBridge` 记录命令：

```bash
conda run -n robot-follow python -m robot_follow.main \
  --config configs/jetson_g1_safe.yaml \
  --dry-run
```

指定 ALX 串口、运行时长和 CSV 输出：

```bash
conda run -n robot-follow python -m robot_follow.main \
  --port /dev/ttyUSB0 \
  --baudrate 115200 \
  --duration 10 \
  --csv logs/alx_dry_run.csv \
  --dry-run
```

也可以通过安装后的脚本入口运行：

```bash
robot-follow --config configs/jetson_g1_safe.yaml --dry-run
```

## AO Radar Web 点云可视化

项目已复用 Robot FollowAO Radar 的成熟 Web viewer，前端源码和已构建的 `dist` 位于 `src/robot_follow/ao/web/frontend/`。Jetson 上有 ROS2/FAST-LIO 点云时可直接启动：

```bash
conda run -n robot-follow robot-follow-ao-web \
  --host 0.0.0.0 \
  --port 8080 \
  --topic /cloud_registered_body
```

然后在同一局域网电脑浏览器打开 `http://<Jetson IP>:8080` 查看点云、zone 和避障决策。无 ROS 环境下可先看模拟页面：

```bash
conda run -n robot-follow robot-follow-ao-web-sim --host 0.0.0.0 --port 8080
```

如果已经在仓库根目录，也可以直接使用 Python 模块入口；这种方式适合开发调试：

```bash
conda run -n robot-follow python -m robot_follow.main --config configs/jetson_g1_safe.yaml --dry-run
```

## ALX ROS2 Kalman 位置 Topic

在 Jetson 已 source ROS2 Foxy 和 FAST-LIO workspace，且 `rclpy`、
`geometry_msgs` 可被 `robot_followao_radar` 导入时，辅助跟随主程序会默认
自动启动并在退出时关闭 ALX 的 ROS2 发布节点。以下独立命令仅供诊断或外部
进程管理使用：

```bash
conda run -n robot_followao_radar robot-follow-alx-ros \
  configs/jetson_g1_lidar_alx_assisted_follow.yaml
```

若尚未通过 editable install 安装该脚本，可将 `robot-follow-alx-ros` 替换为
`python -m robot_follow.lidar_alx_assisted_follow.alx_ros_node`。

该节点是 `/dev/ttyUSB0` 的唯一读取者，发布两个 topic：`/alx/position` 和
`/alx/wristband_state`。前者是 `geometry_msgs/PointStamped`、`frame_id=body`，
随 ALX 有效新帧更新（当前实测约 21 Hz）；`point.x` 和 `point.y` 是经 Kalman
平滑后的 ALX 水平 body 坐标。后者是可靠、保留最后状态的 `std_msgs/UInt8`：
`0=UNKNOWN`、`1=ON`、`2=OFF`、`3=SERIAL_FAULT`，用于手环控制雷达服务启停。
Kalman 和手环 topic 参数位于同一 YAML 的 `alx_ros` 节。

启动后先在零运动条件下检查：

```bash
ros2 topic hz /alx/position
ros2 topic echo /alx/position
ros2 topic echo /alx/wristband_state
```

## 真实机器人运行

真实 G1 运动默认关闭。必须同时传入 `--enable-motion` 和 `--i-understand-this-moves-robot`，程序才会创建 `G1MotionBridge` 并向机器人发送速度命令：

```bash
conda run -n robot-follow python -m robot_follow.main \
  --config configs/jetson_g1_safe.yaml \
  --enable-motion \
  --i-understand-this-moves-robot
```

运行前请确认：

- ALX-G100 接收器串口设备正确，例如 `/dev/ttyUSB0`。
- Jetson 到 G1 的 DDS 网卡名正确，例如 `eth0`。
- G1 已按 Unitree 要求进入可运动状态。
- 现场有足够安全距离，并有人随时准备物理急停。
- 第一轮闭环使用短时长、低速度、低角速度参数。

关键参数可在 `configs/jetson_g1_safe.yaml` 中调整：

- `alx.port` / `alx.baudrate`：ALX 串口配置
- `g1.net_iface`：G1 DDS 网络接口
- `g1.max_vx` / `g1.max_omega`：真实输出限幅
- `safety.*`：数据超时、最小安全距离、跳变检测阈值
- `follow.*`：目标跟随距离、速度上限、角度控制参数

## 记录现场测试

`robot_follow.run_logger` 会为每次运行创建独立目录，保存控制台日志、CSV 数据、命令行和元数据：

```bash
conda run -n robot-follow python -m robot_follow.run_logger \
  --test-env jetson \
  --test-type closed_loop_low_speed \
  --logs-root logs \
  -- \
  --config configs/jetson_g1_safe.yaml \
  --dry-run
```

如果要执行真实低速闭环：

```bash
conda run -n robot-follow python -m robot_follow.run_logger \
  --test-env jetson \
  --test-type closed_loop_low_speed \
  --logs-root logs \
  -- \
  --config configs/jetson_g1_safe.yaml \
  --enable-motion \
  --i-understand-this-moves-robot
```

也可以使用脚本封装：

```bash
scripts/run_jetson_logged.sh
```

注意：`scripts/run_jetson_logged.sh` 是现场闭环测试脚本，默认会启用真实机器人运动。脚本内部会调用 `robot_follow.run_logger`，并自动传入：

```bash
--config configs/jetson_g1_safe.yaml \
--enable-motion \
--i-understand-this-moves-robot
```

常用覆盖方式：

```bash
# 指定单次运行时长，单位秒
DURATION=10 scripts/run_jetson_logged.sh

# 指定测试标签，日志目录会包含这些标签
TEST_ENV=jetson TEST_TYPE=closed_loop_low_speed scripts/run_jetson_logged.sh

# 指定配置文件
CONFIG=configs/jetson_g1_safe.yaml scripts/run_jetson_logged.sh

# 指定 conda 环境
CONDA_ENV=robot-follow scripts/run_jetson_logged.sh
```

脚本后面追加的参数会继续传给 `robot_follow.main`，例如临时覆盖串口或网卡：

```bash
DURATION=10 scripts/run_jetson_logged.sh --port /dev/ttyUSB1 --net-iface eth0
```

## 测试

运行全部测试：

```bash
conda run -n robot-follow pytest
```

运行最近 ALX 重连相关测试：

```bash
conda run -n robot-follow pytest tests/real/test_alx_source.py
```

核心逻辑测试集中在 `tests/core/`，仿真测试集中在 `tests/sim/`，真实适配层的无硬件单元测试集中在 `tests/real/`。

## 安全设计

项目的控制链路默认保守：

- 数据无效、串口错误、SDK 错误、数据超时都会输出 `stop`。
- 距离过近、距离跳变、角度跳变会触发安全停止。
- 目标过远时停止而不是高速追赶。
- dry-run 是默认行为，不会驱动真实机器人。
- G1 桥接层会再次限幅速度，并在 SDK 返回错误时进入错误状态。
- 真实运动需要显式双重参数确认。

## 参考文档

- [CH341_PERSISTENT_INSTALL.md](CH341_PERSISTENT_INSTALL.md)：Jetson Humble 背包上为 ALX/CH340 安装并开机自动加载 CH341 串口驱动
- [requirements-humble.txt](requirements-humble.txt)：本地 `robot_followao_radar_humble`（Python 3.10）可迁移的固定 Python 依赖；背包创建同名 Conda 环境的步骤见 `docs/ros2_humble_backpack_deployment.md`
- `docs/alx_ch340_usb_setup.md`：ALX/CH340 串口连接说明
- `docs/alx_wristband_gui_test_guide.md`：ALX 腕带 GUI 测试指南
- `docs/alx_g1_verification_chain.md`：ALX 到 G1 的验证链路
- `docs/g1_command_bridge_design.md`：G1 命令桥设计
- `docs/g1_jetson_robot_onsite_test_plan.md`：Jetson + G1 现场测试计划
- `docs/test/jetson_backpack_g1_test_runbook.md`：现场测试 runbook

## 当前状态

当前版本适合做 ALX dry-run、仿真验证和受控现场低速测试。真实机器人闭环仍应按测试计划逐步启用：先确认 ALX 数据稳定，再确认 G1 单项运动输出，最后再开启闭环跟随。

## LiDAR + ALX Assisted Follow Prototype

### 概述

`lidar_alx_assisted_follow` 是一个独立的跟随原型：ALX-G100 UWB 腕带的 body-frame 位置是唯一的跟随目标；Livox MID-360 LiDAR / FAST-LIO 点云只用于避障。

**核心设计原则：**
- 云驱动策略：每个点云序列最多处理一次；每次匹配一条不晚于该点云且时间差合格的 ALX 位置，随后完成跟随、避障和一次指令下发
- 40 Hz 安全监控：定时唤醒进行安全检查，不从旧点云重新计算策略
- 最近帧优先：多个点云到达时只处理最新帧
- 分离线程：Web 线程负责 WebSocket，控制线程负责策略计算和 G1 指令分发
- 安全优先：任何线程都可以请求 STOP，只有控制线程可以清除
- 临时校准：当前使用临时校准参数，未经过真实硬件验证

### 局部避障模式

`avoidance.mode` 是 LiDAR + ALX 主链路唯一的避障选择入口。目前只支持
`apf`，其参数位于相邻的 `apf` 配置段。运行时通过统一的
`AvoidanceInput -> AvoidanceResult` 接口调用避障策略；未来的滚动栅格 +
A* 模式会实现相同接口，而不会改动 ALX 匹配、硬停止、限速或 G1 指令分发。

`grid_astar` 目前尚未实现，配置为该模式会在加载配置时明确失败，绝不会悄悄
降级到 APF 后继续驱动机器人。

### 快速开始

#### Mock 模式（无硬件）

在一个已经 source ROS2 Foxy、FAST-LIO workspace 且设置相同 `ROS_DOMAIN_ID`
的终端中，直接启动辅助跟随主程序即可：

```bash
conda run -n robot_followao_radar robot-follow-lidar-alx-assisted-follow \
  configs/jetson_g1_lidar_alx_assisted_follow.yaml
```

辅助跟随程序订阅 `/alx/position` 和 `/alx/wristband_state`，不会再次打开
`/dev/ttyUSB0`；主程序托管的子进程是该串口唯一读取者。此命令使用模拟运动桥，
不会驱动真实机器人，适合验证控制逻辑和 Web 界面。手环 ON 启动 Livox/FAST-LIO；
OFF、状态 topic 超时或串口故障会停止雷达服务。ALX 约 21 Hz 位置进入有界时间
缓存；点云、决策和运动指令仍由点云以约 10 Hz 驱动，并拒绝时间差超过配置项
`alx.cloud_max_skew_s` 的 ALX 样本。

若 ALX 发布节点已由 systemd、launch 文件或另一个终端特意管理，可在主程序末尾
添加 `--external-alx-ros`，避免重复打开串口。

#### 真实 G1 运动条件

真实 G1 运动需要同时满足以下条件：

1. 配置文件中 `output.mode = "real"`
2. 使用 `--confirm-real-g1` 标志确认
3. 如果使用临时校准（`calibration.allow_provisional_calibration_for_prototype = true`），还需要 `--allow-provisional-calibration` 标志

```bash
conda run -n robot_followao_radar robot-follow-lidar-alx-assisted-follow \
  configs/jetson_g1_lidar_alx_assisted_follow.yaml \
  --confirm-real-g1 \
  --allow-provisional-calibration
```

#### Web 界面

默认 Web 服务器绑定到 `127.0.0.1:8080`：

```bash
# 在浏览器中打开
http://127.0.0.1:8080
```

**LAN 访问（需要确认）：**

如果需要从局域网其他设备访问：

```bash
conda run -n robot_followao_radar robot-follow-lidar-alx-assisted-follow \
  configs/jetson_g1_lidar_alx_assisted_follow.yaml \
  --host 0.0.0.0 \
  --allow-unauthenticated-lan
```

**⚠️ 警告：** `--allow-unauthenticated-lan` 会将控制接口暴露到局域网，任何人都可以连接并控制机器人。仅在可信的隔离网络中使用。

### 操作流程

1. **首个 WebSocket 连接成为控制器**
   - 浏览器连接后，第一个客户端自动成为控制器
   - 控制器断开后，新连接的客户端成为控制器
   - 观察者无法发送控制命令

2. **确认 ALX 目标**
   - 等待与新点云时间匹配的 ALX 位置帧
   - 系统将腕带位置设为 `CANDIDATE`（候选）

3. **开始跟随**
   - 点击 Start 按钮
   - 需要：ALX 候选目标、传感器就绪、ALX 新鲜且与该点云同步
   - 目标状态变为 `FOLLOWING`，开始发送运动指令

4. **停止跟随**
   - 点击 Stop 按钮，请求硬停止
   - ALX 目标保留为 `CANDIDATE`，继续零运动监控

### Jetson 部署准备

以下 Jetson 专用预检流程在实施期间**不会执行**。部署到 Jetson 前必须逐一验证：

1. 确认 `/etc/os-release` 为 Ubuntu 20.04，`uname -m` 为 `aarch64`
2. 确认 `ROS_DISTRO=foxy`，`RMW_IMPLEMENTATION` 已设置
3. 确认 `/usr/bin/python3 --version` 为 Python 3.8.x
4. 确认 conda 环境 `robot_followao_radar` 的 Python 也是 3.8.x
5. 运行导入检查：`rclpy`、`PointCloud2`、`sensor_msgs_py.point_cloud2`、`unitree_sdk2py`、`cyclonedds`
6. 确认 `/cloud_registered_body` 话题收到 `frame_id == "body"` 的点云
7. 确认 CycloneDDS 版本与 Unitree SDK 要求一致
8. 在真实 G1 运动前，先在 mock 模式下运行并通过 stop-only 测试

### 已知风险

- **1 秒 G1 指令持续时间：** G1 命令桥的 SDK 指令持续约 1 秒，不能立即中断
- **缺少物理速度反馈：** CSV 记录的 `dispatched_*` 是软件发出的指令速度，不是实际机器人速度
- **临时校准：** 当前所有校准参数标记为 `provisional`，未经过真实硬件验证
- **NumPy 非抢占性：** `max_strategy_process_ms` 超时无法抢占正在运行的 NumPy 计算
- **现场停止注入：** 需要在真实机器人上验证急停按钮和 Watchdog 超时机制

### 测试

```bash
# 运行新原型的所有 Python 测试
conda run -n robot_followao_radar pytest tests/lidar_alx_assisted_follow/ -v

# 运行前端检查
cd src/robot_follow/lidar_alx_assisted_follow/frontend
npm run typecheck
npm run build
```

### 残留风险

- 原型使用临时校准，未经过真实硬件验证
- 缺少物理速度反馈，CSV 记录的是软件指令速度
- G1 命令桥的 SDK 指令持续时间约 1 秒，不能立即中断
- 需要现场停止注入和校准验证 checklist
