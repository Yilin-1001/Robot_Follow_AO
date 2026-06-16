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

如果已经在仓库根目录，也可以直接使用 Python 模块入口；这种方式适合开发调试：

```bash
conda run -n robot-follow python -m robot_follow.main --config configs/jetson_g1_safe.yaml --dry-run
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

- `docs/alx_ch340_usb_setup.md`：ALX/CH340 串口连接说明
- `docs/alx_wristband_gui_test_guide.md`：ALX 腕带 GUI 测试指南
- `docs/alx_g1_verification_chain.md`：ALX 到 G1 的验证链路
- `docs/g1_command_bridge_design.md`：G1 命令桥设计
- `docs/g1_jetson_robot_onsite_test_plan.md`：Jetson + G1 现场测试计划
- `docs/test/jetson_backpack_g1_test_runbook.md`：现场测试 runbook

## 当前状态

当前版本适合做 ALX dry-run、仿真验证和受控现场低速测试。真实机器人闭环仍应按测试计划逐步启用：先确认 ALX 数据稳定，再确认 G1 单项运动输出，最后再开启闭环跟随。
