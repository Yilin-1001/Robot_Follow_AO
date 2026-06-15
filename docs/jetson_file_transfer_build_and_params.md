# Jetson 文件传输、构建与参数更新技术文档

本文档说明本地 Ubuntu 开发环境与 Jetson 边缘端之间如何进行代码部署、数据回传、日志同步和参数更新。目标是让现场机器人测试过程可重复、可追踪，并避免本地代码和 Jetson 代码互相覆盖。

适用项目：

```text
/home/yilin/Code/Robot_Follow
```

推荐原则：

```text
本地开发机是代码主环境
Jetson 是运行和现场测试环境
本地 -> Jetson：同步代码和基础配置
Jetson -> 本地：回传日志、CSV、参数快照和现场结果
不要双向同步整个项目目录
```

## 1. 角色分工

### 1.1 本地 Ubuntu 开发机

本地开发机负责：

```text
代码开发
单元测试
版本管理
部署到 Jetson
接收 Jetson 回传的日志和参数
复盘现场测试结果
```

本地项目路径：

```bash
/home/yilin/Code/Robot_Follow
```

### 1.2 Jetson 边缘端

Jetson 负责：

```text
运行机器人程序
连接 ALX-G100 串口
连接 Unitree G1 网络接口
记录现场日志和 CSV
临时调整参数
生成每次测试的参数快照
```

推荐 Jetson 项目路径：

```bash
~/Robot_Follow
```

## 2. SSH 准备

本地通过 SSH 连接 Jetson。建议在本地 `~/.ssh/config` 中配置别名：

```sshconfig
Host jetson
  HostName 192.168.x.x
  User fk
```

其中：

```text
192.168.x.x 替换为 Jetson 的实际 IP
fk 替换为 Jetson 上的实际用户名
```

验证 SSH：

```bash
ssh jetson 'hostname && uname -m'
```

预期：

```text
能正常登录
uname -m 输出 aarch64
```

## 3. rsync 基本说明

`rsync` 是通过 SSH 进行文件同步的工具。它会比较两边文件差异，只传输发生变化的文件，适合频繁部署代码到 Jetson。

基本格式：

```bash
rsync -av 本地目录/ jetson:远端目录/
```

常用参数：

```text
-a          归档模式，保留目录结构、权限和时间戳
-v          显示同步过程
--delete    删除目标端多余文件，使目标端与源端一致
--exclude   排除不需要同步的文件或目录
```

注意：

```text
本地 -> Jetson 同步代码时可以使用 --delete
Jetson -> 本地 回传日志和参数时不要使用 --delete
```

原因是日志和参数属于现场数据，误用 `--delete` 可能删除本地已有测试记录。

## 4. 推荐目录结构

推荐在项目中使用以下目录：

```text
Robot_Follow/
  src/
    robot_follow/
  tests/
  docs/
  configs/
    jetson_g1_tuned.yaml
    runs/
      2026-06-15_1530.yaml
      2026-06-15_1600.yaml
  logs/
    alx_dry_run_001.csv
    alx_tuned_001.csv
```

目录职责：

```text
src/                 项目源码
tests/               本地和 Jetson 单元测试
docs/                技术文档
configs/             可复用参数配置
configs/runs/        每次现场测试的参数快照
logs/                现场运行日志、CSV 和测试结果
```

如果当前仓库还没有 `configs/` 和 `logs/`，可以在本地或 Jetson 上创建：

```bash
mkdir -p configs/runs logs
```

## 5. 本地构建与测试

部署到 Jetson 前，先在本地确认代码可运行：

```bash
cd /home/yilin/Code/Robot_Follow
python3 -m pip install -e ".[dev]"
pytest -q
```

通过标准：

```text
pytest 通过
没有 import 错误
没有语法错误
```

如果本地测试失败，不要部署到 Jetson，应先在本地修复。

## 6. 本地代码同步到 Jetson

### 6.1 首次同步

从本地开发机执行：

```bash
rsync -av --delete \
  --exclude .git \
  --exclude __pycache__ \
  --exclude .pytest_cache \
  --exclude "*.egg-info" \
  --exclude logs \
  --exclude configs/runs \
  /home/yilin/Code/Robot_Follow/ \
  jetson:~/Robot_Follow/
```

说明：

```text
--delete 让 Jetson 上的代码和本地保持一致
--exclude logs 避免删除 Jetson 上的现场日志
--exclude configs/runs 避免删除 Jetson 上的参数快照
```

### 6.2 高频迭代同步

本地修改代码后继续使用同一条命令：

```bash
rsync -av --delete \
  --exclude .git \
  --exclude __pycache__ \
  --exclude .pytest_cache \
  --exclude "*.egg-info" \
  --exclude logs \
  --exclude configs/runs \
  /home/yilin/Code/Robot_Follow/ \
  jetson:~/Robot_Follow/
```

同步后在 Jetson 上测试：

```bash
ssh jetson 'cd ~/Robot_Follow && conda run -n robot-follow python -m pytest'
```

如果修改了依赖、入口点或包结构，重新安装 editable 包：

```bash
ssh jetson 'cd ~/Robot_Follow && conda run -n robot-follow pip install -e ".[dev]"'
```

## 7. Jetson 环境构建

在 Jetson 上执行：

```bash
cd ~/Robot_Follow
conda create -n robot-follow python=3.10 -y
conda activate robot-follow
pip install -e ".[dev]"
```

如果只测试 ALX-G100 串口和 dry-run，不需要安装 Unitree SDK。

如果要连接 Unitree G1，按项目现场测试计划安装 `unitree_sdk2_python`：

```bash
cd /tmp
git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git
conda run -n robot-follow pip install -e /tmp/unitree_sdk2_python
```

验证：

```bash
cd ~/Robot_Follow
conda run -n robot-follow python -m pytest
conda run -n robot-follow python -m robot_follow.main --help
```

## 8. Jetson 运行与日志生成

### 8.1 ALX dry-run

只读取 ALX-G100，不控制机器人：

```bash
cd ~/Robot_Follow
mkdir -p logs
conda run -n robot-follow python -m robot_follow.run_logger \
  --test-env jetson \
  --test-type alx_dry_run \
  -- \
  --config configs/jetson_g1_safe.yaml \
  --duration 20
```

日志会自动写入：

```text
logs/runs/YYYYMMDD_HHMMSS_jetson_alx_dry_run/
```

这个目录就是单次测试的日志包，里面包含：

```text
YYYYMMDD_HHMMSS_jetson_alx_dry_run_data.csv
YYYYMMDD_HHMMSS_jetson_alx_dry_run_console.log
YYYYMMDD_HHMMSS_jetson_alx_dry_run_command.txt
YYYYMMDD_HHMMSS_jetson_alx_dry_run_metadata.json
```

如果需要单个归档文件用于传输或分享，可以在 `run_logger` 命令上加
`--zip`，额外生成：

```text
logs/YYYYMMDD_HHMMSS_jetson_alx_dry_run.zip
```

CSV 内容通常包括：

```text
distance_m
azimuth_deg
target_x_m
target_y_m
command_kind
command_reason
vx
omega
```

这些 CSV 用于复盘算法输出是否合理。

真实闭环测试也使用同一个记录入口，但必须显式加运动许可：

```bash
conda run -n robot-follow python -m robot_follow.run_logger \
  --test-env jetson \
  --test-type closed_loop_low_speed \
  -- \
  --config configs/jetson_g1_safe.yaml \
  --enable-motion \
  --i-understand-this-moves-robot
```

### 8.2 日志命名建议

`robot_follow.run_logger` 会自动生成带时间和测试目的的文件名。手动命名时也建议沿用：

```text
logs/2026-06-15_1530_alx_dry_run.csv
logs/2026-06-15_1600_low_speed_follow.csv
logs/2026-06-15_1630_turning_test.csv
```

不要反复覆盖同一个日志文件，否则后续无法比较不同参数的效果。

## 9. Jetson 日志回传到本地

从本地开发机执行：

```bash
rsync -av \
  jetson:~/Robot_Follow/logs/runs/ \
  /home/yilin/Code/Robot_Follow/logs/runs/
```

注意：

```text
回传日志不要使用 --delete
```

原因：

```text
Jetson 上可能只保存一部分日志
本地可能已经保存了历史日志
使用 --delete 可能删除本地已有复盘数据
```

## 10. 参数更新策略

参数更新分为两种方式：

```text
推荐方式：使用 configs/*.yaml 配置文件
备用方式：临时 CLI 参数覆盖 YAML
不推荐方式：现场直接修改 Python 代码
```

### 10.1 推荐方式：修改 YAML 配置

主程序支持通过 `--config` 读取 YAML：

```bash
conda run -n robot-follow python -m robot_follow.main \
  --config configs/jetson_g1_safe.yaml \
  --duration 10 \
  --csv logs/closed_loop_001.csv
```

如果要真实控制 G1，仍必须显式加上安全开关：

```bash
conda run -n robot-follow python -m robot_follow.main \
  --config configs/jetson_g1_safe.yaml \
  --enable-motion \
  --i-understand-this-moves-robot
```

命令行参数优先级高于 YAML。例如临时试一轮更低速度：

```bash
conda run -n robot-follow python -m robot_follow.main \
  --config configs/jetson_g1_safe.yaml \
  --max-vx 0.02 \
  --max-omega 0.08 \
  --duration 10
```

确认参数有效后，把结果写回 YAML，避免只留在 shell 历史里。

### 10.2 可调参数位置

推荐配置文件：

```text
configs/jetson_g1_safe.yaml
```

主程序支持以下配置段：

```text
g1       G1 网络接口、速度上限、yaw_sign
alx      串口、波特率、控制频率、数据超时
run      运行时长、打印频率、CSV 路径
filter   ALX 低通滤波 alpha
safety   安全阈值
follow   跟随控制器参数
```

核心跟随参数示例：

```yaml
follow:
  target_distance_m: 1.5
  min_safe_distance_m: 0.5
  hold_tolerance_m: 0.15
  max_forward_vx: 0.20
  max_backward_vx: 0.12
  max_follow_distance_m: 4.0
  max_omega: 0.60
  angle_deadzone_deg: 7.0
  large_angle_deg: 20.0
  distance_gain_forward: 0.18
  distance_gain_backward: 0.15
  angle_gain: 0.5
```

这些字段对应代码里的默认值：

```text
src/robot_follow/core/follow_controller.py
```

但现场测试不应直接改这个 Python 文件，除非要修改控制逻辑本身。

旧版代码中的默认值示例：

```text
target_distance_m = 1.5
min_safe_distance_m = 0.5
hold_tolerance_m = 0.15
max_forward_vx = 0.20
max_backward_vx = 0.12
max_follow_distance_m = 4.0
max_omega = 0.60
angle_deadzone_deg = 7.0
large_angle_deg = 20.0
distance_gain_forward = 0.18
distance_gain_backward = 0.15
angle_gain = 0.5
```

如果确实临时修改了 Python，测试结束后必须把文件回传到本地：

```bash
rsync -av \
  jetson:~/Robot_Follow/src/robot_follow/core/follow_controller.py \
  /home/yilin/Code/Robot_Follow/src/robot_follow/core/follow_controller.py
```

回传后在本地检查差异：

```bash
cd /home/yilin/Code/Robot_Follow
git diff src/robot_follow/core/follow_controller.py
pytest -q
```

这种方式只适合修改逻辑，不适合普通调参。

### 10.3 参数快照

推荐把现场参数保存为独立配置文件，例如：

```text
configs/jetson_g1_tuned.yaml
```

示例内容：

```yaml
follow:
  target_distance_m: 1.5
  min_safe_distance_m: 0.5
  hold_tolerance_m: 0.15
  max_forward_vx: 0.18
  max_backward_vx: 0.10
  max_follow_distance_m: 4.0
  max_omega: 0.45
  angle_deadzone_deg: 8.0
  large_angle_deg: 22.0
  distance_gain_forward: 0.16
  distance_gain_backward: 0.12
  angle_gain: 0.45
```

每次测试再生成参数快照：

```text
configs/runs/2026-06-15_1530.yaml
```

示例内容：

```yaml
test_id: 2026-06-15_1530
robot: unitree_g1
platform: jetson_orin
alx_port: /dev/ttyUSB0
log_file: logs/2026-06-15_1530_alx_dry_run.csv
notes: indoor low-speed dry-run

follow:
  target_distance_m: 1.5
  min_safe_distance_m: 0.5
  hold_tolerance_m: 0.15
  max_forward_vx: 0.18
  max_backward_vx: 0.10
  max_follow_distance_m: 4.0
  max_omega: 0.45
  angle_deadzone_deg: 8.0
  large_angle_deg: 22.0
  distance_gain_forward: 0.16
  distance_gain_backward: 0.12
  angle_gain: 0.45
```

好处：

```text
代码逻辑和参数分离
每次测试能复现
日志和参数能一一对应
本地复盘更清楚
不会因为同步代码误删现场参数
```

## 11. 参数从 Jetson 回传到本地

### 11.1 回传所有参数快照

从本地开发机执行：

```bash
rsync -av \
  jetson:~/Robot_Follow/configs/runs/ \
  /home/yilin/Code/Robot_Follow/configs/runs/
```

### 11.2 回传当前调好的参数文件

如果现场维护的是当前最佳参数：

```text
configs/jetson_g1_tuned.yaml
```

回传：

```bash
rsync -av \
  jetson:~/Robot_Follow/configs/jetson_g1_tuned.yaml \
  /home/yilin/Code/Robot_Follow/configs/jetson_g1_tuned.yaml
```

回传后本地检查：

```bash
cd /home/yilin/Code/Robot_Follow
git diff configs/
```

## 12. 本地参数更新到 Jetson

如果参数已经在本地整理完成，需要推送到 Jetson：

```bash
rsync -av \
  /home/yilin/Code/Robot_Follow/configs/jetson_g1_tuned.yaml \
  jetson:~/Robot_Follow/configs/jetson_g1_tuned.yaml
```

如果要推送整个 `configs/` 目录：

```bash
rsync -av \
  /home/yilin/Code/Robot_Follow/configs/ \
  jetson:~/Robot_Follow/configs/
```

注意：

```text
推送 configs/ 时不要使用 --delete
除非你明确知道 Jetson 上没有需要保留的现场参数快照
```

## 13. 防冲突规则

### 13.1 代码同步规则

代码以本地为主：

```text
本地 -> Jetson
```

推荐命令带 `--delete`，但必须排除现场数据：

```bash
rsync -av --delete \
  --exclude .git \
  --exclude __pycache__ \
  --exclude .pytest_cache \
  --exclude "*.egg-info" \
  --exclude logs \
  --exclude configs/runs \
  /home/yilin/Code/Robot_Follow/ \
  jetson:~/Robot_Follow/
```

### 13.2 日志同步规则

日志以 Jetson 为主：

```text
Jetson -> 本地
```

命令不要带 `--delete`：

```bash
rsync -av jetson:~/Robot_Follow/logs/ /home/yilin/Code/Robot_Follow/logs/
```

### 13.3 参数同步规则

参数必须明确谁是主：

```text
现场正在测试：Jetson 参数是主
本地复盘完成：本地参数是主
```

不要两边同时修改同一个参数文件。

更稳妥的方式是每次测试生成新参数快照：

```text
configs/runs/2026-06-15_1530.yaml
configs/runs/2026-06-15_1600.yaml
configs/runs/2026-06-15_1630.yaml
```

这样回传时不会覆盖已有结果。

## 14. 推荐完整现场流程

### 14.1 本地开发后部署

```bash
cd /home/yilin/Code/Robot_Follow
pytest -q

rsync -av --delete \
  --exclude .git \
  --exclude __pycache__ \
  --exclude .pytest_cache \
  --exclude "*.egg-info" \
  --exclude logs \
  --exclude configs/runs \
  /home/yilin/Code/Robot_Follow/ \
  jetson:~/Robot_Follow/
```

### 14.2 Jetson 构建验证

```bash
ssh jetson 'cd ~/Robot_Follow && conda run -n robot-follow pip install -e ".[dev]"'
ssh jetson 'cd ~/Robot_Follow && conda run -n robot-follow python -m pytest'
```

### 14.3 Jetson 运行测试

```bash
ssh jetson 'cd ~/Robot_Follow && mkdir -p logs configs/runs'

ssh jetson 'cd ~/Robot_Follow && conda run -n robot-follow python -m robot_follow.main \
  --port /dev/ttyUSB0 \
  --duration 20 \
  --csv logs/2026-06-15_1530_alx_dry_run.csv'
```

### 14.4 回传日志和参数

```bash
rsync -av jetson:~/Robot_Follow/logs/ /home/yilin/Code/Robot_Follow/logs/
rsync -av jetson:~/Robot_Follow/configs/runs/ /home/yilin/Code/Robot_Follow/configs/runs/
```

如果现场修改了当前参数文件：

```bash
rsync -av \
  jetson:~/Robot_Follow/configs/jetson_g1_tuned.yaml \
  /home/yilin/Code/Robot_Follow/configs/jetson_g1_tuned.yaml
```

### 14.5 本地复盘和提交

```bash
cd /home/yilin/Code/Robot_Follow
git status
git diff
pytest -q
```

确认现场结果有效后再提交：

```bash
git add configs logs docs src tests
git commit -m "record jetson onsite tuning results"
```

如果日志文件很大，不建议全部提交到 Git；可以只提交参数、摘要和关键 CSV。

## 15. 常见问题

### 15.1 rsync 会不会和 Git 冲突

不会直接冲突。`rsync` 负责传文件，Git 负责版本管理。

推荐方式：

```text
本地用 Git 管代码
部署用 rsync
现场数据回传也用 rsync
```

Jetson 上不一定需要 Git。

### 15.2 双向 rsync 会不会冲突

如果双向同步整个项目目录，会有冲突风险。

正确做法：

```text
本地 -> Jetson：只推代码
Jetson -> 本地：只拉日志和参数
```

不要执行：

```text
Jetson 整个 Robot_Follow/ 反向覆盖本地 Robot_Follow/
```

### 15.3 参数是否可以只写在日志里

不推荐。

日志适合记录结果：

```text
距离
角度
速度输出
停止原因
时间戳
```

参数应单独保存为配置文件或参数快照：

```text
target_distance_m
max_forward_vx
max_omega
angle_gain
```

最佳实践是：

```text
一份参数快照对应一份日志文件
```

### 15.4 什么时候使用 --delete

只在本地代码部署到 Jetson 时使用：

```text
本地代码 -> Jetson 代码
```

不要在以下场景使用：

```text
Jetson 日志 -> 本地
Jetson 参数快照 -> 本地
本地 configs/ -> Jetson configs/
```

除非你明确知道目标端数据可以被删除。

## 16. 最小命令清单

本地推代码：

```bash
rsync -av --delete \
  --exclude .git \
  --exclude __pycache__ \
  --exclude .pytest_cache \
  --exclude "*.egg-info" \
  --exclude logs \
  --exclude configs/runs \
  /home/yilin/Code/Robot_Follow/ \
  jetson:~/Robot_Follow/
```

Jetson 跑测试：

```bash
ssh jetson 'cd ~/Robot_Follow && conda run -n robot-follow python -m robot_follow.main --port /dev/ttyUSB0 --duration 20 --csv logs/alx_dry_run.csv'
```

拉日志：

```bash
rsync -av jetson:~/Robot_Follow/logs/ /home/yilin/Code/Robot_Follow/logs/
```

拉参数快照：

```bash
rsync -av jetson:~/Robot_Follow/configs/runs/ /home/yilin/Code/Robot_Follow/configs/runs/
```

拉当前最佳参数：

```bash
rsync -av jetson:~/Robot_Follow/configs/jetson_g1_tuned.yaml /home/yilin/Code/Robot_Follow/configs/jetson_g1_tuned.yaml
```
