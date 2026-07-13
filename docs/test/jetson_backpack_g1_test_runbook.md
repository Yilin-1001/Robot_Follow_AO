# Jetson 背包端 G1 + ALX-G100 测试 Runbook

本文档用于指导在 Jetson 背包端进行 Unitree G1 + ALX-G100 跟随测试。
测试必须按阶段推进，不能直接进入真实闭环。

## 1. 测试原则

必须遵守：

```text
先验证 Jetson 环境
再验证 ALX-G100 串口和定位数据
再验证 G1 SDK 和单项运动
最后才进行 ALX + G1 低速闭环
```

真实运动测试必须满足：

```text
G1 原厂遥控器在线
安全员随时准备接管或急停
测试区域平整、空旷、无台阶和障碍物
机器人前方 3m 内没有无关人员
首次闭环使用极低速配置
每次测试都通过 run_logger 保存日志
```

真实运动命令必须显式包含：

```text
--enable-motion
--i-understand-this-moves-robot
```

没有这两个参数时，主程序只会使用 `MockMotionBridge`，不会控制真实 G1。

## 2. Jetson 基础检查

登录 Jetson：

```bash
ssh <JETSON_USER>@<JETSON_IP>
```

进入项目目录：

```bash
cd ~/Robot_Follow
```

确认系统和架构：

```bash
hostname
uname -m
lsb_release -a
```

预期：

```text
uname -m 输出 aarch64
系统为 Ubuntu 22.04 或 Jetson 当前使用的 Ubuntu 版本
```

确认 conda 环境：

```bash
conda env list
```

如果还没有 `robot-follow` 环境：

```bash
conda create -n robot-follow python=3.10 -y
conda run -n robot-follow pip install -e ".[dev]"
```

如果代码刚从本地同步到 Jetson，建议重新安装 editable 包：

```bash
conda run -n robot-follow pip install -e ".[dev]"
```

运行测试：

```bash
conda run -n robot-follow python -m pytest
```

确认主程序和日志脚本可用：

```bash
conda run -n robot-follow python -m robot_follow.main --help
conda run -n robot-follow python -m robot_follow.run_logger --help
```

## 3. 配置文件检查

现场调参优先修改：

```text
configs/jetson_g1_safe.yaml
```

检查配置：

```bash
sed -n '1,220p' configs/jetson_g1_safe.yaml
```

第一轮建议保持保守参数：

```yaml
g1:
  net_iface: enp2s0
  max_vx: 0.03
  max_omega: 0.10
  yaw_sign: 1

run:
  duration_s: 10
```

如果 Jetson 上 G1 网卡不是 `enp2s0`，修改：

```yaml
g1:
  net_iface: <ACTUAL_G1_NET_IFACE>
```

命令行参数会覆盖 YAML。例如临时覆盖网卡：

```bash
--net-iface eth0
```

## 4. ALX-G100 串口检查

插入 ALX-G100 接收器后检查：

```bash
groups
lsusb
lsusb -t
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

通过标准：

```text
groups 包含 dialout
lsusb 能看到 1a86:7523 QinHeng Electronics CH340 serial converter
lsusb -t 中 CH340 接口 Driver=ch341
出现 /dev/ttyUSB0 或其他 ttyUSB 设备
```

如果用户不在 `dialout` 组：

```bash
sudo usermod -aG dialout $USER
```

然后退出 SSH，重新登录。

设置串口参数：

```bash
stty -F /dev/ttyUSB0 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw -echo
stty -F /dev/ttyUSB0 -a
```

如果实际设备不是 `/dev/ttyUSB0`，替换成真实设备名。

查看原始帧：

```bash
timeout 5 xxd -g 1 /dev/ttyUSB0
```

通过标准：

```text
能看到 ff ff ff ff 帧头
能看到 20 01 定位帧
```

如果没有数据，按顺序检查：

```text
ALX-G100 接收器是否供电
手环是否开机
接收器和手环是否配对
USB 线是否支持数据
串口设备名是否正确
是否有其他进程占用串口
```

检查串口占用：

```bash
fuser /dev/ttyUSB0
```

## 5. ALX dry-run 测试

该阶段只读取 ALX-G100，不控制 G1。

运行：

```bash
cd ~/Robot_Follow

conda run -n robot-follow python -m robot_follow.run_logger \
  --test-env jetson \
  --test-type alx_dry_run \
  -- \
  --config configs/jetson_g1_safe.yaml \
  --port /dev/ttyUSB0 \
  --duration 30
```

如果实际串口不是 `/dev/ttyUSB0`，替换 `--port`。

观察终端输出：

```text
status=live
alx_fps=...
age=...ms
d=...m
az=...deg
cmd=...
vx=...
omega=...
```

通过标准：

```text
status 进入 live
alx_fps 接近 40
age 不持续升高
手环靠近时 d 变小
手环远离时 d 变大
手环左右移动时 az 有明显变化
没有持续 serial_error
没有持续 data_timeout
```

建议测试动作：

```text
站在接收器正前方 1.5m，保持 10 秒
退到 2.0m，保持 5 秒
靠近到 1.0m，保持 5 秒
靠近到 0.5m 内，确认 stop/too_close
左右移动，确认 azimuth_deg 和 omega 方向变化
```

日志目录：

```text
logs/runs/YYYYMMDD_HHMMSS_jetson_alx_dry_run/
```

目录内包含：

```text
*_data.csv
*_console.log
*_command.txt
*_metadata.json
```

如果 ALX dry-run 不稳定，不要进入 G1 运动测试。

## 6. G1 上电和人工准备

G1 不会自动进入可运动状态。必须由操作员通过遥控器完成站立和运动状态准备。

基本流程：

```text
Power on G1
等待约 1 分钟，进入 zero-torque state
按 L1+A，进入 damping state
扶住 G1 肩部并按 L1+UP，辅助进入 preparation state
按 R2+A，进入 motion state 并站立
```

如果使用 SDK 调试，按遥控器 `L2+R2` 进入 debug mode，避免内置运动控制和 SDK 命令冲突。

继续测试前确认：

```text
G1 站立稳定
遥控器在线
安全员可随时接管
测试区域空旷
Jetson 与 G1 通过以太网连接
```

确认 G1 网卡名：

```bash
ip addr
```

记录实际网卡，例如：

```text
G1_NET_IFACE=enp2s0
```

并同步到 `configs/jetson_g1_safe.yaml`。

## 7. G1 SDK 独立测试

安装 Unitree SDK 时必须使用 GitHub 源码版本：

```bash
cd /tmp
git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git
conda run -n robot-follow pip install -e /tmp/unitree_sdk2_python
```

验证 SDK import：

```bash
conda run -n robot-follow python -c "from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient; print('SDK OK')"
```

运行官方 example：

```bash
cd /tmp/unitree_sdk2_python
conda run -n robot-follow python example/g1/high_level/g1_loco_client_example.py enp2s0
```

如果实际网卡不是 `enp2s0`，替换成真实网卡。

在官方 example 中先做低风险测试，确认 G1 会响应并能停止。

## 8. G1 bridge 分阶段测试

以下命令不读取 ALX，不进入闭环，只验证 G1 motion bridge。

### 8.1 init-only

```bash
cd ~/Robot_Follow

conda run -n robot-follow python -m robot_follow.real.g1.check \
  --net-iface enp2s0 \
  --init-only \
  --i-understand-this-moves-robot
```

通过标准：

```text
SDK 初始化成功
程序退出无异常
G1 不移动
```

### 8.2 stop-only

```bash
conda run -n robot-follow python -m robot_follow.real.g1.check \
  --net-iface enp2s0 \
  --stop-only \
  --i-understand-this-moves-robot
```

通过标准：

```text
StopMove 成功
G1 保持静止
```

### 8.3 single-move

第一轮只做极低速短时前进：

```bash
conda run -n robot-follow python -m robot_follow.real.g1.check \
  --net-iface enp2s0 \
  --single-move \
  --vx 0.03 \
  --omega 0.0 \
  --duration 0.4 \
  --max-vx 0.05 \
  --max-omega 0.15 \
  --i-understand-this-moves-robot
```

通过标准：

```text
G1 低速前进不足 1 秒
随后停止
遥控器可接管
无 error code 日志
```

### 8.4 yaw_sign 校准

测试旋转方向：

```bash
conda run -n robot-follow python -m robot_follow.real.g1.check \
  --net-iface enp2s0 \
  --single-move \
  --vx 0.0 \
  --omega 0.10 \
  --duration 0.4 \
  --max-vx 0.05 \
  --max-omega 0.15 \
  --i-understand-this-moves-robot
```

如果旋转方向和预期相反，修改：

```yaml
g1:
  yaw_sign: -1
```

## 9. ALX + G1 低速闭环测试

只有以下阶段全部通过后，才执行闭环：

```text
ALX dry-run 稳定
G1 SDK import 成功
官方 example 成功
init-only 成功
stop-only 成功
single-move 成功
yaw_sign 已确认
遥控器接管确认
```

第一轮闭环命令：

```bash
cd ~/Robot_Follow

conda run -n robot-follow python -m robot_follow.run_logger \
  --test-env jetson \
  --test-type closed_loop_low_speed \
  -- \
  --config configs/jetson_g1_safe.yaml \
  --enable-motion \
  --i-understand-this-moves-robot
```

第一轮建议：

```text
duration_s = 10
max_vx = 0.03
max_omega = 0.10
target_distance_m = 1.5
```

测试动作：

```text
人站在 G1 正前方 1.5m 到 2.0m
保持稳定，观察 holding 或低速修正
缓慢后退到 2.0m，观察 tracking_forward
缓慢靠近到 1.0m，观察 tracking_backward
靠近到 0.5m 内，观察 stop/too_close
左右移动，观察 G1 转向方向
```

通过标准：

```text
G1 只做低速小动作
没有突然大速度
人靠太近时停止
ALX 数据丢失时停止
Ctrl+C 后程序 stop 并退出
遥控器可接管
日志完整保存
```

停止测试：

```text
Ctrl+C
```

预期：

```text
程序捕获中断
发送 stop/shutdown
G1 停止
日志目录完整生成
```

### 9.1 干净关停：避免 `conda run` 包裹长时间运行的进程

长时间运行的真实机器人进程（闭环、AO radar CLI）建议用 `conda activate` + 直接 `python` 启动，而非 `conda run`：

```bash
conda activate robot-follow
PYTHONPATH=src python -m robot_follow.run_logger ...   # 或 AO radar CLI
```

`conda run` 会在前台进程组里套一层 wrapper：Ctrl+C 时 wrapper 先捕获 SIGINT、打印 `CondaError: KeyboardInterrupt` 并退出，shell 提示符提前返回；而子进程 `python` 若正阻塞在 G1 `SetVelocity`（G1 不可达时最长阻塞 `g1.sdk_timeout_s` 秒，返回 3104），`KeyboardInterrupt` 会被推迟到该 C 调用返回，期间终端继续刷 `[WRISTBAND]` / `[AO_FRAME]` / `G1 stop timed out with code 3104`。直接用 `python` 启动让 SIGINT 直达进程，配合较短的 `sdk_timeout_s`，关停更及时安静。

若提示符已回来却仍在刷屏，说明 `python` 子进程被孤儿化但还占用 TTY：`pkill -f robot_follow`（或按 PID）结束即可。

## 10. 参数调整方法

优先修改：

```text
configs/jetson_g1_safe.yaml
```

常调参数：

```text
g1.max_vx
g1.max_omega
g1.yaw_sign
alx.stale_timeout_s
alx.hz
filter.alpha
safety.min_safe_distance_m
safety.max_distance_jump_m
safety.max_angle_jump_deg
follow.target_distance_m
follow.hold_tolerance_m
follow.max_forward_vx
follow.max_backward_vx
follow.angle_deadzone_deg
follow.large_angle_deg
follow.distance_gain_forward
follow.distance_gain_backward
follow.angle_gain
```

临时试参可以用 CLI 覆盖：

```bash
conda run -n robot-follow python -m robot_follow.run_logger \
  --test-env jetson \
  --test-type closed_loop_low_speed \
  -- \
  --config configs/jetson_g1_safe.yaml \
  --max-vx 0.02 \
  --max-omega 0.08 \
  --enable-motion \
  --i-understand-this-moves-robot
```

确认参数有效后，把结果写回 YAML。

不建议现场直接修改 Python 文件。只有要修改控制逻辑本身时，才改 `src/robot_follow/`。

## 11. 日志回传

日志默认保存在：

```text
logs/runs/YYYYMMDD_HHMMSS_<test-env>_<test-type>/
```

回传到本地：

```bash
rsync -av \
  jetson:~/Robot_Follow/logs/runs/ \
  /home/yilin/Code/Robot_Follow/logs/runs/
```

回传参数快照：

```bash
rsync -av \
  jetson:~/Robot_Follow/configs/ \
  /home/yilin/Code/Robot_Follow/configs/
```

不要用 `--delete` 回传日志，避免删除本地已有测试记录。

## 12. 常见失败排查

### 12.1 没有 `/dev/ttyUSB0`

检查：

```bash
lsusb
lsusb -t
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
dmesg | tail -80
```

如果 `lsusb` 能看到 `1a86:7523` 但没有 `/dev/ttyUSB0`，检查 `brltty` 是否抢占 CH340。

### 12.2 ALX 一直 `waiting_for_position`

排查：

```text
手环是否开机
接收器和手环是否配对
手环是否离接收器太近或遮挡
串口设备名是否正确
原始 xxd 是否有 ff ff ff ff 和 20 01
```

### 12.3 ALX 进入 `live` 但频繁 stop

查看 `command_reason`：

```text
too_close          距离小于 safety.min_safe_distance_m
distance_jump      距离跳变超过 safety.max_distance_jump_m
angle_jump         角度跳变超过 safety.max_angle_jump_deg
data_timeout       数据年龄超过 safety.data_timeout_s
target_too_far     距离超过 follow.max_follow_distance_m
```

### 12.4 G1 SDK import 失败

确认是否安装 GitHub 源码版本：

```bash
conda run -n robot-follow python -c "import unitree_sdk2py; print(unitree_sdk2py)"
```

必要时重装：

```bash
cd /tmp
git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git
conda run -n robot-follow pip install -e /tmp/unitree_sdk2_python
```

### 12.5 G1 不响应

检查：

```text
G1 是否已经进入 motion state
Jetson 与 G1 以太网是否连接
g1.net_iface 是否正确
遥控器是否在线
是否进入 debug mode
是否有其他程序占用控制链路
```

### 12.6 出现 `SetVelocity returned error code`

常见原因：

```text
G1 未进入 motion state
G1 SDK 链路不通
网卡名错误
机器人状态不允许运动命令
```

处理：

```text
不要继续闭环
重新做 G1 手动准备
重新跑 init-only / stop-only / single-move
```

### 12.7 转向方向反了

修改：

```yaml
g1:
  yaw_sign: -1
```

然后重新运行 ALX dry-run 或低速闭环。

