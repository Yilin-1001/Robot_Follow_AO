# ALX-G100 Wristband GUI Test Guide

本文档用于用真实 ALX-G100 接收器和手环驱动 GUI sim，观察手环位置，并用现有 `core` pipeline 模拟机器人运动。

该测试不连接 Unitree G1 运动输出，不会让真实机器人运动。

## 1. 测试目标

确认链路：

```text
ALX-G100 手环
-> ALX-G100 接收器
-> /dev/ttyUSB0
-> ALXG100SerialReader
-> TargetSample
-> FollowPipeline
-> MotionCommand
-> GUI 模拟机器人运动
```

通过这个测试需要确认：

```text
真实手环位置能被稳定读取
GUI 能显示手环相对机器人位置
core 能根据真实 distance / azimuth 输出 move / stop
模拟机器人按 core 输出的 vx / omega 运动
异常数据、断流、过近距离会触发 stop
```

## 2. 前置条件

### 2.1 USB 串口已识别

确认：

```bash
groups
lsusb
lsusb -t
ls -l /dev/ttyUSB*
```

通过标准：

```text
groups 中包含 dialout
lsusb 中有 1a86:7523 QinHeng Electronics CH340 serial converter
lsusb -t 中 CH340 接口 Driver=ch341
/dev/ttyUSB0 存在
```

如果没有通过，先参考：

```text
docs/alx_ch340_usb_setup.md
```

### 2.2 串口参数

设置 ALX 串口：

```bash
stty -F /dev/ttyUSB0 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw -echo
```

确认：

```bash
stty -F /dev/ttyUSB0 -a
```

通过标准：

```text
speed 115200 baud
```

### 2.3 原始数据确认

运行：

```bash
timeout 5 xxd -g 1 /dev/ttyUSB0
```

通过标准：

```text
能看到 ff ff ff ff
能看到 20 01 定位帧
```

示例：

```text
ff ff ff ff 00 25 ... 20 01 ...
```

## 3. 启动 GUI 半实物模式

在项目目录执行：

```bash
cd /home/yilin/Code/Robot_Follow
conda run -n robot-follow python -m robot_follow.sim.GUI_sim.gui_app --source alx --port /dev/ttyUSB0
```

`--source alx` 默认使用 `rotate-only`，也就是：

```text
地图固定
ALX 接收器固定在 (0,0)，朝向 +x
手环位置按真实 ALX distance / azimuth 显示
模拟机器人位置固定在 (0,0)
模拟机器人只根据 core 输出的 omega 转向，不平移
```

如果串口不是 `/dev/ttyUSB0`，替换 `--port`：

```bash
conda run -n robot-follow python -m robot_follow.sim.GUI_sim.gui_app --source alx --port /dev/ttyUSB1
```

如果想显式指定只测试转向逻辑，也可以写：

```bash
conda run -n robot-follow python -m robot_follow.sim.GUI_sim.gui_app --source alx --port /dev/ttyUSB0 --motion-mode rotate-only
```

如果静止手环在图中看起来抖动明显，可以降低地图比例尺：

```bash
conda run -n robot-follow python -m robot_follow.sim.GUI_sim.gui_app --source alx --port /dev/ttyUSB0 --motion-mode rotate-only --pixels-per-meter 60
```

默认比例尺是：

```text
120 px/m
```

在 `800x600` 窗口中约等于显示 `6.7m x 5.0m` 的区域。使用 `60 px/m`
时约等于显示 `13.3m x 10.0m` 的区域，视觉上会降低 ALX 距离抖动造成的像素位移。

GUI 标题应显示：

```text
robot_follow GUI sim (alx input, core @ 40Hz)
```

终端应看到：

```text
EVENT alx_reader_started port=/dev/ttyUSB0
```

## 4. GUI 显示内容

GUI 中：

```text
蓝色圆点 = 模拟机器人
箭头 = 模拟机器人朝向
橙色点 = 根据真实 ALX distance / azimuth 在固定接收器坐标系中反算出的手环位置
扇形区域 = 模拟机器人前方检测范围
连线绿色 = 当前 TargetSample valid=True
连线红色 = 当前 TargetSample valid=False 或数据不可用
```

注意：

```text
ALX 接收器在 GUI 中被视为固定在世界原点、朝向 +x。
ALX 实测默认使用 rotate-only 模式，模拟机器人只应用 omega，忽略 vx，因此只会转向、不平移。
如果显式使用 full 模式，模拟机器人会根据 core 输出的 vx / omega 运动；该模式不推荐用于固定接收器的真实手环测试。
因此手环静止且 ALX distance / azimuth 稳定时，橙色点不应跟着模拟机器人漂移。
```

状态栏会显示：

```text
source=alx
motion mode
distance
azimuth
valid
command kind
reason
vx
omega
robot pose
alx status
fps
sample age
elevation
tag_status
anchor_id
tag_id
```

关键字段说明：

```text
distance      手环到接收器的距离，单位 m
azimuth       手环相对接收器正前方的方位角，单位 deg
elevation     俯仰角，当前只显示，不参与 core 控制
tag_status    ALX 标签状态，当前只显示
fps           最近 ALX 定位帧频率估计
sample age    最新定位帧距当前的时间
vx            core 输出的前后速度
omega         core 输出的转向角速度
reason        core 输出命令原因
```

`distance` / `azimuth` 是喂给 core 的模拟机器人视角相对观测。
`raw distance` / `raw azimuth` 是固定 ALX 接收器直接测得的原始观测。
在 `rotate-only` 模式下，手环静止时 raw 数据应基本稳定，core azimuth 会随着模拟机器人转向逐渐接近 0。

## 5. 测试动作

测试时不要连接 G1 运动控制。只观察 GUI 和终端日志。

### 5.1 正前方静止

动作：

```text
佩戴手环，站在接收器正前方约 1.5m
保持 10 秒不动
```

预期：

```text
distance 接近实际距离
azimuth 接近 0deg
GUI 中橙色点在模拟机器人前方
reason 可能为 holding 或 tracking_forward/backward
vx 接近 0 或较小
omega 接近 0
fps 稳定
sample age 较小
```

### 5.2 靠近和远离

动作：

```text
从约 2.0m 缓慢走到 1.5m
再从 1.5m 缓慢退到 2.0m
```

预期：

```text
靠近时 distance 变小
远离时 distance 变大
distance 连续变化，没有频繁大跳变
目标偏远时 core 输出 tracking_forward，vx > 0
目标在保持距离附近时 reason 变为 holding
目标过近但未小于安全距离时可能 tracking_backward，vx < 0
```

### 5.3 左右移动

动作：

```text
站在约 1.5m 距离
缓慢向左移动 0.5m
再缓慢向右移动 0.5m
```

预期：

```text
azimuth 随左右移动明显变化
左侧和右侧 azimuth 符号相反
GUI 中橙色点左右移动
omega 随 azimuth 变化
reason 可能为 tracking_turning 或 aligning
```

如果左右方向和预期相反：

```text
记录当前符号
后续校准 angle_gain 或坐标符号
不要在真实机器人闭环中直接试错
```

### 5.4 大角度测试

动作：

```text
站到接收器左前方或右前方，使 abs(azimuth) > 20deg
```

预期：

```text
reason = aligning
vx = 0
omega 非 0
模拟机器人主要原地转向
```

### 5.5 过近测试

动作：

```text
缓慢靠近到 0.5m 附近
不要快速冲到接收器前
```

预期：

```text
距离小于 safety 阈值时 command=stop
reason = too_close
vx = 0
omega = 0
模拟机器人停止
```

### 5.6 断流或遮挡测试

动作：

```text
短暂遮挡手环
或者让手环离开有效范围
或者关闭手环几秒
```

预期：

```text
sample age 增大
超过 stale timeout 后 reason = data_timeout 或 data_invalid
command=stop
连线变红或目标状态变 invalid
恢复信号后重新输出 live 数据
```

## 6. 键盘控制

GUI 支持：

```text
Space       暂停 / 继续
R           重置模拟机器人位姿和 pipeline filter
L           切换详细逐帧日志
Q / Escape  退出
```

建议每组动作前按 `R` 重置一次，避免上一组测试的 filter 状态影响观察。

## 7. 终端日志观察

正常运行会输出限频后的 `FRAME` 日志和事件日志。

重点看：

```text
distance
azimuth
valid
command
reason
vx
omega
robot
```

示例：

```text
FRAME ... distance=1.52m azimuth=3.0deg valid=True command=move reason=holding vx=0.000 omega=0.0000 ...
EVENT command_changed from=move/tracking_forward to=move/holding
```

如果按 `L` 打开 verbose，会打印更多逐帧日志。

## 8. 通过标准

本测试通过需要满足：

```text
GUI 能持续运行，不崩溃
状态栏显示 alx=live
fps 稳定且非 0
sample age 通常小于 100ms
distance 随靠近/远离正确变化
azimuth 随左右移动正确变化
core 输出的 reason 与动作一致
过近、断流、异常状态能触发 stop
模拟机器人运动由 core 的 vx / omega 驱动
```

## 9. 常见问题

### 9.1 GUI 启动后显示 waiting_for_position

说明串口打开了，但还没有收到定位帧。

检查：

```text
手环是否开机
手环和接收器是否配对
接收器是否有定位帧输出
是否只收到心跳帧
```

先回到原始数据测试：

```bash
timeout 5 xxd -g 1 /dev/ttyUSB0
```

### 9.2 GUI 显示 error

状态栏可能显示：

```text
alx = error: SerialException ...
```

检查：

```text
/dev/ttyUSB0 是否存在
当前用户是否在 dialout 组
是否有其他进程占用串口
```

命令：

```bash
ls -l /dev/ttyUSB0
groups
fuser /dev/ttyUSB0
```

### 9.3 数据明显延迟

如果 `age` 持续很大：

```text
检查串口数据是否断续
检查手环电量和接收器供电
确认没有其他程序同时读取 /dev/ttyUSB0
```

### 9.4 机器人不怎么转向

如果 `azimuth` 明显变化，但 `omega` 很小，说明当前 core 的角速度增益可能偏保守。

先记录：

```text
azimuth
omega
reason
```

不要直接进入真实 G1 闭环。后续应在 dry-run 中校准 `FollowConfig.angle_gain` 和 `max_omega`。

### 9.5 左右方向反了

如果人向左移动，GUI 或 `omega` 方向与预期相反：

```text
记录 ALX azimuth 的正负方向
确认接收器安装方向
后续通过配置调整符号
```

不要在真实机器人运动阶段临时试错。

## 10. 测试记录模板

```text
日期:
测试地点:
串口设备:
接收器安装方向:
手环 ID:

静止 1.5m:
  distance:
  azimuth:
  fps:
  age:
  reason:

靠近/远离:
  distance 变化是否正确:
  是否触发异常 stop:

左右移动:
  左侧 azimuth:
  右侧 azimuth:
  omega 方向是否符合预期:

过近:
  stop reason:

断流/遮挡:
  stop reason:
  恢复是否正常:

结论:
  通过 / 不通过
  需要调整:
```
