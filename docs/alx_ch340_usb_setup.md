# ALX-G100 CH340 USB Serial Setup

本文档用于在 Jetson 或 Ubuntu 机器上确认 ALX-G100 接收器是否被正确识别为 USB 串口，并确认能够收到 ALX 二进制定位帧。

适用场景：

```text
lsusb 能看到 1a86:7523 CH340
但 /dev/ttyUSB0 没有出现
或者 /dev/ttyUSB0 出现后读不到数据
```

## 1. 预期链路

ALX-G100 接收器通过 CH340 USB-串口芯片接入系统后，正常链路应为：

```text
ALX-G100 接收器
-> CH340 USB serial converter
-> Linux ch341 驱动
-> /dev/ttyUSB0
-> 115200 baud raw binary stream
-> ff ff ff ff ... 20 01 ... 定位帧
```

通过标准：

```text
lsusb 能看到 1a86:7523
lsusb -t 中该接口 Driver=ch341
/dev/ttyUSB0 存在
xxd 能看到 ff ff ff ff 帧头
```

## 2. 插入设备前检查

先查看当前串口设备：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

如果没有输出是正常的。

插入 ALX-G100 接收器后执行：

```bash
lsusb
lsusb -t
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

正常情况下，`lsusb` 应看到类似：

```text
ID 1a86:7523 QinHeng Electronics CH340 serial converter
```

`lsusb -t` 应看到类似：

```text
Driver=ch341
```

并且出现：

```text
/dev/ttyUSB0
```

## 3. 处理 brltty 抢占 CH340

Ubuntu/Jetson 上可能出现 `brltty` 抢占 CH340 的问题。表现是：

```text
lsusb 能看到 1a86:7523
lsusb -t 显示 Driver=usbfs 或 Driver 为空
/dev/ttyUSB0 没有出现
systemctl status brltty-udev.service 显示 active
brltty 日志里有 Ignored Byte
```

处理方法：

```bash
sudo systemctl stop brltty-udev.service
sudo systemctl mask brltty-udev.service
sudo systemctl stop brltty.service
sudo systemctl disable brltty.service
```

然后拔掉 ALX-G100 USB，重新插入。

重新检查：

```bash
lsusb -t
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

通过标准：

```text
CH340 接口 Driver=ch341
/dev/ttyUSB0 出现
```

如果仍然没有出现 `/dev/ttyUSB0`，检查驱动是否加载：

```bash
lsmod | grep -E 'ch341|usbserial'
sudo modprobe ch341
sudo modprobe usbserial
```

再次拔插设备。

## 4. 串口权限

查看设备权限：

```bash
ls -l /dev/ttyUSB0
groups
```

常见输出：

```text
crw-rw---- 1 root dialout ... /dev/ttyUSB0
```

如果当前用户不在 `dialout` 组，会读写失败。

临时测试：

```bash
sudo chmod 666 /dev/ttyUSB0
```

永久配置：

```bash
sudo usermod -aG dialout $USER
```

然后退出 SSH/终端，重新登录。

注意：`chmod 666` 只适合临时测试，设备重插后可能失效。

重启或重新登录后，先确认当前用户是否已经在 `dialout` 组：

```bash
groups
```

通过标准：

```text
输出中包含 dialout
```

如果已经包含 `dialout`，后续即使 `/dev/ttyUSB0` 权限恢复为
`root:dialout 660`，当前用户也应能直接读取串口，不需要再次
`chmod 666`。

## 5. 设置串口参数

ALX-G100 当前按 115200 baud 读取。先设置为 115200、8N1、raw 模式：

```bash
stty -F /dev/ttyUSB0 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw -echo
```

确认参数：

```bash
stty -F /dev/ttyUSB0 -a
```

通过标准：

```text
speed 115200 baud
cs8
-parenb
-cstopb
-crtscts
raw 相关选项已生效
```

如果保持默认 `9600 baud`，可能读不到有效数据。

## 6. 确认原始 ALX 帧

读取 5 秒原始数据：

```bash
timeout 5 xxd -g 1 /dev/ttyUSB0
```

通过标准：

```text
能看到 ff ff ff ff 帧头
能看到 20 01 定位帧命令字
```

示例：

```text
ff ff ff ff 00 25 ... 20 01 ...
```

字段含义：

```text
ff ff ff ff = ALX 帧头
20 01       = 0x2001 定位帧
20 02       = 0x2002 心跳帧
```

如果 `xxd` 没有任何输出，按顺序检查：

```text
1. 串口是否仍然是 /dev/ttyUSB0
2. stty 是否已经设置 115200
3. ALX 接收器是否供电
4. 手环是否开机
5. 接收器和手环是否配对
6. USB 线是否支持数据
7. brltty 是否重新启动
8. 是否有其他进程占用串口
```

检查占用：

```bash
fuser /dev/ttyUSB0
```

## 7. 用项目解析器读取

确认能看到原始帧后，再使用项目解析器读取。推荐先只做 ALX 数据验证，不连接 G1 运动控制。

当前仓库已有：

```text
robot_follow.serial_reader.ALXG100SerialReader
robot_follow.alx_g100_protocol.parse_frame
```

预期解析结果应包含：

```text
distance_m
azimuth_deg
elevation_deg
tag_status
timestamp
```

验证动作：

```text
手环靠近基站，distance_m 应变小
手环远离基站，distance_m 应变大
手环左右移动，azimuth_deg 应变化
静止 30 秒，distance_m 和 azimuth_deg 不应持续大跳变
```

## 8. 常见状态判断

### 8.1 lsusb 看不到 1a86:7523

说明 USB 层没有识别设备。

检查：

```text
USB 线是否支持数据
ALX 接收器是否供电
USB 口是否可用
换 USB 口或 USB 线
```

重启系统或重新插拔设备后，如果执行：

```bash
groups
lsusb -t
ls -l /dev/ttyUSB*
```

其中 `groups` 已经包含 `dialout`，但 `lsusb` / `lsusb -t` 看不到
`1a86:7523` 或 CH340 设备，并且没有 `/dev/ttyUSB0`，这不是权限问题。
应优先按物理枚举问题处理：

```text
1. 拔掉 ALX 接收器，等待 2 秒，再插回
2. 换一个 USB 口
3. 确认 USB 线支持数据
4. 确认 ALX 接收器供电
5. 再执行 lsusb、lsusb -t、ls -l /dev/ttyUSB*
```

通过标准：

```text
lsusb 出现 1a86:7523 QinHeng Electronics CH340 serial converter
lsusb -t 出现 Driver=ch341
/dev/ttyUSB0 出现
```

### 8.2 lsusb 有 1a86:7523，但没有 /dev/ttyUSB0

重点检查：

```bash
lsusb -t
systemctl status brltty-udev.service
lsmod | grep -E 'ch341|usbserial'
```

如果 `Driver=usbfs`，优先处理 `brltty` 抢占。

### 8.3 /dev/ttyUSB0 有，但 Permission denied

临时处理：

```bash
sudo chmod 666 /dev/ttyUSB0
```

永久处理：

```bash
sudo usermod -aG dialout $USER
```

重新登录后再测。

### 8.4 /dev/ttyUSB0 有，但 xxd 没输出

检查：

```bash
stty -F /dev/ttyUSB0 -a
fuser /dev/ttyUSB0
```

然后重新设置：

```bash
stty -F /dev/ttyUSB0 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw -echo
timeout 5 xxd -g 1 /dev/ttyUSB0
```

如果仍然无输出，检查 ALX 手环、基站配对、供电和线缆。

## 9. 本次实测参考结果

本地实测过程中，问题链路为：

```text
lsusb 能看到 CH340
brltty-udev 抢占设备
CH340 接口显示 Driver=usbfs
/dev/ttyUSB0 不存在
```

修复后状态：

```text
brltty-udev.service masked
CH340 接口 Driver=ch341
/dev/ttyUSB0 出现
chmod 666 /dev/ttyUSB0 后可读
stty 设置 115200 raw 后 xxd 能看到 ALX 帧
```

读到的有效帧特征：

```text
ff ff ff ff ... 20 01 ...
```

这说明 USB 驱动、串口节点、权限和 ALX 原始信号链路均已打通。
