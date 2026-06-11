# Jetson 背包现场测试指南

## 1. 测试目标

本文档用于指导在 Jetson 背包上测试 Unitree G1 + ALX-G100 跟随项目。测试必须按阶段推进，不能直接进入机器人闭环。

测试总目标：

```text
先确认代码能在 Jetson ARM64 上运行
再确认 ALX-G100 接收器能被 Jetson 稳定读取
再确认协议解析得到的距离和角度可信
再确认 dry-run 算法输出合理
最后才接入 Unitree G1 低速闭环
```

当前仓库阶段：

```text
已经具备:
  ALX-G100 协议解析代码
  ALX-G100 串口读取代码
  协议解析单元测试

尚未完整具备:
  距离/角度滤波
  安全急停管理
  跟随控制器
  G1 控制桥
  完整 robot-follow CLI
```

因此当前优先测试：

```text
Jetson 环境
ALX-G100 串口识别
原始串口数据
协议解析
安装方向和坐标系
数据稳定性
```

暂不直接测试：

```text
G1 低速闭环跟随
完整软件急停闭环
客户演示
```

## 2. 测试前准备

### 2.1 硬件准备

需要准备：

```text
Unitree G1
Jetson Orin NX 背包电脑
ALX-G100 单基站接收器
ALX-G100 手环
USB 或 USB-TTL 转接线
G1 原厂遥控器
本地开发电脑
局域网或网线
空旷室内平地
```

### 2.2 安全准备

测试前确认：

```text
G1 原厂遥控器保持在线
测试人员能随时接管机器人
测试场地没有障碍物、台阶、坡道和密集人员
首次测试只做 ALX-G100 数据验证，不启动 G1 运动
闭环测试前必须先验证 StopMove 和遥控器接管
```

### 2.3 Jetson 登录

从本地电脑 SSH 登录 Jetson：

```bash
ssh fk@<JETSON_IP>
```

确认主机信息：

```bash
hostname
uname -m
lsb_release -a
```

预期：

```text
uname -m 输出 aarch64
系统为 Ubuntu 22.04
```

## 3. 本地代码同步到 Jetson

### 3.1 本地先运行测试

在本地开发电脑进入项目目录：

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

如果本地测试失败，不要同步到 Jetson，先在本地修复。

### 3.2 同步代码

从本地电脑执行：

```bash
rsync -av \
  --exclude .git \
  --exclude __pycache__ \
  --exclude .pytest_cache \
  /home/yilin/Code/Robot_Follow/ \
  fk@<JETSON_IP>:~/Robot_Follow/
```

说明：

```text
<JETSON_IP> 替换为 Jetson 的实际 IP
目标目录为 Jetson 上的 ~/Robot_Follow
```

### 3.3 Jetson 上安装项目

在 Jetson 上执行：

```bash
cd ~/Robot_Follow
python3 -m pip install -e ".[dev]"
```

如果 pip 版本较旧：

```bash
python3 -m pip install --upgrade pip
```

如果暂时只测试 ALX-G100 串口和协议，不需要安装 `unitree_sdk2_python`。

### 3.4 Jetson 上验证代码可运行

```bash
cd ~/Robot_Follow
python3 -c "import platform; print(platform.machine())"
python3 -c "from robot_follow.alx_g100_protocol import parse_frame; print(parse_frame)"
python3 -c "from robot_follow.serial_reader import ALXG100SerialReader; print(ALXG100SerialReader)"
pytest -q
```

通过标准：

```text
platform.machine() 输出 aarch64
两个 import 命令正常输出函数或类
pytest 通过
```

失败处理：

```text
ModuleNotFoundError:
  确认是否执行 python3 -m pip install -e ".[dev]"

pyserial 缺失:
  执行 python3 -m pip install pyserial

pytest 不存在:
  执行 python3 -m pip install pytest
```

## 4. Jetson 环境检查

### 4.1 Python 环境

```bash
python3 --version
python3 -m pip --version
which python3
```

通过标准：

```text
Python 3 可用
pip 可用
```

### 4.2 USB 设备检查

插入 ALX-G100 接收器前执行：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

插入 ALX-G100 接收器后执行：

```bash
dmesg | tail -n 40
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

通过标准：

```text
出现新的 /dev/ttyUSB0、/dev/ttyUSB1 或 /dev/ttyACM0
dmesg 中能看到 USB 串口设备接入信息
```

记录实际串口：

```text
ALX 串口设备: /dev/tty____
```

### 4.3 串口权限检查

假设设备是 `/dev/ttyUSB0`：

```bash
ls -l /dev/ttyUSB0
groups
```

如果用户不在 `dialout` 组：

```bash
sudo usermod -aG dialout $USER
```

然后退出 SSH，重新登录。

临时测试也可以执行：

```bash
sudo chmod 666 /dev/ttyUSB0
```

注意：`chmod 666` 只适合临时测试，重插设备后可能失效。

## 5. ALX-G100 原始串口数据测试

### 5.1 使用 miniterm 查看原始数据

```bash
python3 -m serial.tools.miniterm /dev/ttyUSB0 115200 --raw
```

退出方式：

```text
Ctrl+]
```

通过标准：

```text
终端有持续输出
输出可能是乱码，这是正常现象，因为协议是二进制帧
```

### 5.2 使用 xxd 查看十六进制数据

```bash
xxd -g 1 /dev/ttyUSB0
```

通过标准：

```text
能看到类似 ff ff ff ff 的帧头
```

停止：

```text
Ctrl+C
```

### 5.3 没有原始数据时的排查

按顺序检查：

```text
1. ALX-G100 接收器是否供电
2. 手环是否开机
3. 接收器和手环是否配对
4. USB 线是否支持数据
5. 串口设备名是否选错
6. 波特率是否为 115200
7. 如果使用裸 UART，TX/RX 是否接反
8. GND 是否共地
9. UART 电平是否为 3.3V TTL
```

## 6. ALX-G100 协议解析测试

### 6.1 单元测试

在 Jetson 上运行：

```bash
cd ~/Robot_Follow
pytest tests/test_alx_g100_protocol.py -q
```

通过标准：

```text
协议解析测试通过
```

### 6.2 真实串口解析测试

当前仓库尚未提供专用 ALX 读取脚本。建议后续新增：

```text
tools/read_alx.py
```

该脚本应输出：

```text
distance_m
azimuth_deg
elevation_deg
tag_status
frame_rate
```

预期输出格式：

```text
distance=1.25m azimuth=3deg elevation=-2deg tag_status=0x1234 fps=10.0
```

通过标准：

```text
手环靠近基站时 distance 变小
手环远离基站时 distance 变大
手环左右移动时 azimuth 有明显变化
tag_status 能持续打印
程序不会因为半包、错包或脏数据崩溃
```

## 7. 基站安装方向和坐标系测试

### 7.1 安装要求

基站安装在 G1 背包上时，应满足：

```text
基站正前方尽量对齐机器人正前方
基站固定牢靠，不能晃动
基站周围避免大面积金属遮挡
线缆固定，避免机器人运动时拉扯
```

### 7.2 五点位置测试

测试前提：

```text
机器人不运动
只读取 ALX-G100 数据
手环佩戴在测试人员手腕
测试人员缓慢移动
```

测试位置：

```text
正前方 1.5m
左前方 1.5m
右前方 1.5m
正前方 0.8m
正前方 2.0m
```

记录表：

```text
位置             实际距离    distance_m    azimuth_deg    elevation_deg    tag_status
正前方 1.5m      1.5m        ______        ______         ______           ______
左前方 1.5m      1.5m        ______        ______         ______           ______
右前方 1.5m      1.5m        ______        ______         ______           ______
正前方 0.8m      0.8m        ______        ______         ______           ______
正前方 2.0m      2.0m        ______        ______         ______           ______
```

通过标准：

```text
正前方 azimuth 接近 0
左前方和右前方 azimuth 符号相反
distance_m 与实际距离大致一致
近距离和远距离能区分
```

如果左/右方向反了：

```text
记录当前符号
后续在 FollowConfig 中反转 angle_gain
不要在硬件测试阶段直接改机器人控制代码
```

如果正前方 azimuth 偏差很大：

```text
检查基站安装方向
检查基站是否歪斜
检查机器人背包是否遮挡天线
```

## 8. 数据稳定性测试

### 8.1 静止测试

测试方法：

```text
测试人员站在机器人正前方 1.5m
保持 30 秒不动
记录 distance_m 和 azimuth_deg 的抖动范围
```

记录：

```text
测试时长: 30s
平均 distance_m: ______
distance_m 最大值: ______
distance_m 最小值: ______
distance_m 最大跳变: ______
平均 azimuth_deg: ______
azimuth_deg 最大值: ______
azimuth_deg 最小值: ______
azimuth_deg 最大跳变: ______
是否丢帧: 是 / 否
```

通过标准：

```text
静止时 distance 和 azimuth 没有持续大跳变
抖动可以被简单低通滤波压住
没有长时间丢帧
```

### 8.2 左右移动测试

测试方法：

```text
测试人员站在 1.5m 左右距离
缓慢向左移动 1m
再缓慢向右移动 1m
整个过程约 10-20 秒
```

观察：

```text
azimuth 是否连续变化
是否出现突然正负跳变
distance 是否大致稳定
```

通过标准：

```text
azimuth 变化方向明确
没有频繁大跳变
```

### 8.3 靠近远离测试

测试方法：

```text
从 2.0m 缓慢走到 0.8m
再从 0.8m 缓慢退到 2.0m
```

观察：

```text
distance 是否连续变小和变大
azimuth 是否保持合理
```

通过标准：

```text
distance 趋势正确
0.8m、1.5m、2.0m 能明显区分
```

### 8.4 遮挡和断流测试

测试方法：

```text
用身体短暂遮挡手环 5 秒
将手环背到身体后方
短暂关闭手环或让手环离开有效范围
```

观察：

```text
是否丢帧
是否只收到心跳帧
是否出现离谱距离或角度
恢复后是否能继续输出定位帧
```

通过标准：

```text
异常状态能被观察到
恢复后数据能继续输出
后续安全模块可以基于超时停止机器人
```

## 9. dry-run 算法测试

该阶段在 `filtering.py`、`safety.py`、`follow_controller.py`、`main.py` 完成后执行。

运行：

```bash
cd ~/Robot_Follow
robot-follow --serial-port /dev/ttyUSB0 --net-iface <G1_NET_IFACE> --dry-run
```

示例：

```bash
robot-follow --serial-port /dev/ttyUSB0 --net-iface enp2s0 --dry-run
```

预期输出：

```text
distance=1.50m azimuth=8.0deg vx=0.10 omega=0.16 reason=tracking_turning
```

测试动作和预期：

```text
人站正前方且距离约 1.3m:
  vx 接近 0
  omega 接近 0

人站正前方且距离约 2.0m:
  vx > 0
  omega 接近 0

人偏左或偏右:
  omega 非 0
  左右方向应与实际偏移匹配

人偏角过大:
  vx = 0
  只输出转向

人距离小于 0.75m:
  输出 STOP safety 或 vx = 0

手环断流:
  超过 data_timeout_s 后输出 STOP safety
```

通过标准：

```text
dry-run 输出符合跟随逻辑
没有连接 G1 时不会调用实际运动
数据异常时进入停止状态
```

## 10. G1 SDK 独立测试

该阶段只测试 Jetson 到 G1 的控制链路，不接 ALX-G100 闭环。

### 10.1 前置检查

确认：

```text
G1 原厂遥控器在线
机器人处于安全测试姿态
场地空旷
测试人员可以随时接管
```

确认网卡：

```bash
ip addr
```

记录 G1 通信网卡：

```text
G1_NET_IFACE: ______
```

### 10.2 运行 Unitree 官方示例

在 `unitree_sdk2_python` 目录中运行：

```bash
python3 example/g1/high_level/g1_loco_client_example.py <G1_NET_IFACE>
```

预期：

```text
出现交互菜单
可以执行低速移动或转向测试
```

测试原则：

```text
只使用低速选项
每次移动时间尽量短
每次动作后立即 StopMove
遥控器随时准备接管
```

通过标准：

```text
G1 能响应 Move
G1 能响应 StopMove
Ctrl+C 后机器人不会继续运动
遥控器能接管
```

如果 G1 不响应：

```text
检查网卡名是否正确
检查 Jetson 与 G1 网络连接
检查 unitree_sdk2_python 是否安装正确
检查 CycloneDDS 依赖
检查是否有其他程序占用控制链路
```

## 11. 低速闭环测试

该阶段必须在以下条件全部满足后执行：

```text
ALX-G100 串口稳定
协议解析稳定
坐标系方向已确认
数据稳定性可接受
dry-run 输出合理
G1 SDK Move/StopMove 独立测试通过
遥控器接管验证通过
```

### 11.1 启动前检查

```text
G1 原厂遥控器在线
测试人员站在安全距离外
ALX-G100 手环佩戴正确
Jetson SSH 终端可用
机器人前方 3m 内无障碍物
程序参数使用低速配置
```

### 11.2 启动闭环

```bash
cd ~/Robot_Follow
robot-follow --serial-port /dev/ttyUSB0 --net-iface <G1_NET_IFACE>
```

### 11.3 闭环动作测试

按顺序执行：

```text
1. 人站正前方 1.3m
   预期: 机器人不明显前进，omega 接近 0

2. 人缓慢退到 2.0m
   预期: 机器人低速前进

3. 人移动到左前方
   预期: 机器人优先转向，不应高速前进

4. 人移动到右前方
   预期: 机器人转向方向与人偏移方向一致

5. 人靠近到 0.75m 内
   预期: 机器人停止

6. 遮挡或关闭手环
   预期: 数据超时后机器人停止
```

### 11.4 人工停止测试

SSH 终端中输入：

```text
q
```

然后按 Enter。

预期：

```text
程序调用 StopMove
G1 停止
程序退出
```

再次运行程序，测试：

```text
Ctrl+C
```

预期：

```text
程序捕获中断
先调用 StopMove
再退出
```

最后测试：

```text
原厂遥控器接管
```

预期：

```text
遥控器可以中断或接管机器人
```

## 12. 失败排查表

### 12.1 没有串口设备

现象：

```text
ls /dev/ttyUSB* /dev/ttyACM* 没有输出
```

排查：

```text
更换 USB 线
更换 Jetson USB 口
检查 ALX-G100 是否供电
运行 dmesg 查看是否枚举失败
确认 USB-TTL 芯片是否需要驱动
```

### 12.2 有串口但没有数据

排查：

```text
确认波特率 115200
确认手环开机
确认基站和手环已配对
确认 TX/RX 是否接反
确认 GND 是否共地
确认是否选择了正确的 /dev/ttyUSB*
```

### 12.3 有数据但解析失败

排查：

```text
确认数据中是否有 FF FF FF FF
确认是否为 ALX-G100 单基站协议
确认 PacketLength 是否为 0x0025 或 0x0010
确认串口数据是否存在错位或丢字节
检查 XorByte 是否匹配
```

### 12.4 distance 不变化

排查：

```text
确认移动的是已配对手环
确认输出的是 0x2001 定位帧，不只是 0x2002 心跳
确认手环没有休眠
确认基站和手环距离在有效范围内
```

### 12.5 azimuth 方向反了

处理：

```text
记录左/右方向对应的符号
优先确认基站是否装反
如果安装方向固定，则后续在 FollowConfig 中反转 angle_gain
```

### 12.6 dry-run 输出不合理

排查：

```text
先检查 distance_m 和 azimuth_deg 是否合理
再检查滤波输出
再检查 FollowController 参数
确认角度单位是 degree，不是 rad
确认距离单位已经 cm 转 m
```

### 12.7 G1 不响应

排查：

```text
确认 G1_NET_IFACE 是否正确
确认 Jetson 与 G1 网络连通
确认 unitree_sdk2_python 示例是否能运行
确认 CycloneDDS 环境
确认机器人当前模式允许高层控制
确认没有其他程序占用控制链路
```

### 12.8 StopMove 不可靠

处理：

```text
立即停止闭环测试
只保留 G1 SDK 独立测试
确认 StopMove 调用是否进入 finally
确认 Ctrl+C 是否能触发清理逻辑
确认遥控器接管是否可靠
不要继续进行客户演示
```

## 13. 现场测试记录模板

### 13.1 环境记录

```text
日期:
测试人员:
Jetson IP:
Jetson 系统:
Python 版本:
项目 commit:
ALX 串口:
G1 网卡:
测试场地:
```

### 13.2 ALX 坐标系记录

```text
位置             实际距离    distance_m    azimuth_deg    elevation_deg    tag_status    备注
正前方 1.5m      1.5m        ______        ______         ______           ______        ______
左前方 1.5m      1.5m        ______        ______         ______           ______        ______
右前方 1.5m      1.5m        ______        ______         ______           ______        ______
正前方 0.8m      0.8m        ______        ______         ______           ______        ______
正前方 2.0m      2.0m        ______        ______         ______           ______        ______
```

### 13.3 稳定性记录

```text
测试项             时长      结果                         是否通过
静止 1.5m          30s       __________________________    是 / 否
左右移动           20s       __________________________    是 / 否
靠近远离           20s       __________________________    是 / 否
人体遮挡           5s        __________________________    是 / 否
手环断开           5s        __________________________    是 / 否
```

### 13.4 dry-run 记录

```text
动作                     预期输出                     实际输出                     是否通过
正前方目标距离           vx≈0, omega≈0                 _______________________      是 / 否
正前方远距离             vx>0, omega≈0                 _______________________      是 / 否
左前方                   omega 方向正确                _______________________      是 / 否
右前方                   omega 方向正确                _______________________      是 / 否
距离过近                 STOP 或 vx=0                  _______________________      是 / 否
手环断流                 STOP safety                   _______________________      是 / 否
```

### 13.5 G1 安全记录

```text
测试项              预期                         实际                         是否通过
Move 低速前进       G1 低速前进                  _______________________      是 / 否
Move 低速转向       G1 低速转向                  _______________________      是 / 否
StopMove            G1 停止                      _______________________      是 / 否
Ctrl+C              StopMove 后退出              _______________________      是 / 否
q + Enter           StopMove 后退出              _______________________      是 / 否
遥控器接管          可接管                       _______________________      是 / 否
```

## 14. 进入下一阶段的门槛

### 14.1 进入 dry-run 的门槛

必须满足：

```text
Jetson 可运行项目代码
ALX-G100 串口稳定
能解析 0x2001 定位帧
distance_m 趋势正确
azimuth_deg 左右方向已记录
静止数据抖动可接受
```

### 14.2 进入 G1 SDK 测试的门槛

必须满足：

```text
G1 遥控器在线
测试场地安全
Jetson 与 G1 网络连接明确
unitree_sdk2_python 安装完成
不运行 ALX 闭环
```

### 14.3 进入低速闭环的门槛

必须满足：

```text
dry-run 输出合理
StopMove 独立测试通过
Ctrl+C 停止通过
q + Enter 停止通过
遥控器接管通过
机器人低速 Move 可控
现场人员明确谁负责接管
```

如果任一门槛不满足，不进入下一阶段。
