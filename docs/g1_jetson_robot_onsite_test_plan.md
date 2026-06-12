# G1 Jetson Onsite Robot Test Plan

本文档描述从本地开发环境部署到 Jetson，并在真实 G1 机器人上进行 ALX-G100 跟随测试的完整流程。目标是优先贴近交付场景进行机器人实测，同时保留必要的安全分阶段验证和可复现记录。

## 1. 测试目标

本轮现场测试的目标按优先级排序：

1. 验证 Jetson 上代码、Python 环境、ALX 串口、G1 SDK 链路可用。
2. 验证 G1 motion bridge 的初始化、StopMove、单次低速 Move。
3. 验证 ALX-G100 数据经过 `FollowPipeline` 后能稳定输出 `MotionCommand`。
4. 在极低速配置下完成 ALX + G1 闭环跟随。
5. 将现场调参结果、日志、CSV 和最终配置回传到本地开发环境。

## 2. 总体流程

```text
本地开发机
  -> 整理代码 / 提交分支
  -> 部署到 Jetson
  -> Jetson 安装环境
  -> Jetson 跑 ALX dry-run
  -> G1 bridge 单项测试
  -> ALX + G1 闭环低速测试
  -> Jetson 调参和记录日志
  -> 参数 / 日志回传本地
  -> 本地复盘、改代码、再部署
```

## 3. 部署到 Jetson

推荐使用 Git 分支部署，避免手工拷贝散文件。

### 3.1 本地准备分支

```bash
cd /home/yilin/Code/Robot_Follow
git status
git checkout -b robot-g1-onsite-test
git add .
git commit -m "wip: prepare g1 onsite robot test"
git push origin robot-g1-onsite-test
```

### 3.2 Jetson 拉取代码

```bash
cd ~/Robot_Follow
git fetch origin
git checkout robot-g1-onsite-test
git pull
```

### 3.3 无法访问 Git 远端时使用 rsync

```bash
rsync -av --delete \
  --exclude .git \
  --exclude __pycache__ \
  --exclude .pytest_cache \
  --exclude "*.egg-info" \
  /home/yilin/Code/Robot_Follow/ \
  jetson:~/Robot_Follow/
```

### 3.4 本地修改代码后通过 SSH 更新 Jetson

现场高频迭代时，可以不用 Git，直接从本地通过 SSH/rsync 同步代码到 Jetson。推荐命令如下：

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

如果 Jetson 的 SSH 名称不是 `jetson`，替换成实际地址：

```bash
rsync -av --delete \
  --exclude .git \
  --exclude __pycache__ \
  --exclude .pytest_cache \
  --exclude "*.egg-info" \
  --exclude logs \
  --exclude configs/runs \
  /home/yilin/Code/Robot_Follow/ \
  yilin@192.168.x.x:~/Robot_Follow/
```

同步后在 Jetson 上验证：

```bash
ssh jetson 'cd ~/Robot_Follow && conda run -n robot-follow python -m pytest'
```

如果修改了依赖、入口点或包结构，再执行一次 editable install：

```bash
ssh jetson 'cd ~/Robot_Follow && conda run -n robot-follow pip install -e ".[dev]"'
```

注意：不要把 `logs/` 和 `configs/runs/` 用 `--delete` 覆盖掉，否则可能删除 Jetson 上刚测出来的现场数据。

## 4. Jetson 环境准备

在 Jetson 上执行：

```bash
cd ~/Robot_Follow
conda env list
conda create -n robot-follow python=3.10 -y
conda activate robot-follow
pip install -e ".[dev]"
```

安装 Unitree SDK 必须使用 GitHub 源码，不要使用 PyPI 包：

```bash
cd /tmp
git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git
conda run -n robot-follow pip install -e /tmp/unitree_sdk2_python
```

验证环境：

```bash
conda run -n robot-follow python -c "from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient; print('SDK OK')"
conda run -n robot-follow python -m pytest
conda run -n robot-follow python -m robot_follow.main --help
```

## 5. 硬件连接检查

### 5.1 ALX 串口

```bash
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM*
dmesg | tail -80
```

确认 CH340：

```bash
lsusb | grep -i "1a86:7523"
```

如果能看到 CH340 但没有 `/dev/ttyUSB0`，优先检查 `brltty` 是否抢占 CH340。

### 5.2 G1 网络接口

```bash
ip addr
```

记录实际 G1 网络接口，例如：

```text
G1_NET_IFACE=enp2s0
```

后续所有 G1 命令都使用该接口。

## 6. 分阶段测试计划

不要一上来直接闭环。每个阶段通过后再进入下一阶段。

### Stage 1: ALX 只读 dry-run

目标：确认 ALX 数据稳定，pipeline 能输出命令，但不控制机器人。

```bash
cd ~/Robot_Follow
mkdir -p logs
conda run -n robot-follow python -m robot_follow.main \
  --port /dev/ttyUSB0 \
  --duration 20 \
  --csv logs/alx_dry_run_001.csv
```

观察标准：

```text
status=live
alx_fps 接近 40
age 不持续升高
distance / azimuth 合理
cmd=move 或 stop 原因符合预期
```

如果这个阶段不稳定，不要进入 G1 闭环。

### Stage 2: G1 SDK init-only

目标：只初始化 G1 bridge，不移动。

```bash
conda run -n robot-follow python -m robot_follow.real.g1.check \
  --net-iface enp2s0 \
  --init-only \
  --i-understand-this-moves-robot
```

通过标准：SDK 初始化成功，退出时没有异常。

### Stage 3: StopMove only

目标：确认 StopMove 能发出去，机器人保持静止。

```bash
conda run -n robot-follow python -m robot_follow.real.g1.check \
  --net-iface enp2s0 \
  --stop-only \
  --i-understand-this-moves-robot
```

通过标准：G1 不移动，日志显示 StopMove 成功。

### Stage 4: 单次低速 Move

目标：验证前进方向、旋转方向、StopMove。

前进测试：

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

旋转方向测试：

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

如果旋转方向反了，记录：

```text
yaw_sign = -1
```

### Stage 5: ALX + G1 极低速闭环

只有前面阶段全部通过后再执行。

```bash
conda run -n robot-follow python -m robot_follow.main \
  --port /dev/ttyUSB0 \
  --net-iface enp2s0 \
  --enable-motion \
  --i-understand-this-moves-robot \
  --max-vx 0.03 \
  --max-omega 0.10 \
  --duration 10 \
  --csv logs/closed_loop_001.csv
```

第一轮不要超过 10 秒。测试人员站在机器人正前方 1.5m 到 2.0m，安全员随时准备遥控器急停。

## 7. Jetson 现场调参方法

现场调参优先通过 CLI 或配置文件，不要直接改代码。

第一批关键参数：

```text
--max-vx
--max-omega
--yaw-sign
--stale-timeout
--hz
```

建议后续增加配置文件，例如：

```yaml
# configs/jetson_g1_safe.yaml
g1:
  net_iface: enp2s0
  max_vx: 0.03
  max_omega: 0.10
  yaw_sign: 1

alx:
  port: /dev/ttyUSB0
  stale_timeout_s: 0.5

follow:
  target_distance_m: 1.5
  hold_tolerance_m: 0.15
  angle_deadzone_deg: 7.0
  large_angle_deg: 20.0
```

现场每轮测试复制一份配置：

```bash
mkdir -p configs/runs
cp configs/jetson_g1_safe.yaml configs/runs/2026-06-12-test-001.yaml
```

测试日志建议按 run 编号保存：

```bash
mkdir -p logs/runs/001
conda run -n robot-follow python -m robot_follow.main \
  --port /dev/ttyUSB0 \
  --net-iface enp2s0 \
  --enable-motion \
  --i-understand-this-moves-robot \
  --max-vx 0.03 \
  --max-omega 0.10 \
  --duration 10 \
  --csv logs/runs/001/closed_loop.csv \
  2>&1 | tee logs/runs/001/console.log
```

如果暂时没有 `--config`，把实际命令写入日志目录：

```bash
cat > logs/runs/001/command.txt <<'EOF'
conda run -n robot-follow python -m robot_follow.main ...
EOF
```

## 8. 参数和日志回传本地

### 8.1 Git 提交配置和摘要

适合回传最终可复现参数和简短结论：

```bash
git add configs/runs/2026-06-12-test-001.yaml logs/runs/001/summary.md
git commit -m "test: record jetson g1 tuning run 001"
git push origin robot-g1-onsite-test
```

本地拉取：

```bash
git pull
```

### 8.2 rsync 回传完整日志

适合回传大 CSV、console log、临时配置：

```bash
rsync -av jetson:~/Robot_Follow/logs/runs/ ./logs/runs/
rsync -av jetson:~/Robot_Follow/configs/runs/ ./configs/runs/
```

### 8.3 只回传最终稳定配置

```bash
scp jetson:~/Robot_Follow/configs/jetson_g1_safe.yaml ./configs/
```

建议：现场保留所有 run 的 CSV 和 console log，但只把可复现配置和简短 summary 提交进 Git。

### 8.4 不使用 Git，直接通过 SSH 传参数文件

现场调参时可以完全不用 Git，直接用 SSH 传输参数、日志和 CSV。

传单个参数文件推荐用 `scp`。

从 Jetson 拉回本地：

```bash
scp jetson:~/Robot_Follow/configs/jetson_g1_safe.yaml \
  /home/yilin/Code/Robot_Follow/configs/
```

从本地推到 Jetson：

```bash
scp /home/yilin/Code/Robot_Follow/configs/jetson_g1_safe.yaml \
  jetson:~/Robot_Follow/configs/
```

同步整个参数或日志目录推荐用 `rsync`。

从 Jetson 拉回本地：

```bash
rsync -av jetson:~/Robot_Follow/configs/runs/ \
  /home/yilin/Code/Robot_Follow/configs/runs/

rsync -av jetson:~/Robot_Follow/logs/runs/ \
  /home/yilin/Code/Robot_Follow/logs/runs/
```

从本地推送配置到 Jetson：

```bash
rsync -av /home/yilin/Code/Robot_Follow/configs/ \
  jetson:~/Robot_Follow/configs/
```

也可以用 `ssh` 快速覆盖远端参数文件：

```bash
ssh jetson 'cat > ~/Robot_Follow/configs/jetson_g1_safe.yaml' < \
  /home/yilin/Code/Robot_Follow/configs/jetson_g1_safe.yaml
```

不用 Git 时，每轮测试必须在 `summary.md` 或 `command.txt` 中记录实际命令、参数和代码版本，否则后续很难复现。

## 9. 单轮测试记录模板

每一轮测试都写一个 `summary.md`：

```markdown
# Run 001

Date:
Operator:
Jetson commit:
Command:
Config:
ALX port:
G1 net iface:

## Result

- Pass/Fail:
- Observed behavior:
- Safety issue:
- Max vx:
- Max omega:
- yaw_sign:

## Notes

- ALX fps:
- data timeout:
- command reason:
- stop behavior:
```

记录当前代码版本：

```bash
git rev-parse HEAD
```

## 10. 现场问题定位策略

现场以机器人实测为主，但遇到问题时要快速降级到最小链路。

### 10.1 机器人没动

1. 先跑 `g1.check --single-move`。
2. 如果不动，问题在 G1 SDK / 网络 / 机器人状态。
3. 如果单次能动，问题在 `main.py` 主循环 / pipeline / safety stop。

### 10.2 机器人乱转

1. 先跑 `g1.check --single-move --omega 0.10`。
2. 确认 yaw 方向。
3. 如果反了，使用 `--yaw-sign -1`。
4. 不要先改 controller。

### 10.3 机器人不跟人

1. 跑 ALX dry-run，看 `distance`、`azimuth`、`status`、`cmd reason`。
2. 如果 `status=stale` 或 `data_timeout`，优先查 ALX。
3. 如果一直 `holding`，查目标距离和 deadzone。
4. 如果一直 `target_too_far`，查最大跟随距离或 ALX 距离。

### 10.4 机器人一会儿动一会儿停

检查 CSV：

```text
status
alx_age_ms
command_reason
```

判断：

```text
高频 data_timeout -> 串口或 ALX 刷新问题
高频 angle_jump / distance_jump -> 滤波或 ALX 数据跳变
高频 sdk_error -> G1 SDK 或运动状态问题
```

## 11. 调参策略

现场每次只改一个参数，每轮 10 到 20 秒。

建议递增顺序：

```text
max_vx:    0.03 -> 0.05 -> 0.08 -> 0.10
max_omega: 0.10 -> 0.15 -> 0.20 -> 0.30
```

调参原则：

```text
不要同时调速度、角速度、滤波、deadzone。
先确认 yaw_sign，再提高 max_omega。
先确认 ALX 稳定，再提高 max_vx。
每轮都保存 CSV 和 console.log。
```

## 12. 推荐交付节奏

### 第一天：打通链路

```text
ALX dry-run 稳定
G1 init 成功
G1 stop-only 成功
G1 single-move 成功
ALX + G1 低速闭环 10 秒成功
```

### 第二天：参数优化

```text
yaw_sign 固化
max_vx 提高
max_omega 提高
target distance 调整
deadzone 调整
日志复盘
```

### 第三天：交付场景验证

```text
连续 1 分钟低速跟随
人左右偏移
人走近 / 走远
ALX 暂时遮挡
急停恢复
重复 3 轮
```

## 13. 建议优先补齐的能力

为了提高现场测试效率，建议优先实现：

1. `robot_follow.real.g1.check`
   - 专门做 G1 init / stop / single-move。
   - 不读 ALX。
   - 不进入跟随闭环。

2. 配置文件加载
   - 让 Jetson 调参不需要改代码。
   - 参数可以直接回传本地。
   - 每轮测试可复现。

完成这两个能力后，现场问题可以快速拆成 ALX、pipeline、G1 bridge 三段定位。
