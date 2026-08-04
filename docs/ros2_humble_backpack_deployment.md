# ROS 2 Humble 背包部署

本项目的 Humble 目标基线是 Ubuntu 22.04、`arm64`、系统 Python 3.10 和
ROS 2 Humble。不要复用 Foxy 的 `build/`、`install/`、conda 环境或
`/home/fk/...` 的绝对路径。

## 1. 建立运行时

在新背包上先 source ROS，再用**系统 Python**建立 venv。ROS 二进制的 `rclpy`
必须与系统解释器 ABI 一致，不能用旧的 conda Python 替代。

```bash
source /opt/ros/humble/setup.bash
cd /opt/robot-follow
/usr/bin/python3 -m venv .venv-humble --system-site-packages
.venv-humble/bin/pip install --upgrade pip
.venv-humble/bin/pip install -e '.[dev]'
```

验证 `python3 --version` 和 `.venv-humble/bin/python --version` 都是 3.10，
并由 venv 运行 `import rclpy`。不要从 PyPI 安装 `rclpy`。

### Conda 运行时（`robot_followao_radar_humble`）

若要将本地 `robot_followao_radar_humble` 的纯 Python 依赖带到背包，必须新建
**Python 3.10** 的同名环境；不要克隆开发机的 Conda 环境，也不要在 Conda 中
安装 PyPI 版本的 `rclpy`、ROS 消息包或 `cyclonedds`。

先将整个项目源码（包含 `requirements-humble.txt`）复制到背包，例如
`$HOME/Robot_Follow_AO`，再执行：

```bash
conda create -n robot_followao_radar_humble python=3.10 pip -y
conda activate robot_followao_radar_humble

# 不让 ~/.local 的旧全局 pip 包混入这个 Conda 环境。
conda env config vars set PYTHONNOUSERSITE=1
conda deactivate
conda activate robot_followao_radar_humble

cd "$HOME/Robot_Follow_AO"
python -m pip install --upgrade pip
python -m pip install -r requirements-humble.txt
python -m pip install --no-deps -e .
```

激活 Conda 环境后，再 source ROS 与新的 FAST-LIO workspace；这一步让 Conda 的
Python 3.10 看见由 apt 安装的 ROS 2 Python 包：

```bash
conda activate robot_followao_radar_humble
source /opt/ros/humble/setup.bash
source "$HOME/g1_mid360_ros2_ws/install/setup.bash"

python --version
python -c 'import rclpy; from sensor_msgs.msg import PointCloud2; from sensor_msgs_py import point_cloud2; print("ROS Python OK")'
```

通过条件是 Python 为 3.10 且输出 `ROS Python OK`。若这一步失败，不要用 pip
补装 ROS 包；回退到本节前半部分的系统 Python venv，并保留 Conda 环境仅用于
不依赖 ROS 的开发和测试。`colcon build` 始终在 `conda deactivate` 后使用系统
Python 3.10 执行。

如果 `python -m pip check` 报错的包位置位于 `~/.local/lib/python3.10/site-packages`，
说明旧的用户级 pip 包泄漏进了 Conda 环境。不要卸载这些用户级包；在该环境中执行
`conda env config vars set PYTHONNOUSERSITE=1`，重新激活环境后再运行
`python -m pip check`。若之前已经在未隔离状态安装过 requirements，则在重新激活后
执行一次 `python -m pip install --no-user --force-reinstall -r requirements-humble.txt`，
把所有锁定依赖实际安装到 Conda 环境内。

## 2. 重新构建传感器工作空间

以 Humble 兼容版本重新构建 `livox_ros_driver2` 和 FAST-LIO，并 source：

```bash
source /opt/ros/humble/setup.bash
source /opt/g1_mid360_ros2_ws/install/setup.bash
```

确认实际的 launch 包名、launch 文件和 MID-360 参数文件；本项目不会替你假定
它们与 Foxy 分支完全相同。当前 Humble 的 `mapping.launch.py` 使用
`config_path` 与 `config_file` 两个参数；项目启动脚本会从完整路径变量
`FAST_LIO_CONFIG` 推导这两个参数。当前生产启动命令为：

```bash
cd ~/Robot_Follow_AO

source /opt/ros/humble/setup.bash
source /home/fk/g1_mid360_ros2_ws/install/setup.bash
conda activate robot_followao_radar_humble

export ROS_DOMAIN_ID=77
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

export FAST_LIO_WS=/home/fk/g1_mid360_ros2_ws
export FAST_LIO_CONFIG="$FAST_LIO_WS/src/FAST_LIO/config/mid360.yaml"
export FAST_LIO_CONFIG_DIR="$(dirname "$FAST_LIO_CONFIG")"
export FAST_LIO_CONFIG_FILE="$(basename "$FAST_LIO_CONFIG")"

export CUPY_CACHE_DIR=/tmp/robot-follow-cupy-cache
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"

test -r "$FAST_LIO_CONFIG" || {
  echo "FAST-LIO config not found: $FAST_LIO_CONFIG"
  exit 1
}

python -m robot_follow.lidar_alx_assisted_follow.cli \
  configs/jetson_g1_lidar_alx_assisted_follow_production.yaml \
  --confirm-real-g1 \
  --allow-provisional-calibration \
  --host 0.0.0.0 \
  --port 8080 \
  --allow-unauthenticated-lan \
  --log-level DEBUG
```

启动前也可运行以下只读检查：

```bash
scripts/check_humble_backpack.sh
ros2 topic info -v /cloud_registered_body
ros2 topic echo --once /cloud_registered_body
```

点云必须是 `sensor_msgs/msg/PointCloud2`，包含 `x/y/z` 字段，且 `frame_id` 与
配置的 `sensors.expected_frame_id` 一致（默认 `body`）。

## 3. ALX 设备与配置

新背包确认使用 `/dev/ttyUSB0`。将运行用户加入 `dialout` 组，并在硬件测试前确认该
设备存在。若未来出现 USB 重连导致编号变化，可选地再为 CH340/ALX 创建稳定 udev
链接；当前 Humble 配置和 systemd 模板不要求这样做。

新背包还必须根据实际设备更新：

- `g1.network_interface`：G1 所在网卡名；不能假定为 `eth0`。
- `FAST_LIO_CONFIG`：Humble 工作空间内的真实 MID-360 参数文件。
- `ROS_DOMAIN_ID`：当前生产 ROS 图使用 77；所有 ROS 节点和 FAST-LIO 必须一致。

## 4. Unitree DDS 隔离与验证

Unitree Python SDK 要求 `cyclonedds==0.10.2`，Humble 的 ROS 中间件版本线不同。
因此，默认启动脚本使用 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 和 ROS domain 20；
Unitree SDK 继续在其 domain 0 工作。禁止在未验证前将 Unitree 的 CycloneDDS
安装覆盖到通用 ROS venv。

`vendor/` 被主项目的 `.gitignore` 忽略，不会随主仓库自动同步。若需要真实 G1
输出，请在背包端检出与开发端一致的 Unitree SDK 版本，并将它安装到
`robot_followao_radar_humble` Conda 环境，不能依赖 `~/.local` 的旧用户级安装：

```bash
cd "$HOME/Robot_Follow_AO"
mkdir -p vendor
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git \
  vendor/unitree_sdk2_python
git -C vendor/unitree_sdk2_python checkout 432dd0655d0c88c301d53762c932ce88ebc35563

conda activate robot_followao_radar_humble
python -m pip install --no-user --no-deps -e vendor/unitree_sdk2_python
python -m pip install --no-user \
  'cyclonedds==0.10.2' \
  'opencv-python==4.10.0.84'
python -m pip check
```

随后先使用 `python -m robot_follow.real.g1.check --init-only` 做不发送移动指令的
SDK 初始化验证；必须将 `--net-iface` 设置为实际连接 G1 的网卡名。

先在 mock 输出下验证完整启动、ALX 手环 OFF、点云超时和进程退出都能 STOP；随后
做 Unitree 单独导入与零速度测试。若同一进程导入/初始化 ROS 和 Unitree SDK 出现
动态库、DDS 发现或控制异常，必须将 G1 bridge 分离为独立进程，用本地 IPC 传递
速度命令后再继续真实运动测试。

## 5. systemd

将 `systemd/robot-follow-humble.env.example` 复制为
`/etc/robot-follow/humble.env` 并修改路径；再部署
`systemd/robot-follow-humble.service`。服务使用 `/dev/ttyUSB0` 作为启动前置条件，
且所有 ROS 环境只由 `scripts/run_humble_lidar_assisted_follow.sh` 建立。

当前启动脚本默认使用 production 配置，并传入真实 G1 所需的
`--confirm-real-g1` 与 `--allow-provisional-calibration`。因此，只有在已完成现场
安全检查并确认隔离局域网后，才能部署或启动该 systemd 服务。
