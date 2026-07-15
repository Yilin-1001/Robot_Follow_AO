# LiDAR + ALX 目标跟踪 Kalman 滤波设计

## 目标

为 `lidar_alx_assisted_follow` 增加一个简洁的二维匀速 Kalman 滤波器，降低点云目标质心抖动。每帧质量检查通过后，滤波后的目标中心直接作为 APF 的跟随目标。

设计参考 `6-robot/jie_deamon` 的 `KalmanFilter2D`：状态为 `[x, y, vx, vy]`，观测为点云目标质心，使用匀速模型和两个噪声参数。与参考实现不同，本项目使用 `CloudSnapshot.received_at_s` 计算时间间隔，以保持与既有年龄安全链路一致。

## 范围

本次只实现相对 `body` 坐标系中的短时平滑与相对速度估计。

不实现：

- 里程计或 FAST-LIO 位姿接入；
- 世界坐标系中的人类真实速度；
- ALX 与点云的 Kalman 多传感器融合；
- 无观测期间持续预测并驱动 APF；
- 预测门控、协方差 Web 展示或 CSV 新字段。

## 算法与数据流

滤波器位于 `tracking/kalman.py`，只负责状态估计，不负责目标质量、停止决策或运动命令。

```text
有效新点云
  -> 现有动态椭圆门控与质量检查
  -> 原始质心 raw_center_xy
  -> Kalman.update(raw_center_xy, cloud.received_at_s)
  -> filtered_center_xy
  -> TargetLockManager.center_xy
  -> APF
```

滤波器状态为：

```text
x = [px, py, vx, vy]
z = [px, py]
```

对时间间隔 `dt`，状态转移使用匀速模型：

```text
px' = px + vx * dt
py' = py + vy * dt
vx' = vx
vy' = vy
```

每次 `update()` 先预测，再用当前原始点云质心校正。输出的校正后位置是 `filtered_center_xy`；它不是纯预测位置，也不是原始质心的简单替换。

## 时间与失效语义

- `dt` 是相邻两次有效目标观测对应点云的 `received_at_s` 之差，单位为秒。
- `dt <= 0` 的观测不参与滤波更新。
- `dt > target.max_frame_interval_s` 时，认为轨迹已经中断：以当前原始质心重置滤波器，速度归零，不利用陈旧速度。
- 本帧门控或几何质量不通过时，不调用 `update()`；保留当前既有的 `lost_timeout_s`、置信度下降和停止语义。
- 无有效观测时，不调用 `predictOnly()`，不将预测位置写入 `center_xy`，也不允许 APF 使用预测位置产生运动。
- 新 Web 选人、`unlock`、`LOST`、生命周期硬故障和硬停止都会重置滤波器。

现有 `cloud_stale`、`strategy_timeout`、`strategy_stale`、ALX stale、StopLatch、命令 watchdog 和 APF 碰撞检查不因滤波器改变。

## 组件变更

### `tracking/kalman.py`

新增紧凑的 `KalmanFilter2D`：

- `reset(x, y, timestamp_s)`：以初始位置、零速度初始化；
- `update(x, y, timestamp_s)`：使用真实点云时间戳执行预测和校正，返回滤波位置；
- `is_initialized`：供跟踪器判断状态。

内部采用 NumPy 管理状态向量和 `4 x 4` 协方差，无额外第三方依赖。

### `tracking/target.py`

- `TargetLockManager` 私有持有一个 Kalman 滤波器；
- `select()` 在候选通过质量检查后，以原始质心初始化滤波器，并将该质心写入 `center_xy`；
- `update()` 在动态门控和质量检查均通过后调用 Kalman；将返回的滤波中心写入 `center_xy`；
- `GatedTargetObservation.center_xy` 始终保留原始质心；
- 失败、解锁或停止路径按本设计的重置语义重置滤波器。

为简化契约，不新增 `TargetTrackEstimate`、不改变 Web 协议和 CSV 列。现有 Web 的目标连接带和椭圆门控会自然使用已更新的 `TargetLockManager.center_xy`。

### `runtime/controller.py`

调用 `TargetLockManager.select()` 与 `update()` 时传入 `cloud.received_at_s`，而不是控制线程的 `now`。`now` 仍用于传感器年龄、watchdog 和策略总时长。

### 配置

在 `TargetConfig` 和部署 YAML 中新增且只新增：

```yaml
target:
  kalman_enabled: true
  kalman_process_noise: 0.10
  kalman_measurement_noise: 0.05
```

- `kalman_enabled` 为 `false` 时保留当前逐帧原始质心行为；
- `kalman_process_noise` 越大，滤波器越快追随新观测；
- `kalman_measurement_noise` 越大，滤波器越平滑、越信任历史状态；
- 三项必须分别满足布尔或有限正数校验。

## 安全与坐标系限制

状态仍在 `body` 坐标系，因此估计的速度是“目标相对机器人”的速度，混有机器人本体运动。该速度不进入制动距离、扫掠包络或任何物理速度断言。

原始点簇仍用于几何置信度；ALX 一致性仍与原始观测中心比较。这样 Kalman 不会掩盖错误目标点簇或 ALX 不匹配。

## 验收与测试

按 TDD 实现以下测试：

1. `KalmanFilter2D` 的首次初始化、固定速度预测加校正、非正 `dt` 拒绝和超间隔重置。
2. `TargetLockManager` 在首次选择时初始化滤波器；后续有效观测后 `center_xy` 为滤波中心、`GatedTargetObservation.center_xy` 保持原始中心。
3. 质量失败、空点云、`LOST`、`unlock` 与重新选择均不会使用旧滤波状态。
4. 运行时向跟踪器传递 `cloud.received_at_s`，APF 接收滤波后的中心。
5. 现有点云年龄、策略年龄、策略耗时、StopLatch 和 APF 安全测试全部保持通过。

验证命令：

```bash
conda run -n robot_followao_radar pytest tests/lidar_alx_assisted_follow/ -q
cd src/robot_follow/lidar_alx_assisted_follow/frontend && npm run typecheck && npm run build
```

不运行 ROS2、串口、Livox、FAST-LIO、Unitree SDK 或真实 G1。
