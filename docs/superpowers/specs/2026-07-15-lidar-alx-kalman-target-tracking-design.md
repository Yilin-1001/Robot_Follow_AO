# LiDAR + ALX：仅 APF 目标点 Kalman 平滑

## 目标与范围

为 `lidar_alx_assisted_follow` 增加二维匀速 Kalman 滤波器，只平滑最终交给 APF 的已确认目标中心。原始点云目标确认链路保持不变。

实现参考 [`6-robot/jie_deamon` 的 `KalmanFilter2D`](https://github.com/6-robot/jie_deamon/blob/main/include/kalman_filter.hpp)：状态 `[x, y, vx, vy]`，观测 `[x, y]`，使用两个噪声参数和 `4 x 4` 协方差。实现采用现有 NumPy 依赖。

不实现预测驱动、预测门控、世界坐标速度、传感器融合、速度输出、Web/CSV 字段或额外安全机制。

## 数据流

```text
有效新点云
  -> 现有门控、质量检查、置信度和 ALX 一致性
  -> raw_center_xy
  -> Kalman.update(raw_center_xy, cloud.received_at_s)
  -> follow_center_xy
  -> APF.target_center_xy
```

- `TargetLockManager.center_xy` 和 `GatedTargetObservation.center_xy` 始终是原始质心。动态门控、Web、ALX 一致性、几何/时间置信度和失联逻辑继续使用原始值。
- `follow_center_xy` 是 APF 专用内部输出：Kalman 启用时为校正后的滤波位置，关闭时等于原始质心。
- 无有效观测时不预测、不运行 APF。

## 时间与重置

- 仅 Kalman 的 `dt` 使用相邻有效点云的 `CloudSnapshot.received_at_s`；`now_s` 继续服务于既有失联和安全计时。
- `dt <= 0` 不更新滤波状态，继续使用上一滤波位置。
- `dt > target.max_frame_interval_s` 时以当前原始质心、零速度重置。
- 新选人、`unlock`、`LOST` 重置滤波器。质量失败和短暂空点云不更新或预测滤波器。

## 组件与配置

`tracking/kalman.py` 提供 `KalmanFilter2D`：`reset(x, y, timestamp_s)`、`update(x, y, timestamp_s)` 和 `is_initialized`。初始位置协方差为 `1.0`，初始速度协方差为 `10.0`；过程与测量噪声默认值为 `0.10` / `0.05`。

`TargetLockManager` 私有持有滤波器和 `follow_center_xy`，但不改变原始 `center_xy`。`select()`、`update()` 接收可选 `measurement_timestamp_s`；控制器传入 `cloud.received_at_s`。

```yaml
target:
  kalman_enabled: true
  kalman_process_noise: 0.10
  kalman_measurement_noise: 0.05
```

关闭时 APF 逐帧接收原始质心，行为与改动前一致。

## 验收

1. Kalman 首帧初始化、平滑更新、非正 `dt` 保持状态和超间隔重置有单元测试。
2. 原始 `center_xy`、门控和置信度不因滤波而改变。
3. 启用时 APF 接收 `follow_center_xy`，关闭时接收原始中心。
4. 重选、解锁、丢失和长时间间隔后不复用旧速度。
5. 既有 LiDAR + ALX 测试与前端类型检查/构建保持通过；不运行 ROS2、串口、Livox、FAST-LIO、Unitree SDK 或真实 G1。
