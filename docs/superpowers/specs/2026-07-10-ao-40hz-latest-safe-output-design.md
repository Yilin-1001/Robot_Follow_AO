# Spec: AO 输出 40 Hz 与 Raw Motion Command 同步

**Author:** Codex / Robot Follow team  
**Date:** 2026-07-10  
**Status:** Draft  
**Reviewers:** Project owner  
**Related specs:** `2026-07-09-robot-follow-ao-radar-design.md`  
**Evidence:** `logs/ao_radar_frames_20260710_154014.csv`

## 1. 背景

当前 AO Radar live runtime 的配置已经把 `ao_radar.latency.ao_output_loop_hz`
设为 `40.0`，理论输出周期应为 25 ms。参考日志
`logs/ao_radar_frames_20260710_154014.csv` 显示，本次运行共有 956 行 AO
输出记录，`loop_dt_ms` 中位数约 100.14 ms，p95 约 100.37 ms，p99 约
101.86 ms，实际 AO 输出约为 10 Hz。

同一份日志中，`g1_dispatch_ms` 中位数约 2.26 ms，p95 约 3.67 ms，p99 约
6.40 ms，最大约 15.92 ms；`ao_process_ms` 中位数约 2.80 ms，p99 约
6.53 ms。因此这次证据不支持优先引入复杂的异步 G1 sender、mailbox 或
stop epoch 机制。更可行的第一步是让现有同步链路按 25 ms 周期稳定运行，
使 AO safe command 的输出频率与 motion command raw 的输出频率保持一致。

本规格只覆盖 AO 输出频率和 raw/safe command 对齐语义。原有 AO 避障策略、
G1 限幅、雷达生命周期和手环安全门控继续沿用既有设计；除非本规格明确要求，
不做架构重写。

## 2. 目标

1. AO runtime MUST 以配置项 `ao_output_loop_hz=40.0` 对应的 25 ms 周期执行
   `tick()`。
2. 每个 AO tick MUST 产生一条最终 safe command，且该 safe command MUST
   对应该 tick 读取到的最新 raw motion command 或明确的 stop 原因。
3. 当 raw motion command 持续 40 Hz 更新时，AO safe command 输出也 MUST
   按 40 Hz 更新，不允许固定在 10 Hz。
4. 日志 MUST 能直接验证 raw command、safe command、AO 处理耗时、G1 dispatch
   耗时和实际循环周期。

## 3. 非目标

- 本次不引入异步 G1 sender、latest-safe mailbox、stop epoch 或新的 stop latch
  状态机。
- 本次不改变 AO planner 的避障规则、障碍区几何、follow 控制增益或速度限幅。
- 本次不要求 raw command 与 G1 RPC 在任何网络或 SDK 阻塞场景下绝对一一对应。
- 本次不实现物理运动反馈闭环，也不把 RPC ack 等价为机器人已经停止。
- 本次不改造 Web UI，只要求 Web 入口不能把 AO runtime 主循环降频。

## 4. 术语

- **Raw motion command:** Robot Follow follow pipeline 在进入 AO 避障前生成的
  原始运动命令。
- **Safe command:** raw motion command 经过手环、ALX 年龄、雷达年龄、AO planner
  和输出限幅后得到的最终命令。
- **AO tick:** AO runtime 的一次 `tick()` 调用。40 Hz 时周期为 25 ms。
- **Loop dt:** 当前 AO tick 与上一个 AO tick 开始时间的差值，对应日志字段
  `loop_dt_ms`。
- **AO output:** 每个 tick 生成并记录的 safe command；在 `output.mode=g1` 时也会
  通过现有 bridge dispatch 到 G1。

## 5. 功能需求

- **FR-1:** `robot-follow-ao-radar` CLI MUST 使用
  `1.0 / config.latency.ao_output_loop_hz` 作为 AO tick 周期；当配置为 40.0
  时，目标周期 MUST 为 25 ms。
- **FR-2:** `robot-follow-ao-radar-web` MUST 使用同一个 AO tick 周期运行
  runtime；`--web-hz` 只允许影响 Web 推送频率，MUST NOT 降低 AO tick 频率。
- **FR-3:** 主循环 MUST 以 tick 开始时间计算 `loop_dt_ms`，并在每次 tick 后
  按剩余周期 sleep；如果本次 tick 超过 25 ms，MUST 立即进入下一 tick，
  MUST NOT 额外 sleep 到 100 ms。
- **FR-4:** 每个 AO tick MUST 调用一次 raw follow pipeline，除非手环 OFF、
  手环 unknown、串口错误或 ALX 超时等安全门控在 raw 生成前已经要求 stop。
- **FR-5:** raw command 已生成的 tick MUST 在同一 tick 内完成 AO planner
  评估，输出 safe command；safe command 日志 MUST 保留该 tick 的 raw command
  kind、reason 和生成耗时。
- **FR-6:** 如果雷达未 ready、点云超时、command 超时或 AO 超时，AO MUST 在该
  tick 输出 stop，并在日志中保留可用的 raw command 信息。
- **FR-7:** `command_age_ms` MUST 反映当前 tick raw command 到 safe/dispatch
  阶段的年龄，不得用固定 0 或新的时间戳掩盖旧 command。
- **FR-8:** `cloud_age_ms` MUST 按当前 tick 时间计算，允许点云频率低于 40 Hz；
  AO 可以在多个 tick 使用最新未过期点云，但 MUST 按年龄超时规则输出 stop。
- **FR-9:** `g1_dispatch_ms` MUST 继续记录同步 bridge dispatch 的耗时；本规格不
  要求把 G1 dispatch 移到独立线程。
- **FR-10:** 如果单次 G1 dispatch 或 AO 处理超过 25 ms，主循环 MAY 漏掉该周期，
  但下一次 tick MUST 尽快开始，并通过 `loop_dt_ms` 暴露真实延迟。
- **FR-11:** 日志 MUST 每个 AO tick 写一行，字段至少包含：
  `timestamp`、`loop_dt_ms`、`ao_process_ms`、`g1_dispatch_ms`、
  `raw_command_kind`、`raw_command_reason`、`safe_command_kind`、
  `safe_command_reason`、`command_age_ms`、`cloud_age_ms`、
  `raw_command_generated_at_ms`、`safe_command_generated_at_ms`、
  `g1_response_at_ms`、`total_latency_ms`。
- **FR-12:** 配置文件 `configs/jetson_g1_ao_radar.yaml` MUST 保持
  `ao_radar.latency.ao_output_loop_hz: 40.0`，并且运行日志启动信息 MUST 打印
  实际 AO loop Hz。
- **FR-13:** 终端 summary、CSV 写入和 Web hub 更新 SHOULD 避免明显阻塞 AO tick；
  如果发现这些路径导致 40 Hz 不达标，优先降低 summary/Web 频率或优化日志 flush，
  而不是改变 AO 输出目标频率。

## 6. 非功能需求

- **NFR-1:** 在 mock G1 或当前日志同等 dispatch 耗时条件下，120 秒运行的
  `loop_dt_ms` p50 SHOULD 在 25 +/- 3 ms 内，p95 SHOULD <= 30 ms，p99 SHOULD
  <= 40 ms。
- **NFR-2:** `ao_process_ms + g1_dispatch_ms` 的 p99 SHOULD 小于 20 ms；若超过，
  日志 MUST 能定位是 AO、G1、日志还是 Web 更新造成延迟。
- **NFR-3:** 真实 G1 运行时，如果 `g1_dispatch_ms` 偶发超过 25 ms，系统 MUST
  记录真实 `loop_dt_ms`，不得伪造 40 Hz。
- **NFR-4:** 40 Hz 修改 MUST 保持原有安全停止优先级：手环 OFF/unknown、串口错误、
  ALX 超时、雷达未 ready、雷达超时、AO 超时均输出 stop。
- **NFR-5:** 本次改动 SHOULD 控制在 AO live runtime、CLI/web CLI 主循环、日志和
  对应测试范围内，避免引入跨模块大重构。

## 7. 数据模型

本规格沿用现有 `AORadarFrameResult` / `AORadarLogRow`，不新增 mailbox 或 sender
数据结构。

| 字段 | 类型 | 约束 |
|---|---|---|
| `timestamp` | float | tick 开始时的 monotonic 时间 |
| `loop_dt_ms` | float | 与上一 tick 开始时间差，目标约 25 ms |
| `raw_command_kind` | str | raw 已生成时为 `move` 或 `stop`；raw 前安全门控时可为空 |
| `raw_command_reason` | str | raw command 原因；无 raw 时为空 |
| `safe_command_kind` | str | 每 tick 必须为 `move` 或 `stop` |
| `safe_command_reason` | str | safe stop 必须写明原因 |
| `command_age_ms` | float | raw command 年龄 |
| `cloud_age_ms` | float | 最新点云年龄；无点云时可为 0 或空，但原因必须为 `radar_not_ready` |
| `ao_process_ms` | float | AO planner 耗时；未进入 planner 时为 0 |
| `g1_dispatch_ms` | float | bridge dispatch 耗时 |
| `raw_command_generated_at_ms` | float/null | raw 相对 tick 开始的生成时刻 |
| `safe_command_generated_at_ms` | float/null | safe 相对 tick 开始的生成时刻；raw 前 stop 可为空 |
| `g1_response_at_ms` | float/null | G1 dispatch 返回相对 tick 开始的时刻 |
| `total_latency_ms` | float/null | raw 生成到 G1 返回的延迟 |

## 8. 边界情况

- **EC-1:** 手环 OFF 持续 10 秒时，AO tick 仍应保持 40 Hz 输出 stop；不应退化为
  10 Hz。
- **EC-2:** Web 入口使用 `--web-hz 10` 时，Web payload 10 Hz 更新，但 AO tick
  和 CSV 日志仍为 40 Hz。
- **EC-3:** 点云低于 40 Hz 时，AO tick 仍为 40 Hz；只要点云未超过
  `max_cloud_age_s`，safe command 可使用最新点云评估。
- **EC-4:** 点云超过 `max_cloud_age_s` 时，每个 tick 输出
  `stop(reason="radar_timeout")`。
- **EC-5:** G1 dispatch 偶发耗时 35 ms 时，该 tick 的 `loop_dt_ms` 可超过 25 ms，
  后续 tick 必须尽快恢复，不能固定睡成 100 ms。
- **EC-6:** CSV 写入或终端 summary 变慢时，不得主动把 AO loop 降到 10 Hz；应通过
  日志暴露延迟来源。

## 9. 验收标准

### AC-1: CLI AO 主循环达到 40 Hz

Given `configs/jetson_g1_ao_radar.yaml` 中 `ao_output_loop_hz=40.0`  
When 使用 mock G1 运行 `robot-follow-ao-radar` 120 秒  
Then CSV 中 `loop_dt_ms` p50 在 25 +/- 3 ms 内  
And `loop_dt_ms` p95 <= 30 ms  
And `loop_dt_ms` p99 <= 40 ms

### AC-2: Web 入口不降低 AO 频率

Given AO loop 配置为 40 Hz 且 `robot-follow-ao-radar-web --web-hz 10`  
When 运行 120 秒  
Then AO CSV 日志约为 40 行/秒  
And Web hub 更新约为 10 次/秒  
And Web 更新不得改变 AO tick 周期

### AC-3: Safe command 与 raw command 同 tick 对齐

Given 手环 ON、ALX 新鲜、雷达点云新鲜  
When raw follow pipeline 在一个 tick 生成 `move`  
Then 同一 tick 的 safe command 必须由该 raw command 经过 AO planner 得到  
And 日志同一行包含 raw 和 safe 的 kind/reason  
And `safe_command_generated_at_ms >= raw_command_generated_at_ms`

### AC-4: 安全门控 stop 保留 raw 信息

Given raw command 已生成  
When 雷达未 ready、点云超时、command 超时或 AO 超时触发 stop  
Then safe command 为 `stop`  
And 日志保留该 tick 的 raw command kind/reason  
And safe stop reason 写明具体原因

### AC-5: Raw 前 stop 可以没有 raw command

Given 手环 OFF 或手环状态 unknown  
When AO tick 执行  
Then safe command 为 `stop`  
And raw command 字段允许为空  
And `loop_dt_ms` 仍按 40 Hz 节奏记录

### AC-6: 日志可复现实测频率

Given 生成一份新的 `ao_radar_frames_*.csv`  
When 统计 `loop_dt_ms`、`ao_process_ms` 和 `g1_dispatch_ms`  
Then 可以判断 AO 是否达到 40 Hz  
And 可以区分延迟来自 AO、G1 dispatch、日志/Web 或外部输入

## 10. 实施范围

预计只需要检查和必要时修改以下位置：

- `src/robot_follow/ao/live/cli.py`
- `src/robot_follow/ao/live/web_cli.py`
- `src/robot_follow/ao/live/runtime.py`
- `src/robot_follow/ao/logging/csv_logger.py`
- `configs/jetson_g1_ao_radar.yaml`
- `tests/ao/live/`
- `tests/ao/logging/`

实现时优先新增或调整确定性测试，覆盖 25 ms 周期、Web 降频隔离、日志字段和
raw/safe 同 tick 对齐。

## 11. 发布门槛

1. 单元测试通过：AO runtime、CLI/web CLI 周期计算、日志序列化相关测试必须通过。
2. Mock 运行验证：120 秒 mock G1 运行满足 AC-1。
3. Web 入口验证：120 秒 web CLI 运行满足 AC-2。
4. 真实 G1 低速验证：在当前安全限幅下运行，日志显示 AO tick 不再固定为约 100 ms；
   若真实 G1 dispatch 偶发拖慢，必须在日志中体现真实原因。

## 12. 开放问题

- `logs/ao_radar_frames_20260710_154014.csv` 显示当前 `g1_dispatch_ms` 足够低，
  但仍需确认导致 100 ms 周期的直接原因是运行入口、Web 入口、外部启动参数还是
  其他 sleep/summary 行为。
- 如果后续真实 G1 出现稳定超过 25 ms 的 dispatch 阻塞，再单独立项评估异步 sender
  或 latest-only 输出架构；该架构不属于本规格。
