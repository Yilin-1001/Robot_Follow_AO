# Spec: 手环状态日志由 AO 主线程输出

**Date:** 2026-07-13  
**Status:** Approved

## Goal

Ctrl+C 之后，即使 G1 SDK 调用仍阻塞，后台串口线程也不得向终端打印
`[WRISTBAND] OFF`。正常运行时，手环 OFF 后再 ON 仍须重新启动 AO。

## Design

`ALXG100SerialReader` 继续在后台线程解析 ON/OFF 帧，并通过既有 callback
更新 `WristbandGateState`；它不再执行任何终端打印。AO CLI 和 Web CLI 的主
控制循环在每个 tick 前读取该状态，仅在 `enabled` 从一个已知值变为另一个已知
值时打印一次 `[WRISTBAND] ON` 或 `[WRISTBAND] OFF`。

状态更新与日志输出解耦：`AORadarRuntime.tick()` 仍按当前状态停止或启动雷达，
所以 OFF → ON 的 AO 重启行为保持不变。主线程被 G1 SDK 阻塞或正在退出时不会
处理新的日志事件，因此后台收到的 OFF 帧不能污染终端。

## Non-goals

- 不改变 ALX 帧协议、手环状态 callback、AO 安全门控或雷达生命周期。
- 不解决 G1 SDK 同步调用的最长 10 秒阻塞；该问题另行处理。
- 不改变 G1 bridge 的既有未提交修改。

## Acceptance criteria

1. 解析 ON/OFF 帧仍更新 `WristbandGateState`，但串口 reader 本身没有 stdout
   输出。
2. 两个 AO CLI 入口各自在主线程仅打印一次状态转换；重复同状态不重复打印。
3. OFF 后 ON 仍让 runtime 进入现有的正常运行路径。
4. 相关单元测试通过。
