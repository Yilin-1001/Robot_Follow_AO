# LiDAR APF Kalman Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to implement this plan task-by-task.

**Goal:** Only pass a Kalman-filtered confirmed target center to APF while leaving target confirmation raw.

**Architecture:** `TargetLockManager` owns the raw target center and a private compact filter. It exposes a separate APF-only center; the controller supplies cloud receive time and uses only that output in `APFInput`.

**Tech Stack:** Python 3.8, NumPy, dataclasses, pytest, Conda environment `robot_followao_radar`.

## Constraints

- Do not alter dynamic gating, quality checks, ALX consistency, confidence inputs, Web protocol, or CSV fields.
- Do not add prediction-only behavior, sensor fusion, dependencies, or hardware execution.
- Use `CloudSnapshot.received_at_s` only for filter time; retain `now_s` for loss and safety timers.

## Tasks completed

- [x] Add a 4-state constant-velocity `KalmanFilter2D` with reference-compatible covariance initialization and tests for initialization, smoothing, invalid timestamps, and reset after a gap.
- [x] Add validated Kalman configuration and an APF-only `TargetLockManager.follow_center_xy`; keep all raw tracking state unchanged.
- [x] Pass `cloud.received_at_s` only to the filter and provide `follow_center_xy` to APF; regression-test the controller boundary.
- [x] Run `conda run -n robot_followao_radar pytest tests/lidar_alx_assisted_follow/ -q` and frontend `npm run typecheck && npm run build`.
