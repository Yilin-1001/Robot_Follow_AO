# LiDAR + ALX Assisted Follow Prototype Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an independent `lidar_alx_assisted_follow` runtime in which one Web controller selects a local LiDAR target and authorizes low-speed Unitree G1 following, with ALX used only as supporting identity evidence.

**Architecture:** Follow the deliberately small `jie_deamon`-style model agreed in review: an aiohttp Web thread owns sockets, while one control thread is awakened by each new cloud and also wakes at least every 25 ms for safety supervision. Target extraction, confidence, APF, limiting, and `G1MotionBridge.dispatch()` run synchronously in that control thread; there is no G1 output worker, motion mailbox, authority epoch, AO runtime, or direct-follow pipeline. Cross-thread safety consists of a latest-cloud slot, a bounded Web-event deque, immutable latest status, and a one-way shared STOP latch that any thread may set but only the control thread may clear after a valid Start.

**Tech Stack:** CPython 3.8 (`robot_followao_radar`), ROS2 Foxy on Ubuntu 20.04/aarch64, NumPy, aiohttp, existing FAST-LIO and Unitree G1 adapters, TypeScript/Vite.

## Locked Decisions

- Strategy is cloud-driven: each distinct cloud sequence is processed at most once. A 40 Hz timeout wake performs safety checks only and never recomputes target extraction, confidence, or APF from an old cloud.
- Overload is latest-only. If several clouds arrive during one strategy pass, process the newest next and discard intermediate unprocessed frames.
- Web and control run in separate execution threads. Strategy and G1 dispatch remain synchronous in the one control thread.
- No output queue, G1 worker, authority epoch, or generation counters are used.
- Any thread may call `StopLatch.request(reason, kind)`. A HARD request dominates SOFT. Only the control thread may clear it through `clear_for_start` or `clear_soft_recovery`. Every nonzero dispatch rechecks the latch immediately before calling the bridge.
- Provisional calibration may drive real G1 only with both `--confirm-real-g1` and `--allow-provisional-calibration` and the velocity caps in this document.
- Malformed ALX frames remain silently discarded by the existing reader. This prototype does not modify `serial_reader.py` or `real/alx_source.py`; lack of a fresh valid ALX sample provides the fail-safe stop.
- `confidence_low` and `alx_stale` are soft stops that may auto-resume on the same lock. User Stop, controller loss, wristband/serial/lifecycle/bridge faults, cloud stale, and LOST/unlock are hard stops requiring Start again.
- User Stop preserves the target as `CANDIDATE` and continues zero-motion tracking. Controller loss and wristband/serial/bridge faults unlock.
- Existing observers are never promoted. After controller loss, the first newly connected WebSocket becomes controller and must select and Start again.
- The new mode owns an independent frontend source and `dist`; existing AO frontend files and payloads are unchanged.
- Web defaults to `127.0.0.1`. Non-loopback binding requires `--allow-unauthenticated-lan`; that risk is printed in startup logs and is not shown in the frontend.
- The browser can send only `select_target`, `start_follow`, and `stop_follow`; it can display but cannot provide velocity.
- Runtime CSV records APF-requested and software-dispatched velocities. It never labels them as physical/actual G1 velocity because no odometry feedback is consumed.
- This prototype stays on Python 3.8 for ROS2 Foxy ABI compatibility. It does not upgrade the Jetson runtime to Python 3.10+ independently of Ubuntu and ROS.
- Only new-mode Python tests and the new frontend typecheck/build are run. Existing Python/AO/G1 suites and all hardware-facing commands remain unexecuted.

## Global Constraints

- Mode name, package name, and YAML root are exactly `lidar_alx_assisted_follow`.
- Existing `alx_follow`, `ao_radar`, AO frontend, and current dirty AO/G1 changes remain untouched.
- Subscribe only to `/cloud_registered_body` and accept only exact `frame_id == "body"`; never use `base_link` or `robot_follow.ao.app.TransformConfig`.
- Calibration defaults are `x_m=0.0`, `y_m=0.0`, `yaw_deg=0.0`, `lidar_height_m=1.25`, `alx_height_m=1.10`; the canonical protocol/log value is always `calibration_status="provisional"`.
- Wristband ON plus the first fresh valid position binds `session_tag_id` and starts Livox/FAST-LIO. OFF or serial error requests zero, unlocks, clears the tag, and stops both services.
- ALX position age-out while wristband remains ON requests a soft stop but keeps Livox/FAST-LIO running.
- G1 bridge initializes at most once per CLI process. Wristband OFF stops motion and sensors but does not close/reinitialize DDS; a bridge init/runtime fault requires CLI restart.
- Controller disconnect requests zero synchronously in the disconnect handler and enqueues a mandatory disconnect event; the control thread unlocks before it can dispatch any later strategy result.
- No full-cloud person clustering, candidate enumeration/ranking, automatic discovery/switching, mapping, reverse translation, or live calibration.
- `vx >= 0`; `vy` is allowed. Mandatory stops bypass and reset the limiter.
- Every new Python module starts with `from __future__ import annotations`. Python enums use `class X(str, Enum)`, not Python 3.11-only `StrEnum`; do not use `asyncio.to_thread`, `removeprefix`, `removesuffix`, `tomllib`, `asyncio.Runner`, `TaskGroup`, structural pattern matching, or other Python 3.9+ APIs/syntax.
- Status frames and CSV rows use monotonic receive times for freshness. Never replace `ALXSnapshot.received_at` with a later polling time.
- The confirmed development environment is the conda environment `robot_followao_radar`, currently CPython 3.8.20. Before any Python execution, still follow `.codex/skills/robot-follow-conda-run-guard/SKILL.md`, report the active interpreter, and invoke this environment explicitly.
- Do not run ROS2, serial, Livox, FAST-LIO, Unitree SDK, or real G1 commands during implementation verification.

## Python and Jetson Compatibility Contract

- Deployment baseline is Jetson Ubuntu 20.04, ROS2 Foxy, aarch64, and Python 3.8. Do not infer Humble/22.04 assumptions from other repository documents.
- Python 3.8 and ROS2 Foxy are upstream end-of-life. That is an acknowledged deployment risk, not a reason to upgrade Python alone. A future migration must move JetPack/Ubuntu, ROS, FAST-LIO workspace, `rclpy`, and Python together as a separately tested project.
- The local `robot_followao_radar` environment currently proves only pure-Python development compatibility. It does not contain `rclpy`, `unitree_sdk2py`, or `cyclonedds`, and its x86_64 results do not qualify the Jetson.
- The Jetson environment may be used only if, after sourcing `/opt/ros/foxy/setup.bash` and the FAST-LIO workspace, its `robot_followao_radar` interpreter is Python 3.8 and imports `rclpy`, `sensor_msgs.msg.PointCloud2`, `sensor_msgs_py.point_cloud2`, `unitree_sdk2py`, and `cyclonedds` successfully.
- Matching the Python minor version is necessary but not sufficient: apt ROS binaries and Conda can still conflict through extension ABI and shared libraries. If the import/topic smoke gate fails, do not install a random PyPI `rclpy` and do not upgrade only Python. Recreate the Jetson runtime with `/usr/bin/python3.8 -m venv --system-site-packages` (environment directory named `robot_followao_radar`) or rebuild the complete ROS Python stack against the chosen interpreter.
- Unitree's vendored SDK contract remains Python `>=3.8` with `cyclonedds==0.10.2`. On aarch64, build/installation and DDS interface tests are Jetson qualification steps; never assume an x86 wheel result applies.
- Keep `pyproject.toml` at `requires-python = ">=3.8"`, but constrain newly added code to the 3.8 standard library. The implementation is verified by actually collecting and running the new suite under `robot_followao_radar`, not by metadata alone.

## Core Data Contracts

Use frozen dataclasses at component boundaries. NumPy arrays are copied once on construction and marked read-only.

```python
class StopKind(str, Enum):
    SOFT = "soft"
    HARD = "hard"

@dataclass(frozen=True)
class CloudSnapshot:
    sequence: int
    sensor_timestamp_s: float
    received_at_s: float
    frame_id: str
    points_xyz: np.ndarray

@dataclass(frozen=True)
class ALXObservation:
    snapshot: ALXSnapshot
    received_at_s: float
    body_xy: np.ndarray | None
    cloud_skew_s: float | None
    synchronized: bool

@dataclass(frozen=True)
class PreprocessedCloud:
    points_xyz: np.ndarray
    source_indices: np.ndarray

@dataclass(frozen=True)
class GatedTargetObservation:
    center_xy: tuple[float, float]
    target_mask: np.ndarray
    support_points_xyz: np.ndarray
    point_count: int
    vertical_extent_m: float
    horizontal_footprint_m: float
    radial_mad_m: float
    observed_at_s: float

@dataclass(frozen=True)
class ConfidenceResult:
    c_t: float
    c_g: float
    c_a: float
    total: float
    alx_fresh: bool
    reason: str

@dataclass(frozen=True)
class MotionIntent:
    sequence: int
    vx: float
    vy: float
    omega: float
    reason: str
    generated_at_s: float

@dataclass(frozen=True)
class StrategyResult:
    sequence: int
    apf_requested: MotionIntent
    dispatched: MotionIntent
    safety_complete: bool
    stop_reason: str | None
    compute_ms: float

@dataclass(frozen=True)
class StopSnapshot:
    requested: bool
    kind: StopKind | None
    reason: str | None
    start_after_sequence: int
    version: int
```

`StopLatch.request` increments `version`; HARD cannot be overwritten by SOFT. `clear_for_start` requires a valid Candidate, controller ownership, ready lifecycle, fresh ALX, `C>=start`, and records the latest cloud as `start_after_sequence`. `clear_soft_recovery` accepts only a current SOFT reason in `{confidence_low, alx_stale}` and the recovery conditions in this plan. HARD is cleared only by a valid explicit Start.

## Exact Prototype Defaults and Equations

All values below are fields in `configs/jetson_g1_lidar_alx_assisted_follow.yaml`; configuration rejects unknown keys and invalid ordering.

### Timing and bounds

```yaml
alx:
  port: /dev/ttyUSB0
  baudrate: 115200
  stale_timeout_s: 0.50
  cloud_max_skew_s: 0.15
  min_distance_m: 0.10
  max_distance_m: 10.0
sensors:
  topic: /cloud_registered_body
  expected_frame_id: body
  max_cloud_age_s: 0.25
  ready_timeout_s: 10.0
  stop_timeout_s: 3.0
  livox_start_command: "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
  fast_lio_start_command: "ros2 launch fast_lio mapping.launch.py config_file:=/home/fk/g1_mid360_ros2_ws/src/FAST_LIO_ROS2/config/mid360.yaml rviz:=false"
g1:
  network_interface: eth0
  max_vx: 0.10
  max_abs_vy: 0.08
  max_abs_omega: 0.25
  yaw_sign: 1
  sdk_timeout_s: 10.0
  sdk_timeout_code: 3104
  sdk_timeout_retry_interval_s: 1.0
  max_consecutive_sdk_timeouts: 3
  command_timeout_s: 0.50
output:
  mode: mock
runtime:
  watchdog_hz: 40.0
  max_strategy_age_s: 0.15
  max_strategy_process_ms: 20.0
  control_event_capacity: 32
```

The control loop waits on the cloud event with timeout `1 / watchdog_hz`. A cloud pass whose measured total exceeds `max_strategy_process_ms` is discarded and emits `strategy_timeout` zero. The timeout cannot preempt NumPy already running; this limitation is logged and documented.

### Cloud preprocessing and target gate

```yaml
target:
  radial_min_m: 0.25
  radial_max_m: 5.0
  physical_height_min_m: 0.10
  physical_height_max_m: 2.00
  self_x_min_m: -0.35
  self_x_max_m: 0.25
  self_abs_y_max_m: 0.25
  voxel_size_m: 0.05
  web_pick_radius_m: 0.35
  gate_radial_radius_m: 0.45
  gate_tangential_radius_m: 0.35
  min_support_points: 8
  min_vertical_extent_m: 0.45
  max_horizontal_footprint_m: 0.90
  max_radial_mad_m: 0.18
  max_center_displacement_m: 0.35
  max_frame_interval_s: 0.20
  lost_timeout_s: 0.75
  desired_distance_m: 1.00
  stop_distance_m: 0.80
```

For body point `(x,y,z)`, physical height is `z + lidar_height_m`. Filter non-finite points, radial/height ROI, then the inclusive self box, then one deterministic 5 cm voxel downsample. `PreprocessedCloud.source_indices[i]` is the lowest raw index in voxel `i`; all target masks and obstacle exclusion use the downsampled index domain.

The initial click gate is `hypot(x-click_x, y-click_y) <= web_pick_radius_m`. The dynamic gate uses radial/tangential axes defined by the last accepted centre vector `u` and its left normal `v`:

```python
delta = point_xy - last_center_xy
inside = (
    (dot(delta, u) / gate_radial_radius_m) ** 2
    + (dot(delta, v) / gate_tangential_radius_m) ** 2
    <= 1.0
)
```

Accept support only when all configured count/extent/footprint/MAD checks pass. Centre is component-wise median. A failed new click leaves the old candidate unchanged; a failed dynamic tracking gate produces no observation for that frame and advances the lost timer.

The measurements are exact: `vertical_extent_m = max(z)-min(z)`, `horizontal_footprint_m = max(max(x)-min(x), max(y)-min(y))`, radii are `hypot(x,y)`, and `radial_mad_m = median(abs(radius-median(radius)))`. An absent or rejected support set has `c_t=c_g=0`.

### ALX transform and confidence

```yaml
calibration:
  x_m: 0.0
  y_m: 0.0
  yaw_deg: 0.0
  azimuth_sign: 1
  azimuth_zero_offset_deg: 0.0
  lidar_height_m: 1.25
  alx_height_m: 1.10
  allow_provisional_calibration_for_prototype: true
confidence:
  weight_temporal: 0.35
  weight_geometry: 0.30
  weight_alx: 0.35
  alx_body_to_wrist_tolerance_m: 0.80
  no_observation_baseline: 0.0
  start: 0.80
  stop: 0.70
  unlock: 0.45
```

Validate ALX as finite, within configured distance, `abs(azimuth_deg) <= 180`, `abs(elevation_deg) <= 90`, error-free, fresh, and matching `session_tag_id`. Deduplicate by exact `ALXSnapshot.received_at`. For a cloud, select only the newest buffered observation with `alx.received_at <= cloud.received_at_s` and skew within 0.15 s; never use a future sample.

```python
alpha = azimuth_sign * radians(raw_azimuth_deg + azimuth_zero_offset_deg)
raw_xy = np.array([distance_m * cos(alpha), distance_m * sin(alpha)])
alx_body_xy = np.array([x_m, y_m]) + rotation(radians(yaw_deg)) @ raw_xy
```

`lidar_height_m` is used only for physical-height filtering. `alx_height_m` is explicit provisional metadata for later 3-D calibration and status/logging; this prototype's ALX association is deliberately planar and does not subtract the configured heights.

For accepted support:

```python
c_t = 1.0 if selected_seed_or_accepted_track_within_displacement_and_interval else 0.0
p = clip(point_count / (2.0 * min_support_points), 0.0, 1.0)
h = clip(vertical_extent_m / (2.0 * min_vertical_extent_m), 0.0, 1.0)
q = 1.0 - clip(radial_mad_m / max_radial_mad_m, 0.0, 1.0)
c_g = 0.40 * p + 0.30 * h + 0.30 * q
c_a = clip(1.0 - norm(alx_body_xy - target_center_xy) / alx_body_to_wrist_tolerance_m, 0.0, 1.0)
C = clip(0.35 * c_t + 0.30 * c_g + 0.35 * c_a, 0.0, 1.0)
```

For the selected click seed, `c_t=1`. For later frames it is 1 only when support is accepted, `center_displacement <= 0.35`, and `frame_interval <= 0.20`; otherwise it is 0.

Absent/unsynchronized ALX sets `c_a=0` and independently causes soft stop once stale. Start requires fresh synchronized ALX and `C >= 0.80`. While following, `C < 0.70` enters `CONFIDENCE_STOP`; `C < 0.45` or 0.75 s without accepted support unlocks. Soft-stop recovery requires the same lock, fresh post-stop cloud/support, fresh synchronized ALX, and `C >= 0.80`.

### APF, swept direction, and output

```yaml
apf:
  target_desired_distance_m: 1.00
  target_stop_distance_m: 0.80
  non_target_influence_m: 1.20
  non_target_hard_stop_clearance_m: 0.60
  body_half_length_m: 0.30
  body_half_width_m: 0.25
  swept_margin_m: 0.10
  swept_min_distance_m: 0.40
  swept_horizon_s: 1.00
  attraction_gain: 0.60
  repulsion_gain: 0.02
  repulsion_cap: 0.25
  candidate_angles_deg: [0, 30, -30, 60, -60, 90, -90]
  turn_in_place_bearing_deg: 35.0
  omega_gain: 0.80
output:
  max_vx: 0.10
  max_abs_vy: 0.08
  max_abs_omega: 0.25
  accel_vx: 0.15
  decel_vx: 0.30
  accel_vy: 0.15
  decel_vy: 0.30
  accel_omega: 0.50
  decel_omega: 1.00
```

Target points are excluded by the accepted downsampled target mask. Target distance is the minimum planar norm of accepted support; `<=0.80 m` returns zero. For each non-target point, body-envelope clearance is:

```python
dx = max(abs(x) - body_half_length_m, 0.0)
dy = max(abs(y) - body_half_width_m, 0.0)
clearance = hypot(dx, dy)
```

Any clearance `<=0.60 m` returns zero. Otherwise:

```python
r = norm(target_center_xy)
u = target_center_xy / r
F_att = attraction_gain * max(r - target_desired_distance_m, 0.0) * u
F_rep = sum(
    repulsion_gain * (1.0/d - 1.0/non_target_influence_m) / d**2 * (-point_xy/d)
    for each non_target point with 0 < d < non_target_influence_m
)
F_rep = cap_norm(F_rep, repulsion_cap)
F = F_att + F_rep
```

If target bearing exceeds 35 degrees, request zero translation plus `omega=clip(omega_gain*bearing_rad, -0.25, 0.25)` only when every non-target point is outside the rotation radius `hypot(body_half_length_m, body_half_width_m) + swept_margin_m`. Otherwise set `requested_speed=min(norm(F), hypot(max_vx,max_abs_vy))`, order configured candidate headings by angular difference from `atan2(F_y,F_x)`, and calculate each candidate as:

```python
vx = clip(requested_speed * cos(theta), 0.0, max_vx)
vy = clip(requested_speed * sin(theta), -max_abs_vy, max_abs_vy)
omega = clip(omega_gain * target_bearing_rad, -max_abs_omega, max_abs_omega)
sweep_distance = max(swept_min_distance_m, hypot(vx, vy) * swept_horizon_s)
e = np.array([cos(theta), sin(theta)])
n = np.array([-sin(theta), cos(theta)])
longitudinal = obstacle_xy @ e
lateral = obstacle_xy @ n
unsafe = (
    (-body_half_length_m <= longitudinal)
    & (longitudinal <= body_half_length_m + sweep_distance + swept_margin_m)
    & (abs(lateral) <= body_half_width_m + swept_margin_m)
).any()
```

Choose the first safe candidate; if none is safe, return zero. Never emit negative `vx`.

The limiter uses elapsed time between accepted clouds, clamped to `[0.001,0.10]` s. Mandatory stop calls `reset()` and dispatches a `MotionCommand(kind="stop", vx=vy=omega=0)` without slew limiting.

### Web and logging bounds

```yaml
web:
  host: 127.0.0.1
  port: 8080
  max_clients: 8
  max_message_bytes: 4096
  max_events_per_second: 20
  status_hz: 10.0
  max_visualization_points: 5000
logging:
  enabled: true
  csv_path: logs/lidar_alx_assisted_follow.csv
  flush_interval_s: 1.0
```

Status serialization deterministically strides preprocessed points to at most 5000 and always includes accepted target points separately. Slow Web clients skip old status versions. CSV uses `record_kind=strategy|safety_stop`; repeated identical stop reasons do not create 40 Hz duplicate rows.

Client messages use these exact key sets:

```json
{"type":"select_target","x":1.2,"y":-0.3}
{"type":"start_follow"}
{"type":"stop_follow"}
```

The server sends role once, then versioned status frames:

```json
{"type":"role","payload":{"role":"controller"}}
{"type":"frame","payload":{
  "version":1,
  "cloud_sequence":42,
  "frame_id":"body",
  "points":[[1.0,0.2,0.4]],
  "target_points":[[1.1,0.1,0.8]],
  "target_gate":{"kind":"ellipse","center_xy":[1.1,0.1],"radial_radius_m":0.45,"tangential_radius_m":0.35},
  "alx":{"body_xy":[1.0,0.0],"age_s":0.04,"synchronized":true},
  "lifecycle_state":"sensors_ready",
  "lock_state":"following",
  "confidence":{"c_t":1.0,"c_g":0.8,"c_a":0.9,"total":0.91},
  "stop":{"requested":false,"kind":null,"reason":null},
  "dispatched_command":{"vx":0.08,"vy":0.0,"omega":0.05,"reason":"follow"},
  "calibration_status":"provisional"
}}
```

Unavailable optional values are JSON `null`, never NaN. Error replies are `{"type":"error","payload":{"code":"read_only","message":"observer cannot control"}}`; `code` is one of `invalid_json`, `invalid_schema`, `message_too_large`, `rate_limited`, `read_only`, `sensors_unavailable`, or `invalid_state`.

The CSV header is exactly:

```text
record_kind,wall_time_iso,monotonic_s,cloud_sequence,cloud_age_ms,alx_age_ms,
lifecycle_state,lock_state,target_center_x,target_center_y,target_support_count,
c_t,c_g,c_a,confidence,stop_reason,apf_requested_vx,apf_requested_vy,
apf_requested_omega,dispatched_vx,dispatched_vy,dispatched_omega,
strategy_compute_ms,g1_dispatch_ms,bridge_state,sdk_error,calibration_status
```

---

## File Structure

```text
src/robot_follow/lidar_alx_assisted_follow/
  __init__.py
  cli.py
  domain/
    __init__.py
    models.py
  config/
    __init__.py
    schema.py
    loader.py
    adapters.py
  sensors/
    __init__.py
    cloud.py
    alx.py
    lifecycle.py
  tracking/
    __init__.py
    preprocessing.py
    target.py
    confidence.py
  motion/
    __init__.py
    apf.py
    limiter.py
    dispatch.py
  runtime/
    __init__.py
    state.py
    csv_log.py
    controller.py
  web/
    __init__.py
    protocol.py
    server.py
  frontend/
    index.html
    package.json
    package-lock.json
    tsconfig.json
    src/main.ts
    src/styles.css
    dist/index.html
    dist/assets/*
tests/lidar_alx_assisted_follow/
  config/
    test_config.py
  sensors/
    test_cloud.py
    test_alx.py
    test_lifecycle.py
  tracking/
    test_preprocessing.py
    test_target.py
    test_confidence.py
  motion/
    test_apf.py
    test_limiter.py
    test_dispatch.py
  runtime/
    test_state.py
    test_csv_log.py
    test_controller.py
  web/
    test_protocol.py
    test_server.py
  test_cli.py
configs/jetson_g1_lidar_alx_assisted_follow.yaml
```

The package follows capability boundaries rather than a flat technical-layer layout:

- `domain` owns only frozen cross-component contracts and enums; it has no dependency on the other prototype packages.
- `config` owns strict YAML schema/validation, loading, and adapters into unchanged existing radar/G1 configuration types.
- `sensors` owns LiDAR sequencing, causal ALX buffering/transforms, and the wristband-driven sensor lifecycle.
- `tracking` owns cloud preprocessing, the single local target lock, and confidence computation/state transitions.
- `motion` owns APF planning, slew limiting, and the sole G1 dispatch boundary.
- `runtime` owns thread-safe shared state, CSV recording, watchdog behavior, and control-loop orchestration.
- `web` owns wire-schema validation and aiohttp connection/lease/status transport; it does not call sensors or G1 directly.
- `cli.py` is the composition root only. `frontend` stays independent because it has its own toolchain and build output.

Imports point inward through these contracts: `domain` is dependency-free; capability packages may depend on `domain` and `config`; `runtime` may orchestrate all capability packages; `web` may depend on `domain` and `runtime.state`; only `cli.py` wires concrete runtime, Web, sensor, and output objects together. Tests mirror the source capability folders so ownership remains obvious.

Only `pyproject.toml` and `README.md` outside those paths are modified. Existing AO frontend, ALX source/reader, FAST-LIO source, radar manager, G1 bridge, and their tests remain unchanged.

### Task 0: Freeze the Python 3.8/Foxy execution contract

**Files:**
- Verify: `pyproject.toml`
- Verify: `vendor/unitree_sdk2_python/setup.py`
- Document later in Task 7: `README.md`

- [ ] **Step 1: Verify the selected development interpreter before coding**

Run: `conda run -n robot_followao_radar python --version`

Expected: `Python 3.8.x`. If it is not 3.8, stop; do not silently implement or test this mode under another interpreter.

- [ ] **Step 2: Verify the pure-Python development dependencies without importing hardware adapters**

Run: `conda run -n robot_followao_radar python -c "import aiohttp, numpy, serial, yaml; print(aiohttp.__version__, numpy.__version__)"`

Expected: imports succeed. Do not import `rclpy`, open serial, initialize DDS, or install Unitree dependencies on the development machine as part of this task.

- [ ] **Step 3: Preserve the package and SDK version contracts**

Confirm `pyproject.toml` remains `requires-python = ">=3.8"`, and confirm the vendored Unitree SDK still declares Python `>=3.8` and `cyclonedds==0.10.2`. All new modules and tests must be parseable and runnable by CPython 3.8.

- [ ] **Step 4: Define, but do not execute, the Jetson qualification gate**

Task 7 must document an on-device preflight that records Ubuntu, L4T/JetPack, architecture, ROS distribution, `/usr/bin/python3`, the selected environment interpreter, and imports of ROS message packages plus Unitree/CycloneDDS. It must also include a real `/cloud_registered_body` receive smoke test and a mock/stop-only G1 bridge check before any nonzero motion. These are deployment qualification commands and remain unexecuted during implementation.

### Task 1: Define strict configuration, immutable types, and cloud sequencing

**Files:**
- Create: `src/robot_follow/lidar_alx_assisted_follow/__init__.py`
- Create: `src/robot_follow/lidar_alx_assisted_follow/domain/{__init__,models}.py`
- Create: `src/robot_follow/lidar_alx_assisted_follow/config/{__init__,schema,loader,adapters}.py`
- Create: `src/robot_follow/lidar_alx_assisted_follow/sensors/{__init__,cloud}.py`
- Create: `configs/jetson_g1_lidar_alx_assisted_follow.yaml`
- Modify: `pyproject.toml`
- Test: `tests/lidar_alx_assisted_follow/config/test_config.py`
- Test: `tests/lidar_alx_assisted_follow/sensors/test_cloud.py`

**Interfaces:**
- Consumes: `robot_follow.ao.config.load_yaml_config`, `FastLIOPointCloudSource`, `G1BridgeConfig`.
- Produces: `PrototypeConfig`, `CloudSnapshot`, `ALXObservation`, `MotionIntent`, `StrategyResult`, `SequencedFastLIOPointCloudSource.wait_for_update(timeout_s)`, `build_radar_service_config`, and `build_g1_bridge_config`.

- [ ] **Step 1: Write failing strict-config and cloud-event tests**

```python
def test_unknown_nested_key_is_rejected(tmp_path):
    path = write_yaml(tmp_path, "lidar_alx_assisted_follow:\n  output:\n    typo: 1\n")
    with pytest.raises(ValueError, match="unknown keys.*output.*typo"):
        load_prototype_config(path)

def test_each_source_update_gets_one_sequence_and_sets_event():
    source = SequencedFastLIOPointCloudSource()
    source.update_for_test(body_frame(received_at_s=10.0))
    assert source.wait_for_update(0.0).sequence == 1
    assert source.latest_snapshot().sequence == 1
```

- [ ] **Step 2: Implement frozen nested config and exact validation**

Reject non-finite numbers, non-positive timeouts/budgets, negative clearances/speeds, invalid signs, weights not summing to 1 within `1e-9`, `start <= stop`, `stop <= unlock`, and non-body expected frame. Use `str, Enum` enums. Non-loopback authorization is a CLI concern in Task 7, not config parsing.

- [ ] **Step 3: Implement the event source without modifying FAST-LIO code**

Subclass `FastLIOPointCloudSource`; override its production-used `update_for_test(frame)` hook to call `super()`, increment a lock-protected sequence, store `CloudSnapshot(received_at_s=frame.received_at_s, ...)`, and set `threading.Event`. `wait_for_update` clears the event after copying the latest immutable snapshot.

- [ ] **Step 4: Add YAML and console entry point**

Add `robot-follow-lidar-alx-assisted-follow = "robot_follow.lidar_alx_assisted_follow.cli:main"`. Populate every value from the Exact Prototype Defaults section plus deployment launch commands, serial port, `eth0`, and mock-by-default output.

Implement explicit adapters from prototype sensor/G1 sections into unchanged `AORadarServiceConfig` and `G1BridgeConfig`; do not rely on undocumented duck typing.

Add `PyYAML>=6.0` to project dependencies because strict nested mappings and `candidate_angles_deg` must not rely on the scalar-only fallback parser.

- [ ] **Step 5: Run only the new tests after conda confirmation**

Run: `conda run -n robot_followao_radar pytest tests/lidar_alx_assisted_follow/config/test_config.py tests/lidar_alx_assisted_follow/sensors/test_cloud.py -v`

Expected: all tests PASS; no existing test path is collected.

### Task 2: Implement shared STOP state, ALX causality, and lifecycle

**Files:**
- Create: `src/robot_follow/lidar_alx_assisted_follow/runtime/{__init__,state}.py`
- Create: `src/robot_follow/lidar_alx_assisted_follow/sensors/{alx,lifecycle}.py`
- Test: `tests/lidar_alx_assisted_follow/runtime/test_state.py`
- Test: `tests/lidar_alx_assisted_follow/sensors/{test_alx,test_lifecycle}.py`

**Interfaces:**
- Produces: `StopLatch.request`, `StopLatch.clear_for_start`, `StopLatch.clear_soft_recovery`, `ControlEventInbox`, `LatestStatus`, `ALXSnapshotBuffer.for_cloud`, `WristbandLifecycleGuardian.tick`.

- [ ] **Step 1: Write failing STOP ownership tests**

```python
def test_any_thread_can_request_but_only_control_thread_can_clear():
    latch = StopLatch(control_thread_id=threading.get_ident())
    latch.request("controller_disconnected", StopKind.HARD)
    assert latch.snapshot().requested
    latch.clear_for_start(start_after_sequence=8)
    assert not latch.snapshot().requested
    assert latch.snapshot().start_after_sequence == 8

def test_nonzero_requires_post_start_cloud():
    latch = cleared_latch(start_after_sequence=8)
    assert not latch.motion_allowed_for(sequence=8)
    assert latch.motion_allowed_for(sequence=9)
```

- [ ] **Step 2: Implement bounded event/status state**

`ControlEventInbox` holds 32 events. Coalesce pending `select_target` to the newest. `stop_follow` and disconnect call `StopLatch.request` synchronously before enqueue; if full, remove a select event first. `LatestStatus` replaces rather than queues status frames.

Test and implement priority: an existing HARD request cannot be replaced or auto-cleared by a later SOFT request; a later HARD request replaces SOFT. User Stop, controller loss, lifecycle/bridge/cloud faults are HARD. Only `confidence_low` and `alx_stale` are SOFT.

- [ ] **Step 3: Implement causal ALX buffering using source receipt time**

Append only when `snapshot.received_at` changes; never call `time.monotonic()` as a replacement timestamp. Buffer one second, reject future/excess-skew samples and tag mismatch, and return `body_xy=None` when unavailable.

- [ ] **Step 4: Implement lifecycle transitions and one-time G1 init**

States are `WRISTBAND_OFF`, `STARTING_SENSORS`, `SENSORS_READY`, `FAULT`. OFF/serial error are hard stops with unlock/tag clear/service stop. ALX stale after readiness is not a lifecycle transition. `bridge.init()` occurs only on the first fresh body cloud in the process; later sessions reuse STOPPED/INITIALIZED bridge. Ready timeout enters FAULT and stops services.

- [ ] **Step 5: Run only Task 2 tests**

Run: `conda run -n robot_followao_radar pytest tests/lidar_alx_assisted_follow/runtime/test_state.py tests/lidar_alx_assisted_follow/sensors/test_alx.py tests/lidar_alx_assisted_follow/sensors/test_lifecycle.py -v`

Expected: PASS.

### Task 3: Implement local target tracking and confidence state machine

**Files:**
- Create: `src/robot_follow/lidar_alx_assisted_follow/tracking/{__init__,preprocessing,target,confidence}.py`
- Test: `tests/lidar_alx_assisted_follow/tracking/{test_preprocessing,test_target,test_confidence}.py`

**Interfaces:**
- Produces: `preprocess_tracking_cloud`, `TargetLockManager.select/update/start/stop/unlock`, `compute_confidence`.

- [ ] **Step 1: Write tests for index-domain exclusion and failed-gate semantics**

```python
def test_target_mask_is_aligned_to_downsampled_points():
    cloud = preprocess_tracking_cloud(raw_points, config)
    obs = manager.select(cloud, click_xy=(1.0, 0.0), now_s=1.0)
    assert obs.target_mask.shape == (cloud.points_xyz.shape[0],)
    assert np.all(cloud.source_indices >= 0)

def test_failed_reselection_keeps_candidate_but_failed_tracking_advances_loss():
    before = manager.snapshot
    rejected = manager.select(empty_cloud, click_xy=(4.0, 0.0), now_s=2.0)
    assert rejected.snapshot == before
    tracked = manager.update(empty_cloud, now_s=2.1)
    assert tracked.observation is None
    assert tracked.last_seen_at_s == before.last_seen_at_s
```

- [ ] **Step 2: Implement preprocessing and exact gate checks**

Use the equations/defaults above. The dynamic ellipse is radial/tangential, not world-axis aligned. Do not enumerate any other cluster.

- [ ] **Step 3: Implement confidence and two-level recovery**

Implement exact `c_t/c_g/c_a/C`. `stop_follow` returns to CANDIDATE and preserves tracking. Soft stop may auto-resume only on the same lock; every hard stop sets `requires_start=True`. LOST clears the target.

- [ ] **Step 4: Run only Task 3 tests**

Run: `conda run -n robot_followao_radar pytest tests/lidar_alx_assisted_follow/tracking/test_preprocessing.py tests/lidar_alx_assisted_follow/tracking/test_target.py tests/lidar_alx_assisted_follow/tracking/test_confidence.py -v`

Expected: PASS.

### Task 4: Implement APF, swept checks, limiter, and final output adapter

**Files:**
- Create: `src/robot_follow/lidar_alx_assisted_follow/motion/{__init__,apf,limiter,dispatch}.py`
- Test: `tests/lidar_alx_assisted_follow/motion/{test_apf,test_limiter,test_dispatch}.py`

**Interfaces:**
- Produces: `compute_apf`, `CommandSlewRateLimiter`, `dispatch_strategy_or_stop`.

- [ ] **Step 1: Write stop-precedence and no-reverse tests**

```python
def test_target_is_excluded_from_repulsion_but_too_close_stops():
    result = compute_apf(target_at(0.75), cloud_with_only_target(), following_confidence(), config)
    assert result.stop_reason == "target_too_close"
    assert result.intent == zero_intent()

def test_stop_latch_wins_at_last_dispatch_boundary():
    latch, bridge = cleared_latch(8), FakeBridge()
    latch.request("controller_disconnected", StopKind.HARD)
    dispatch_strategy_or_stop(move(sequence=9), latch=latch, bridge=bridge, limiter=limiter)
    assert bridge.last.kind == "stop"
```

- [ ] **Step 2: Implement exact APF and swept candidate order**

Use the equations/defaults above. Validate every input as finite. Stop precedence is: latch/lifecycle/control ownership, invalid/stale input, target too close, non-target hard clearance, non-FOLLOWING confidence, unsafe rotation/all directions, ordinary motion.

- [ ] **Step 3: Implement limiter and `MotionCommand` conversion**

The adapter converts `MotionIntent` to `robot_follow.core.types.MotionCommand`; this is the only prototype location that calls `bridge.dispatch`. Immediately before nonzero dispatch, re-read `StopLatch` and require `sequence > start_after_sequence`.

- [ ] **Step 4: Run only Task 4 tests**

Run: `conda run -n robot_followao_radar pytest tests/lidar_alx_assisted_follow/motion/test_apf.py tests/lidar_alx_assisted_follow/motion/test_limiter.py tests/lidar_alx_assisted_follow/motion/test_dispatch.py -v`

Expected: PASS.

### Task 5: Assemble cloud-driven runtime and truthful CSV logging

**Files:**
- Create: `src/robot_follow/lidar_alx_assisted_follow/runtime/{csv_log,controller}.py`
- Test: `tests/lidar_alx_assisted_follow/runtime/{test_csv_log,test_controller}.py`

**Interfaces:**
- Produces: `PrototypeCSVLogger`, `LidarALXAssistedFollowRuntime.run_control_loop`, `watchdog_tick`.

- [ ] **Step 1: Write cloud-frequency and watchdog tests**

```python
def test_strategy_runs_once_per_sequence_not_at_watchdog_rate():
    runtime.publish_cloud(cloud(sequence=1))
    runtime.control_iteration(now_s=1.000)
    runtime.control_iteration(now_s=1.025)
    runtime.control_iteration(now_s=1.050)
    assert runtime.strategy_sequences == [1]

def test_clouds_arriving_while_busy_collapse_to_latest():
    runtime.publish_cloud(cloud(sequence=2))
    runtime.publish_cloud(cloud(sequence=3))
    runtime.control_iteration(now_s=1.075)
    assert runtime.strategy_sequences[-1] == 3
```

- [ ] **Step 2: Implement the control loop**

Wait for cloud event or 25 ms timeout; drain controls; sample/deduplicate ALX; update lifecycle; process newest unseen cloud; run watchdog; call `bridge.tick(now)` after dispatch. Web Stop/disconnect has already set the latch even if strategy is computing.

- [ ] **Step 3: Implement CSV schemas without `actual_v`**

Write `strategy` rows per processed cloud and `safety_stop` rows only when reason changes. Fields are those in the logging defaults section, including `apf_requested_*`, `dispatched_*`, bridge state/error, latency, and `calibration_status`. Catch `OSError`, log once, disable CSV, and continue safety/control operation.

- [ ] **Step 4: Run only Task 5 tests**

Run: `conda run -n robot_followao_radar pytest tests/lidar_alx_assisted_follow/runtime/test_csv_log.py tests/lidar_alx_assisted_follow/runtime/test_controller.py -v`

Expected: PASS.

### Task 6: Implement bounded Web lease/protocol and independent frontend

**Files:**
- Create: `src/robot_follow/lidar_alx_assisted_follow/web/{__init__,protocol,server}.py`
- Create: `src/robot_follow/lidar_alx_assisted_follow/frontend/*`
- Test: `tests/lidar_alx_assisted_follow/web/{test_protocol,test_server}.py`

**Interfaces:**
- Produces: `PrototypeWebHub`, `create_app`, `run_web_server`, independent Vite build.

- [ ] **Step 1: Write lease, bounds, and disconnect tests**

```python
def test_controller_disconnect_stops_and_unlocks_without_promoting_observer():
    hub.connect("a")
    hub.connect("b")
    hub.disconnect("a")
    assert hub.stop_latch.snapshot().reason == "controller_disconnected"
    assert hub.controller_id is None
    assert hub.role_of("b") == "observer"

def test_new_connection_after_vacancy_becomes_controller():
    hub = hub_after_controller_disconnect_with_observer()
    assert hub.connect("c").role == "controller"
```

- [ ] **Step 2: Implement strict protocol and bounded broadcast**

Accept only exact JSON key sets for the three events; reject booleans as numbers, non-finite/out-of-range clicks, messages over 4096 bytes, observers, unavailable sensors, and rate over 20 events/s. Limit clients to eight. A 10 Hz aiohttp task reads latest immutable status and sends only newer versions; close clients whose send exceeds one status period.

- [ ] **Step 3: Build the independent UI**

Render 360-degree cloud, purple gate, purple-white target points, yellow-green fresh ALX, software-dispatched command, lifecycle/lock/confidence/stop reason, Start/Stop buttons, role, and persistent red `PROVISIONAL CALIBRATION`. Do not show the unauthenticated-LAN warning in the page.

- [ ] **Step 4: Run new Web tests and frontend checks**

Run: `conda run -n robot_followao_radar pytest tests/lidar_alx_assisted_follow/web/test_protocol.py tests/lidar_alx_assisted_follow/web/test_server.py -v`

Run from `src/robot_follow/lidar_alx_assisted_follow/frontend`: `npm ci`

Then run: `npm run typecheck`

Then run: `npm run build`

Expected: pytest PASS, TypeScript exits 0, Vite emits independent `dist/index.html` and hashed assets.

### Task 7: Implement CLI safety gates and documentation

**Files:**
- Create: `src/robot_follow/lidar_alx_assisted_follow/cli.py`
- Modify: `README.md`
- Test: `tests/lidar_alx_assisted_follow/test_cli.py`

- [ ] **Step 1: Write CLI gate tests**

```python
def test_real_provisional_motion_requires_both_confirmations():
    with pytest.raises(SystemExit, match="confirm-real-g1"):
        build_runtime(real_args(allow_provisional=True, confirm_real=False))
    with pytest.raises(SystemExit, match="allow-provisional-calibration"):
        build_runtime(real_args(allow_provisional=False, confirm_real=True))

def test_non_loopback_requires_lan_confirmation():
    with pytest.raises(SystemExit, match="allow-unauthenticated-lan"):
        build_runtime(args(host="0.0.0.0", allow_lan=False))
```

- [ ] **Step 2: Implement startup/shutdown order**

Start ALX reader, sequenced cloud reader, control thread, then aiohttp. Sensors and G1 are lifecycle-gated. In `finally`: request STOP, join control thread after its zero attempt, close bridge, stop services, stop sources, close logger, and clean aiohttp. Print LAN risk only in terminal logs.

- [ ] **Step 3: Document operation, Jetson qualification, and residual risks**

README includes mock command, real provisional command, first-client rule, localhost/LAN flags, 1 s G1 command-duration residual risk, absence of physical velocity feedback, calibration banner, tests allowed, and hardware validation still required.

Add a separate Jetson-only preflight block, clearly marked **not executed during implementation**, that:

1. sources `/opt/ros/foxy/setup.bash` and the deployed FAST-LIO workspace;
2. records `/etc/os-release`, `uname -m`, `/etc/nv_tegra_release`, `ROS_DISTRO`, `RMW_IMPLEMENTATION`, `/usr/bin/python3 --version`, and `conda run -n robot_followao_radar python --version`;
3. requires Ubuntu 20.04, `aarch64`, `ROS_DISTRO=foxy`, and Python 3.8 for both the system and selected runtime;
4. runs one selected-interpreter import smoke for `rclpy`, `PointCloud2`, `sensor_msgs_py.point_cloud2`, NumPy, aiohttp, serial, YAML, `unitree_sdk2py`, and `cyclonedds`;
5. verifies that `/cloud_registered_body` delivers exact `frame_id == "body"` through the same Python interpreter used by the application;
6. records the ROS RMW implementation and Unitree CycloneDDS version. If ROS also uses CycloneDDS, qualification must prove that its native library and Unitree's required `cyclonedds==0.10.2` coexist; otherwise stop and resolve the DDS library conflict before motion;
7. runs CLI mock mode and G1 stop-only qualification before the two real-motion confirmation flags may be used.

Failure of any preflight item is a deployment blocker. The fallback is the system Python 3.8 `venv --system-site-packages` route or a complete ROS rebuild against the selected interpreter—not a Python-only upgrade and not a PyPI `rclpy` install.

- [ ] **Step 4: Run only CLI tests**

Run: `conda run -n robot_followao_radar pytest tests/lidar_alx_assisted_follow/test_cli.py -v`

Expected: PASS without importing Unitree SDK or opening serial/ROS resources.

### Task 8: Verify the isolated prototype

**Files:**
- Verify all files listed above.

- [ ] **Step 1: Run the complete new-mode Python suite only**

Run: `conda run -n robot_followao_radar pytest tests/lidar_alx_assisted_follow/ -v`

Expected: PASS; collected paths are all under `tests/lidar_alx_assisted_follow/`.

- [ ] **Step 2: Run independent frontend verification**

Run from its frontend directory: `npm run typecheck`

Then run: `npm run build`

Expected: exit 0 and updated independent dist.

- [ ] **Step 3: Run static isolation checks**

Run `git diff --check` and use `rg` to prove the new package has no imports of `AORadarRuntime`, `FollowPipeline`, `TransformConfig`, `ConstantCommandSource`, AO avoidance planner, or AO frontend paths. Confirm `git diff --name-only` does not add modifications to the user's pre-existing dirty AO/G1 files.

- [ ] **Step 4: Handoff honestly**

Report exact files, test/build results, unexecuted existing/hardware suites, provisional calibration, lack of physical velocity feedback, SDK non-preemptibility, and the required on-site stop-injection/calibration checklist. Do not claim field safety.

## Self-Review

- Spec coverage: isolated mode/config/frontend, body-frame input, wristband lifecycle, causal ALX, local target-only tracking, confidence recovery, APF/swept checks, no reverse, STOP latch, Web lease, logging, CLI gates, and focused verification each map to a task.
- Placeholder scan: no implementation decisions are deferred; formulas, thresholds, protocols, bounds, state recovery, and commands are explicit.
- Type consistency: `CloudSnapshot.sequence`, downsampled target masks, `MotionIntent`, `StrategyResult`, `StopLatch`, and `MotionCommand` conversion retain one meaning across tasks.
- Existing-code safety: the plan does not modify ALX source/reader, AO frontend/runtime, FAST-LIO source, radar manager, G1 bridge, or their tests.
- Verification boundary: only the new test directory and independent frontend are executed after conda environment confirmation.

## Execution Handoff

Plan execution must use `superpowers:subagent-driven-development` or `superpowers:executing-plans`. Git commits are not made per task in this dirty worktree; after all focused verification passes and the user confirms the implementation is final, `commit-on-final-confirmation` may create one scoped checkpoint containing only the new mode, its config/tests/frontend, `pyproject.toml`, and README changes.
