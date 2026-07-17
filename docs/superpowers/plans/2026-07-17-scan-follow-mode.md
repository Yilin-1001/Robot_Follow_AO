# Lightweight SCAN-Inspired Follow Mode Implementation Plan

Status: Draft. This replaces the earlier rolling-map plan after the decision to use frame-local planning only. Implementation requires plan approval first.

Goal: Add a startup-selected scan mode alongside APF to lidar_alx_assisted_follow. The mode uses only each fresh FAST-LIO body-frame point cloud and current ALX body-frame target to build a temporary local 2.5D grid, run capsule-aware local A*, and generate a lookahead velocity command.

Architecture: Keep the current cloud-driven controller, AvoidanceStrategy factory, StopLatch, output limiter, and G1 bridge. Every fresh cloud still triggers a control cycle. ScanAvoidanceStrategy rebuilds a local grid each cycle and keeps only the latest accepted ALX target version plus the short-lived command created by the last successful A*. A new ALX sample triggers normal A*; intermediate cloud cycles validate the held command direction against the current grid. A collision invalidation triggers one emergency A* using the held target. The grid and A* path are discarded before the next fresh cloud. CUDA GPU acceleration is the default compute path for scan-mode point filtering, grid construction, and batched footprint collision queries; the serial A* frontier remains CPU-controlled unless profiling later justifies a separate GPU search implementation.

Tech stack: Python 3.8-compatible dataclasses, NumPy host interfaces, CuPy/CUDA device kernels, existing YAML loader, pytest. CUDA GPU is required by default for scan mode; an explicit CPU backend is permitted only for development and diagnosis. No FAST-LIO Odometry subscription, persistent world frame, accumulated map, or upstream C++ or ROS1 SCAN node.

## Locked decisions

- avoidance.mode is selected only before startup: apf or scan. Runtime and Web mode switching are out of scope.
- scan.compute_backend is startup-only and defaults to cuda. CPU is an explicit diagnostic/development selection, never a silent fallback. If cuda is selected but its compatible GPU, driver, CUDA runtime, or required GPU arrays/kernels cannot initialise, scan-mode startup fails closed with a clear configuration error.
- scan is a Python, SCAN-inspired frame-local 2.5D planner, not a wrapper for upstream C++ SCAN.
- Input is only the current valid cloud from /cloud_registered_body and current ALX target in the existing body convention. Do not subscribe to world pose or transform points into world coordinates.
- Every fresh cloud creates a new local occupancy grid. No obstacle, free-space evidence, path, or map state is retained after that planning cycle.
- The controller remains point-cloud-event driven. It accepts a new ALX target by sample version or timestamp, not by a brittle fixed cloud-frame count. With a 10 Hz ALX stream and a roughly 10-times-faster cloud stream, one target normally governs about ten cloud cycles.
- A new ALX target triggers ordinary A* planning. Cloud cycles with no new target do not repeat A* for the same goal; they retain only the last safe command and verify its immediate capsule sweep against the current grid. They do not reuse a former body-frame path.
- If a current cloud blocks the held command direction, output zero immediately and trigger one emergency A* from that cloud with the most recent fresh ALX target. If it cannot plan safely, keep zero until a later valid trigger.
- Collision uses a yaw-aware capsule or twin-disc model of G1 measured outer hull plus configurable margin.
- The wearer is a synthetic dynamic keep-out region. The goal is a stand-off point at desired follow distance, not the wearer position.
- A* returns a local path. A body-frame lookahead controller produces vx, vy, and omega. It does not track a time-parameterized trajectory.
- The existing 20 ms strategy budget is mandatory.
- no_path sends zero for that cloud and retries on next fresh cloud. invalid local grid and planner_timeout hard-stop, disable follow, and require explicit Start. No automatic APF fallback.
- Existing dirty worktree changes are outside this plan and MUST be preserved.

## Target data flow

Fresh body-frame cloud plus current ALX body target
-> height filter and downsample
-> temporary body-centred 2.5D occupancy grid
-> new ALX target or blocked held command?
   -> yes: wearer keep-out plus ideal stand-off point -> project to safe in-grid local goal -> capsule-aware A* -> lookahead command
   -> no: validate held command immediate sweep
-> MotionIntent to existing limiter, StopLatch, and G1 bridge

## Task 1: Generalize configuration and strategy wiring

Files:

- Modify src/robot_follow/lidar_alx_assisted_follow/config/schema.py
- Modify src/robot_follow/lidar_alx_assisted_follow/config/loader.py
- Modify configs/jetson_g1_lidar_alx_assisted_follow.yaml
- Modify src/robot_follow/lidar_alx_assisted_follow/motion/avoidance/factory.py
- Modify or add config and avoidance strategy tests

- [ ] Permit exactly apf and scan as validated modes. Reject every other value and never silently substitute APF.
- [ ] Add frozen ScanConfig under YAML key scan. It must contain `compute_backend` defaulting to `cuda`, temporary grid dimensions and resolution, collision-relevant vertical band, capsule dimensions and margin, person keep-out radius, heading discretization, A* maximum expanded states, local-goal maximum range, lookahead distance, and speed and yaw gains.
- [ ] Validate non-positive dimensions, invalid resolution, grid too small for footprint plus margin, invalid heading count, invalid desired distance or keep-out radius, and non-positive A* expansion limit.
- [ ] Initialise conservative capsule values from current APF body geometry and swept margin, while keeping SCAN geometry independently YAML-configurable for physical calibration.
- [ ] At startup, validate the requested backend before enabling follow. For the default cuda backend, verify a CUDA-capable device, compatible CuPy runtime, and minimal allocation/kernel smoke check; report device name and runtime version. Do not silently route scan work to CPU when this validation fails.
- [ ] Update factory so APF builds unchanged strategy and scan builds ScanAvoidanceStrategy.

Acceptance: current APF YAML remains backward-compatible; scan YAML loads without any pose topic configuration; factory tests show selected class and mode.

## Task 2: Build a fresh local 2.5D occupancy grid per cloud

Files:

- Add src/robot_follow/lidar_alx_assisted_follow/motion/scan/types.py
- Add src/robot_follow/lidar_alx_assisted_follow/motion/scan/local_grid.py
- Add tests/lidar_alx_assisted_follow/motion/scan/test_local_grid.py

- [ ] Use existing cloud preprocessing and body-axis convention. On the default CUDA path, upload the fresh cloud once, then use CuPy/device operations for finite/range/height filtering, downsampling, and grid-cell indexing; retain device-resident arrays until the current control cycle completes.
- [ ] Allocate or clear a fixed-size device grid centred at body origin for every compute call. Mark cells occupied from this cloud only; no prior cloud observation may influence current occupancy. Provide a feature-equivalent explicit CPU implementation for deterministic tests and diagnosis.
- [ ] Treat unobserved cells according to an explicit conservative policy: unknown cells outside the observed safe local region are not valid for A* traversal.
- [ ] Provide diagnostics for current cloud sequence, grid dimensions, occupied-cell count, usable planning region, and any invalid-grid reason.
- [ ] Do not add world pose, map origin, ray-history, rolling window, map reset, or cross-frame path cache concepts.

Acceptance: repeated calls with different clouds do not retain obstacles from former frames; Y-axis convention remains consistent; height filtering and invalid inputs behave deterministically.

## Task 3: Build capsule collision checks and bounded heading-aware A*

Files:

- Add src/robot_follow/lidar_alx_assisted_follow/motion/scan/footprint.py
- Add src/robot_follow/lidar_alx_assisted_follow/motion/scan/astar.py
- Add tests/lidar_alx_assisted_follow/motion/scan/test_footprint.py
- Add tests/lidar_alx_assisted_follow/motion/scan/test_astar.py

- [ ] Represent G1 as two discs or capsule endpoints in yaw-aligned body frame. Include configured hull dimensions and margin in every collision query; do not use APF rectangle as SCAN collision oracle. Batch candidate footprint samples and occupancy-clearance queries on the default CUDA path; transfer only the compact collision result/cost data needed by the CPU A* frontier.
- [ ] Use planner state grid x, grid y, and heading bin. A state is valid only if the full capsule has current-frame occupancy clearance and vertical clearance.
- [ ] Implement deterministic local A* with bounded open set and configured maximum expanded states. Cost prefers local-goal progress, low heading change, and clearance.
- [ ] Enforce a monotonic 20 ms end-to-end deadline that includes host-to-device upload, device kernels, required synchronization, result transfer, collision, and A* loops. Deadline expiry returns planner_timeout; exhausted frontier returns no_path; malformed or unusable grid returns invalid_grid.
- [ ] Return a body-frame path and planner diagnostics, never a command directly.

Acceptance: tests cover collision at multiple yaw values, passable and impassable corridors, obstacle detour, goal collision, unknown-region rejection, no-path, and deadline expiry.

## Task 4: Model wearer and convert local path to velocity

Files:

- Add src/robot_follow/lidar_alx_assisted_follow/motion/scan/target.py
- Add src/robot_follow/lidar_alx_assisted_follow/motion/scan/lookahead.py
- Add focused tests under tests/lidar_alx_assisted_follow/motion/scan/

- [ ] Build a synthetic person keep-out disc from current ALX body target. It is a current-frame planning constraint, not a point-cloud mask or stored obstacle.
- [ ] Compute the **ideal follow point** in body frame on the robot-facing side of the ALX keep-out using desired follow distance: `ideal = alx_position - unit(robot_to_alx) * desired_follow_distance`. This is the desired destination, not ALX's position.
- [ ] When that ideal follow point lies outside the usable grid, create a virtual local goal by projecting it along the robot-to-ideal direction to the farthest traversable in-grid point, then inset it by a configured safety margin. This is receding-horizon local planning: retain the remote ALX-derived ideal point only as the current direction constraint; do not enlarge or accumulate the grid.
- [ ] Reject only a goal that is inside static occupancy or the person keep-out, or for which no safe in-grid projected goal exists. Report the distinction between `ideal_goal_outside_grid` and a rejected local goal in planner diagnostics.
- [ ] Select the first path point at configured arc-length lookahead. Scale translational command by lookahead distance and clearance, compute yaw correction from bearing, and clamp to existing output_motion limits.
- [ ] If stand-off is reached, return zero MotionIntent without emergency stop. The first commanded segment must pass current-frame capsule validation.
- [ ] If ALX target is too close, preserve existing configured safe behavior explicitly for scan mode; do not inherit APF automatic retreat or escape logic implicitly.

**Command-hold rule:** The result of a successful A* is reduced to a bounded body-frame command for use until the next target update. Do not retain or transform its path across cloud cycles without world pose. Every intermediate cloud must revalidate the command's immediate swept capsule region; failure yields zero and requests emergency A*.

Acceptance: direct target produces forward command; off-axis target produces bounded lateral and yaw command; path never crosses keep-out; arrival produces zero; blocked first segment produces zero or no_path.

## Task 5: Integrate ScanAvoidanceStrategy with cloud-driven runtime

Files:

- Add src/robot_follow/lidar_alx_assisted_follow/motion/avoidance/scan_strategy.py
- Modify src/robot_follow/lidar_alx_assisted_follow/motion/avoidance_types.py only for generic local-grid diagnostics
- Modify src/robot_follow/lidar_alx_assisted_follow/runtime/controller.py
- Modify relevant domain models, CSV logging, Web serialization, and tests

- [ ] Pass current preprocessed cloud, current ALX body target, and monotonic ALX sample version into generic avoidance input without changing APF observable behavior.
- [ ] Keep target_mask only for APF compatibility. SCAN uses synthetic keep-out and must not remove wearer from collision semantics.
- [ ] Build and discard the local grid inside each fresh SCAN compute. The runtime MUST NOT reuse a previous path or obstacle grid after cloud sequence changes.
- [ ] On every fresh cloud, compare ALX sample version with the last version consumed by ordinary A*. A new sample runs normal A*. With no new sample, validate only the held command's immediate capsule sweep and do not rerun A* solely because a cloud arrived.
- [ ] If the held command is blocked, dispatch zero for that cloud and attempt one emergency A* from the current grid and latest fresh ALX target. Do not wait for a new ALX sample to avoid a newly observed obstacle.
- [ ] Move or gate automatic escape and target retreat so they remain APF-specific and cannot override a SCAN path or generate unplanned reverse motion.
- [ ] Map no_path to immediate zero while follow stays enabled. Map invalid_grid and planner_timeout to existing hard-stop path, disable follow, and require explicit Start. Never invoke APF as fallback.
- [ ] Keep existing final StopLatch re-read, slew limiter, cloud/strategy age gate, and bridge dispatch as sole final command authority.

Acceptance: integration tests prove APF compatibility, YAML-selected SCAN, per-cloud grid reconstruction, mode-specific guards, no-path retry, invalid-grid hard-stop, timeout hard-stop, and no APF fallback.

## Task 6: Make diagnostics and logs planner-neutral

Files:

- Modify src/robot_follow/lidar_alx_assisted_follow/runtime/csv_log.py
- Modify src/robot_follow/lidar_alx_assisted_follow/web/serialization.py and Web tests
- Modify controller and log tests

- [ ] Add avoidance mode, planner outcome, replan trigger (new ALX, blocked command, or none), accepted ALX sample version/age, current-grid occupied-cell count, usable region, path length, path minimum clearance, expanded nodes, and compute duration to status and CSV diagnostics.
- [ ] Preserve existing APF fields or provide backward-compatible neutral aliases. No log field may label SCAN command as APF output.
- [ ] Bound serialized path and grid visualization so monitoring cannot make a 20 ms strategy frame overrun.
- [ ] Do not expose or log map generation, pose topic, pose offset, map age, or map reset concepts because the lightweight mode has none.

Acceptance: APF logs retain their meaning, SCAN logs expose mode and outcome, diagnostics are finite and bounded, and operator can distinguish no-path, invalid-grid, and timeout.

## Task 7: Verification and release gate

- [ ] Run focused pure-Python config, local-grid, footprint, A*, scan strategy, controller, logging, and serialization tests in the mandated project environment.
- [ ] Run full tests/lidar_alx_assisted_follow regression suite; APF remains default and passes unchanged behavior tests.
- [ ] Add deterministic tests for expansion limits and deadline handling instead of non-deterministic performance assertions.
- [ ] Add CUDA-guarded parity tests proving GPU filtering, occupancy, and collision outputs match the explicit CPU reference; skip only when CUDA is unavailable in unit-test CI, but require them on the target deployment image.
- [ ] Profile end-to-end GPU latency, including cloud upload and synchronization, on recorded target-hardware replays. The CUDA default is accepted only when p50 and p99 remain below the 20 ms strategy budget at expected point-cloud density; otherwise reduce grid/point workload before considering a different algorithm.
- [ ] Add a deterministic asynchronous-rate test: ten fresh cloud cycles with one new ALX sample run ordinary A* exactly once, validate the held command on the other cycles, and run emergency A* only when a later cloud blocks that command.
- [ ] Before real G1 output, verify body-frame cloud axes, cloud density, physical G1 hull dimensions, point-cloud coverage during turns, and local-grid update plus A* compute time below 20 ms using recorded replay.
- [ ] Choose conservative initial speed and lookahead values because the planner has no memory of temporarily occluded obstacles.
- [ ] Do not run ROS2, Livox, FAST-LIO, serial, Unitree SDK, or real G1 during implementation unit-test verification. Hardware validation is a separate low-speed release gate.

## Out of scope

- FAST-LIO Odometry subscription, point-cloud pose synchronization, world-frame transform, rolling occupancy map, and cross-frame obstacle persistence.
- Direct reuse of upstream SCAN C++ or ROS1 controller.
- Runtime or Web switching between APF and SCAN.
- B-spline or time-parameterized trajectory generation.
- Multi-floor, stair, or non-flat terrain traversal; this release uses collision-relevant height filtering only.
- Automatic APF fallback after SCAN failure.
- Altering unrelated AO runtime or frontend work already in the worktree.

## Final acceptance checklist

1. apf mode remains compatible with existing behavior.
2. scan mode defaults to an initialised CUDA GPU backend and uses only current valid body-frame cloud and ALX target, never a world pose or former cloud; unavailable GPU causes explicit startup failure, not silent CPU/APF fallback.
3. Each new cloud produces a fresh local grid and never inherits former obstacles or paths; it may retain only a short-lived command until its immediate sweep is revalidated by that cloud.
4. Person keep-out and stand-off prevent unsafe path crossing or approach.
5. No SCAN result bypasses StopLatch, cloud and strategy age checks, output limits, or G1 dispatch safety.
6. Planner work unable to complete within 20 ms fails closed.
