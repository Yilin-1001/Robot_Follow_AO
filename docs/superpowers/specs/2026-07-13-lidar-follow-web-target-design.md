# Web-Selected LiDAR Human Follow Design

> Status: approved design, pending user review before implementation  
> Date: 2026-07-13  
> Scope: a new independent `lidar_follow` mode for Unitree G1

## 1. Goal

Add a LiDAR-only follow mode in which an operator selects one visible human
point-cloud cluster from the Web view and explicitly presses **Start follow**.
The G1 then follows that locked human while avoiding all other obstacles with
the existing AO pipeline.

The mode must be safe by default: loss of the selected target, stale radar
input, an operator stop, or a hardware/control fault produces an immediate
zero-velocity command. The robot must never continue walking from a predicted
target position.

## 2. Scope and non-goals

### In scope

- A distinct `lidar_follow` mode, independent of the current `alx_follow`
  target source.
- Enlarged point-cloud perception for human detection and Web selection.
- 3D clustering and geometric human-candidate filtering.
- Web selection, explicit start, stop/unlock, visual target status, and
  target-cluster highlighting.
- Short-horizon cluster association while the target is visible.
- Existing FAST-LIO latest-frame source, AO planner, and G1 bridge reuse.
- Target-aware safety distance plus AO avoidance of all non-target obstacles.

### Explicitly out of scope for the first version

- RGB-D fusion, person re-identification, face recognition, and learned human
  classifiers.
- Automatically selecting or reselecting a person.
- Continuing motion during target occlusion or prediction-only tracking.
- Following arbitrary objects such as carts or boxes.
- Dynamic mode switching in a running process.

`lidar_follow` is selected before launch/configuration. An `alx_follow` run
continues to use its existing serial source; a `lidar_follow` run must not
open, read, or depend on the ALX serial device.

## 3. Chosen approach

Three approaches were considered:

1. Nearest-cluster tracking is small but can silently switch people when they
   cross.
2. Geometric 3D human clusters plus Kalman prediction and strict association
   gates is the selected MVP. It needs no trained model and fits the existing
   LiDAR/AO/G1 architecture.
3. LiDAR plus RGB-D person re-identification gives stronger identity handling,
   but requires camera calibration, inference deployment, and clock
   synchronization. It is a later extension.

The MVP therefore uses approach 2. It protects against an unsafe identity
switch by stopping whenever a current cluster does not pass association gates;
it does not claim to maintain identity through occlusion.

## 4. Architecture

```text
FAST-LIO /cloud_registered_body (latest body-frame cloud)
  -> point-cloud preparation
      -> tracking ROI ----------------------> HumanClusterDetector
                                                -> human candidates
Web control mailbox ------------------------> TargetLockManager
                                                -> selected/locked cluster
human candidates + locked state -----------> HumanTrackAssociator
                                                -> current target cluster
current target cluster --------------------> TargetProximityGuard
current target cluster --------------------> TargetMasker
      -> avoidance ROI + non-target points -> existing AO pipeline
target position + guards + AO result ------> LidarFollowRuntime
                                                -> existing G1MotionBridge
```

The live control loop owns all state transitions and all G1 calls. The Web
server is a producer of control events and a consumer of status frames only;
it never sends a motion command directly.

### 4.1 Separate tracking and avoidance point clouds

The same body-frame input frame is prepared once, then split into two logical
uses:

- **Tracking ROI:** a wide region used to detect and maintain the selected
  human. Initial configurable bounds: `x=0.3..5.0 m`, `y=-2.5..2.5 m`,
  `z=0.1..2.0 m`.
- **Avoidance ROI:** the existing conservative near-field AO region. Its
  bounds must not expand merely because the tracking ROI is larger.

The selected target cluster is removed only from AO's ordinary-obstacle input.
All other point-cloud clusters, including other humans, remain obstacles. A
separate target-distance guard always evaluates the selected human, so masking
does not permit the robot to walk into that person.

### 4.2 Runtime states

| State | Meaning | Motion output |
|---|---|---|
| `IDLE` | No target selected; candidates may be displayed. | stop |
| `CANDIDATE` | A Web click resolved to one current human cluster. | stop |
| `FOLLOWING` | Operator pressed Start and all freshness/safety gates passed. | controlled follow or stop |
| `LOST` | The locked cluster cannot be associated in the current frame. | immediate `stop(target_lost)` then `IDLE` |
| `FAULT` | Radar, loop, AO, or G1 bridge failure. | stop until explicitly recovered/reselected |

There is deliberately no prediction-only moving state. A Kalman filter may
predict the expected cluster center solely to define the next-frame association
gate; it must never provide a motion target if the associated cluster is
absent.

## 5. Components and contracts

### `HumanClusterDetector`

Consumes the prepared tracking ROI and produces a list of immutable
`HumanCandidate` records. Each record includes a per-frame cluster ID, point
indices, centroid, 3D extents, point count, nearest distance, and a geometric
confidence/rejection reason.

The detector uses voxel filtering followed by Euclidean clustering. A candidate
must pass configurable size, height, width/depth, point-count, and range
limits. These are geometric plausibility filters, not an assertion that the
cluster is a uniquely identified person.

### `TargetLockManager`

Owns state transitions and processes ordered, bounded Web control events in
the runtime thread:

- `select_target(x, y)`: resolve the click to the nearest valid current
  `HumanCandidate` within a configurable pick radius; enter `CANDIDATE`.
- `start_follow`: allowed only from `CANDIDATE` after the runtime revalidates
  the chosen candidate and fresh radar.
- `stop_unlock`: send stop, clear candidate/track state, enter `IDLE`.

Multiple unprocessed selections coalesce to the newest selection. Stop/unlock
is idempotent and has priority over selection/start events.

### `HumanTrackAssociator`

For a locked target, predicts a center using a constant-velocity Kalman state,
then scores current candidates by Mahalanobis/center distance, plausible
velocity, and compatible dimensions. Exactly one candidate must pass the
configured gates. Zero or ambiguous matches produce `LOST`; the runtime stops
in that control tick. It does not switch to the nearest other person.

### `TargetProximityGuard`

Calculates human-to-robot clearance from the selected cluster's nearest point
and extents. Initial policy values, all configuration-controlled, are:

- desired follow distance: `1.2 m`;
- no-forward distance: `0.9 m`;
- mandatory stop distance: `0.7 m`.

It prevents positive forward velocity inside the no-forward distance and
outputs a stop inside the mandatory-stop distance. Final values must be
calibrated against the physical G1 body envelope and LiDAR extrinsics before
walking tests.

### `TargetMasker`

Uses the exact point indices of the currently associated target cluster to
exclude only those points from the non-target AO input. It must not mask an
expanded radius or neighbouring clusters. If cluster membership, timestamp, or
association validity is uncertain, it returns an error rather than masking;
the runtime stops through the target-loss/fault path.

### `LidarFollowRuntime`

Owns the 40 Hz latest-only control cycle. It reads the latest point cloud,
drains Web control events, detects candidates, updates target state, runs the
target guard and AO, then dispatches at most one current command to the
existing G1 bridge. It does not queue historical point clouds or motion
commands.

The existing ALX runtime remains separate. Shared lower-level modules such as
`FastLIOPointCloudSource`, radar preprocessing, AO planning, loop timing, CSV
logging, and `G1MotionBridge` can be reused behind mode-specific adapters.

## 6. Web design and protocol

The current point-cloud canvas becomes a live human-candidate view:

- valid human candidates use a distinct outline/colour;
- a clicked candidate is highlighted as `CANDIDATE` with distance, dimensions,
  and confidence displayed;
- the locked candidate has a persistent follow colour and target status;
- non-candidate raw points remain visible for operator context.

Controls:

| Control | Enabled when | Result |
|---|---|---|
| Start follow | current state is `CANDIDATE`, radar is fresh, candidate is still valid | sends `start_follow` |
| Stop and unlock | `CANDIDATE`, `FOLLOWING`, `LOST`, or `FAULT` | sends priority `stop_unlock` |
| Select target | radar is fresh and a click resolves to a human candidate | sends `select_target(x, y)` |

The runtime publishes target state, stop reason, selected cluster geometry,
target clearance, candidate summaries, cloud age, and active AO decision in
each Web status frame. A Web disconnect does not stop an already-running robot
by itself; the last explicit state remains until an operator stop, a safety
event, or a target loss. Browser reconnection restores the most recent status.

## 7. Control and safety policy

The effective command is created in this order:

```text
operator/mode gate
  -> sensor freshness gate
  -> target-lock gate
  -> target-proximity guard
  -> existing AO planner for non-target obstacles
  -> G1 final command validation and watchdog
```

Safety precedence is strict:

```text
physical E-stop / G1 bridge fault
  > stop/unlock or mode not following
  > missing, stale, malformed, or wrong-frame point cloud
  > target lost or ambiguous association
  > target mandatory-stop distance
  > AO emergency stop / blocked route
  > normal follow command
```

For the first physical run, use only low forward and yaw speeds, disable
lateral motion, and require an operator with an independent physical stop.
Recommended initial maximums are `vx=0.2–0.3 m/s` and `|omega|=0.3–0.5 rad/s`.
The production control configuration must retain the current age budgets:
newest-only frames, point-cloud timeout, command timeout, AO time budget, G1
dispatch watchdog, and fail-closed behaviour.

## 8. Verification and acceptance criteria

### Automated tests

- Point-cloud preprocessing and cluster extraction with synthetic fixtures.
- Human geometry acceptance/rejection at boundary values.
- Click-to-cluster resolution, event priority, and every lock-state transition.
- Association accepts the intended nearby continuation but rejects an ambiguous
  or materially different candidate.
- Target mask removes only selected-cluster indices; other humans remain AO
  obstacles.
- Target proximity values cap forward motion and force stop at the configured
  boundary.
- Runtime emits stop for target loss, stale/malformed cloud, AO failure,
  control-loop overrun, and G1 bridge fault.
- Web protocol and frontend tests prove Start is disabled before candidate
  selection and Stop/unlock is always available while armed/active.

### Recorded-data acceptance

Use FAST-LIO rosbag/replay captures containing one person, brief occlusion,
two people crossing, a non-person object, and static obstacles. The expected
result is no movement without Start, no automatic person switch, and a stop on
the first unassociated target frame.

### Physical acceptance sequence

1. Robot stationary: compare cluster overlays with people in the field of
   view; no G1 motion output.
2. Robot stationary: exercise select/start/stop and injected stale/lost data;
   verify only zero commands on every fault path.
3. Open, level, controlled space: one walking person, low speed, independent
   E-stop, and a safety observer.
4. Add static obstacles, then a second person; verify non-target obstacles
   remain in AO and target loss stops immediately.

The feature is acceptable for the first deployed mode only if it passes all
automated checks, replay scenarios, and low-speed physical safety cases without
an automatic target switch or a motion command after a target-loss condition.

## 9. Configuration and observability

Add a `lidar_follow` configuration section rather than changing ALX defaults.
It contains mode selection, tracking/avoidance ROI bounds, clustering and human
geometry thresholds, click radius, association gates, distance thresholds,
speed limits, and required freshness budgets.

Per control tick log: lock state, Web event sequence, chosen candidate ID,
association score/reason, target distance/clearance, target-mask count,
candidate count, cloud age, AO reason, safe command, and G1 dispatch timing.
These fields make a target switch, loss, or safety stop diagnosable from a
recorded run.

## 10. Deferred extension

If field tests show that geometric association causes too many stops around
similar people, add RGB-D person detection/re-identification as an optional
identity score. It must be additive: a missing camera, disagreement between
sensors, or stale visual result must fail closed rather than silently retaining
motion.
