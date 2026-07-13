# Web-Selected LiDAR Human Follow Design

> Status: revised with confirmed operating and safety parameters; pending implementation planning
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
- Web control of radar start/stop, target selection, explicit follow start,
  stop/unlock, visual target status, and target-cluster highlighting.
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
open, read, or depend on the ALX serial device. In a `lidar_follow` run,
Livox and FAST-LIO are initially off and are started only by the unique Web
controller's explicit radar-start action.

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
it does not claim to maintain identity through occlusion. Candidate filtering
is for adults only. Web selection is the final human confirmation; no learned
human semantic model is introduced in this version.

## 4. Architecture

```text
FAST-LIO /cloud_registered_body (latest `body`-frame cloud)
  -> point-cloud preparation
      -> tracking ROI ----------------------> HumanClusterDetector
                                                -> human candidates
Web control mailbox ------------------------> TargetLockManager
                                                -> selected/locked cluster
human candidates + locked state -----------> HumanTrackAssociator
                                                -> current target cluster
current target cluster --------------------> TargetProximityGuard
current target cluster --------------------> TargetAwareObstacleClassifier
      -> avoidance ROI + non-target points -> existing AO pipeline
target position + guards + AO result ------> LidarFollowRuntime
                                                -> existing G1MotionBridge
```

The live control loop owns all state transitions and all G1 calls. The Web
server is a producer of control events and a consumer of status frames only;
it never sends a motion command directly.

The mode reuses the project's configured Livox and FAST-LIO launch commands,
the existing `FastLIOPointCloudSource`, and its `/cloud_registered_body` input
whose expected frame is `body`.

### 4.1 Separate tracking and avoidance point clouds

The same body-frame input frame is prepared once, then split into two logical
uses:

- **Tracking ROI:** a cylindrical wide region used to detect and maintain the
  selected human: radial distance `0.25..5.0 m`, full 360-degree horizontal
  coverage, and physical height `0.1..2.0 m` above the local ground plane.
  The latter must be obtained through calibrated frame transforms and ground
  estimation, not assumed from raw point-cloud `z` values.
- **Avoidance ROI:** the existing conservative near-field AO region. Its
  detection range is `2.5 m`; its bounds must not expand merely because the
  tracking ROI is larger.

The selected target cluster is removed only from AO's ordinary-obstacle input.
All other point-cloud clusters, including other humans, remain obstacles. A
separate target-distance guard always evaluates the selected human, so masking
does not permit the robot to walk into that person.

### 4.2 Radar lifecycle and runtime states

Radar lifecycle is independent from target-lock state:

| Radar state | Meaning | Permitted target action |
|---|---|---|
| `RADAR_OFF` | Livox and FAST-LIO are stopped. | none |
| `RADAR_STARTING` | Web requested startup; runtime waits for a fresh valid body-frame cloud. | none |
| `RADAR_READY` | Services are running and a valid cloud is fresh. | select target / start follow |
| `RADAR_FAULT` | Startup failed, readiness timed out, or a running cloud became stale. | controller must click Start radar again |

Clicking **Start radar** starts Livox first and FAST-LIO second, then waits for
the configured `10 s` readiness timeout. A startup failure, timeout, or later
cloud-staleness fault enters `RADAR_FAULT`; there is no automatic retry. The
runtime retains the Livox/FAST-LIO processes in this fault state, and a
subsequent explicit Start-radar click performs the reset/readiness sequence.
Clicking **Stop radar** is enabled only while target state is `IDLE`; it
verifies zero velocity, clears target state, stops FAST-LIO and Livox, and
returns to `RADAR_OFF`. The controller cannot turn radar off while following:
it must first use Stop and unlock.

The G1 bridge is initialized only after Start radar has produced a fresh valid
point cloud. Because the SDK/DDS bridge is not safely reinitialized within the
same process, stopping radar later keeps the initialized bridge connected and
continuously commanded at zero velocity.

Target-lock state is:

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
must pass configurable adult size, height, width/depth, point-count, and range
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
velocity, and compatible dimensions. The center-displacement association gate
is `0.4 m` per `10 Hz` frame. Exactly one candidate must pass the configured
gates. Zero or ambiguous matches produce `LOST`; the runtime stops in that
control tick. It does not switch to the nearest other person.

### `TargetProximityGuard`

Calculates human-to-robot clearance from the selected cluster's nearest point
and extents. Initial policy values, all configuration-controlled, are:

- desired follow distance: `1.0 m`;
- mandatory stop distance: `0.8 m`.

It outputs a stop inside the mandatory-stop distance. The `0.8 m` threshold
already includes the allocated measurement-error and sudden-approach margin;
there is no additional independent target buffer. Between `0.8 m` and the
desired `1.0 m` distance, the follow controller may issue a reverse command to
restore separation, capped by the configured reverse-speed limit. Final
thresholds must be calibrated against the physical G1 body envelope and LiDAR
extrinsics before walking tests.

### `TargetAwareObstacleClassifier`

Reclassifies the exact point indices of the currently associated target cluster
as **followed human**, rather than as ordinary obstacles. Those points are
excluded only from AO's ordinary-obstacle avoidance calculation; they are
always retained by `TargetProximityGuard` for person-to-robot clearance. It
must not classify an expanded radius or neighbouring clusters as target points.
If cluster membership, timestamp, or association validity is uncertain, it
returns an error rather than reclassifying points; the runtime stops through
the target-loss/fault path.

### `LidarFollowRuntime`

Owns a latest-only runtime with two distinct rates. Each new point cloud drives
candidate detection, target association, target guard, local AO, and nonzero
follow-command issuance at `10 Hz`. A separate `40 Hz` safety watchdog checks
controller ownership, radar/bridge state, and cloud/strategy freshness and
issues only immediate zero-velocity stops on a failed gate; it does not
reissue movement commands. Point-cloud and strategy age are each limited to
`0.2 s`. It does not queue historical point clouds or motion commands.

The G1 movement-command duration is the YAML-controlled short horizon `0.2 s`.
Forward speed is dynamically limited using the conservative assumed total stop
time `0.5 s`, so the commanded speed falls as target clearance approaches the
mandatory distance. The runtime also owns Livox/FAST-LIO process lifecycle:
only a `start_radar` event from the unique controller may launch/reset the
services, and it stops both services after `stop_radar` or controller
disconnection.

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

The Web view receives the downsampled cloud and status at `10 Hz` (the existing
visualizer's bounded point payload is retained). Candidate/target overlays are
derived from the same control frame, so a click always resolves against the
latest displayed candidate generation.

Controls:

| Control | Enabled when | Result |
|---|---|---|
| Start radar | controller owns control, radar is `RADAR_OFF` or `RADAR_FAULT` | sends `start_radar` |
| Stop radar | controller owns control, radar is on, target state is `IDLE` | sends `stop_radar` |
| Start follow | controller owns control, target state is `CANDIDATE`, radar is `RADAR_READY`, candidate is still valid | sends `start_follow` |
| Stop and unlock | controller owns control; target state is `CANDIDATE`, `FOLLOWING`, `LOST`, or `FAULT` | sends priority `stop_unlock` |
| Select target | controller owns control, radar is `RADAR_READY`, and a click resolves to a human candidate | sends `select_target(x, y)` |

The runtime publishes target state, stop reason, selected cluster geometry,
target clearance, candidate summaries, cloud age, and active AO decision in
each Web status frame. The first successfully connected WebSocket client becomes
the unique controller; later clients are read-only observers. The deployment is
limited to a trusted LAN and has no login/token layer. If the controller
disconnects, the runtime immediately stops, unlocks the target, stops FAST-LIO
and Livox, releases radar resources, and relinquishes control ownership.
The next successfully connected client may become the new controller.

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
  > radar lifecycle fault or controller disconnect
  > missing, stale, malformed, or wrong-frame point cloud
  > target lost or ambiguous association
  > target mandatory-stop distance
  > AO emergency stop / blocked route
  > normal follow command
```

Configured speed limits for this mode are maximum forward `vx=1.0 m/s`,
maximum reverse `vx=-1.0 m/s`, maximum lateral `|vy|=0.5 m/s`, and maximum
angular `|omega|=1.0 rad/s`. Lateral AO avoidance is permitted. Physical tests
must begin below these configurable caps and require an operator with an
independent physical stop.

AO is strictly local: it may turn and move laterally to bypass an obstacle, but
does not build a map, plan a global detour, or use reverse motion to escape.
If the `2.5 m` local detection region has no safe velocity while the target is
still visible, the runtime retains the lock and sends zero velocity until a
safe local velocity appears. For all non-target obstacles, `0.6 m` is the hard
minimum clearance measured from the G1 physical body envelope, and `1.2 m` is
the slowdown/planning distance. The implementation inflates the G1 body model;
it does not interpret either value as a LiDAR-origin distance.

The operator must use the physical remote to put G1 into a walk-capable state
before starting follow. The runtime never auto-stands or changes G1 locomotion
mode. It also deliberately does not subscribe to or act on remote-control
input during follow; until another higher-level G1 arbitration takes effect,
the runtime continues issuing its configured follow commands.

The production control configuration must retain newest-only frames, the
`0.2 s` cloud/strategy age budgets, command timeout, AO time budget, G1
dispatch watchdog, and fail-closed behaviour.

## 8. Verification and acceptance criteria

### Automated tests

- Point-cloud preprocessing and cluster extraction with synthetic fixtures.
- Human geometry acceptance/rejection at boundary values.
- Click-to-cluster resolution, event priority, and every lock-state transition.
- Association accepts the intended nearby continuation but rejects an ambiguous
  or materially different candidate, including a `0.4 m` association-gate
  boundary.
- Target-aware obstacle classification reclassifies only selected-cluster
  indices; other humans remain AO obstacles and the selected human remains in
  the proximity guard.
- Target proximity values cap forward motion and force stop at the configured
  boundary.
- Runtime emits stop for target loss, ambiguity, stale/malformed cloud, AO
  failure, control-loop overrun, and G1 bridge fault. Target loss, ambiguity,
  and point-cloud staleness all unlock the target immediately.
- Radar lifecycle tests prove manual start order, no automatic retry, timeout
  fault, runtime cloud-staleness fault, explicit reset/readiness after fault,
  deferred G1-bridge initialization, and stop only after follow has been
  unlocked.
- Scheduling tests prove that a new cloud issues at most one strategy command
  at `10 Hz`, while the `40 Hz` watchdog cannot issue a move command and stops
  on cloud/strategy age over `0.2 s`.
- Local-AO tests prove a `0.6 m` physical-envelope hard stop, `1.2 m` slowdown,
  no reverse escape, retained lock while a visible target is temporarily
  blocked, and automatic resumption only after a safe local velocity exists.
- Web protocol and frontend tests prove unique-controller ownership, observer
  read-only rejection, controller-disconnect stop/unlock/radar shutdown, Start
  disabled before candidate selection, and Stop/unlock availability while
  armed/active.

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
   remain in AO, an ambiguous association stops/unlocks, and target loss stops
   immediately.

The feature is acceptable for the first deployed mode only if it passes all
automated checks, replay scenarios, and low-speed physical safety cases without
an automatic target switch or a motion command after a target-loss condition.

## 9. Configuration and observability

Add a `lidar_follow` configuration section rather than changing ALX defaults.
It contains mode selection, Livox/FAST-LIO launch and readiness settings,
tracking/avoidance ROI bounds, clustering and adult-human geometry thresholds,
click radius, association gates (including `0.4 m`), target and ordinary-
obstacle distance thresholds, forward/reverse/lateral/yaw speed limits,
`0.2 s` G1 command duration, `0.5 s` assumed total stop time, `10 Hz`
strategy/Web rates, `40 Hz` safety-watchdog rate, Web-controller policy, and
required freshness budgets. The YAML file is loaded only at process startup;
runtime Web controls cannot alter or reload parameters, and any configuration
change requires a stopped-process restart.

Per control tick log: radar state, lock state, controller ownership, Web event
sequence, chosen candidate ID, association score/reason, target
distance/clearance, reclassified-point count, candidate count, cloud age, AO
reason, safe command, and G1 dispatch timing. These fields make a target
switch, loss, radar lifecycle issue, or safety stop diagnosable from a
recorded run.

## 10. Deferred extension

If field tests show that geometric association causes too many stops around
similar people, add RGB-D person detection/re-identification as an optional
identity score. It must be additive: a missing camera, disagreement between
sensors, or stale visual result must fail closed rather than silently retaining
motion.
