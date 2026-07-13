# Web-Selected LiDAR + ALX Assisted Human Follow Design

> Status: revised design; pending implementation planning
> Date: 2026-07-13
> Scope: an independent `lidar_alx_assisted_follow` mode for Unitree G1

## 1. Goal

Add a follow mode in which an operator selects a visible human point-cloud
cluster in the Web view and explicitly presses **Start follow**.  The selected
person must wear the configured, unique ALX tag.  LiDAR is the authoritative
source for the person's current geometry, clearance, and obstacle layout; the
ALX distance/bearing observation raises or lowers confidence that the selected
LiDAR cluster is that tagged person.

The G1 follows only a currently confirmed target cluster, using a
target-aware artificial potential field (APF) for local obstacle avoidance.
This mode is fail-closed: ALX failure or contradiction, target loss, stale
point cloud, Web stop, controller disconnect, AO failure, or G1 fault issues
zero velocity immediately.  No predicted LiDAR or ALX position may keep the
robot moving.

## 2. Mode boundary

`lidar_alx_assisted_follow` is selected before process launch and is distinct
from both existing modes:

- `alx_follow` keeps its existing serial-only target source and control path.
- `lidar_follow`, if deployed, remains a pure LiDAR mode and must not open the
  ALX serial device.
- `lidar_alx_assisted_follow` starts ALX, Livox, and FAST-LIO only for this
  process.  It reads exactly the configured target `tag_id`; frames for any
  other tag are invalid for this mode.

There is no runtime mode switch and no automatic target or tag selection.  The
operator's point-cloud click establishes the initial candidate; the configured
ALX tag establishes the identity that must support it.

## 3. Chosen approach

The mode combines three independent evidence sources:

1. **LiDAR human candidates.** Voxel filtering, Euclidean clustering, and
   configurable adult geometry filters produce current candidate clusters.
2. **LiDAR temporal continuity.** A constant-velocity Kalman state predicts
   the selected cluster only to score next-frame association; it is never a
   moving target by itself.
3. **ALX probabilistic identity evidence.** A valid ALX frame supplies
   `tag_id`, `tag_status`, range, bearing, elevation, and timestamp.  Its range
   and bearing are compared with the measurements predicted from every LiDAR
   candidate after calibrated transforms and time alignment.

This is neither nearest-cluster tracking nor a weighted heuristic.  It is a
log-likelihood fusion with non-negotiable safety gates.  A high LiDAR score can
never compensate for an invalid tag, stale serial data, or a gross ALX/LiDAR
contradiction.

The local motion layer uses the APF principle in
[`6-robot/jie_deamon`](https://github.com/6-robot/jie_deamon): target-directed
motion is combined with point-obstacle repulsion, bounded by emergency and
slowdown distances.  This design strengthens that approach with body-envelope
clearance, exact target masking, deterministic fail-closed gates, no automatic
reverse escape, and the G1 watchdog.

## 4. Architecture

```text
FAST-LIO /cloud_registered_body (latest body-frame cloud)
  -> tracking ROI -> HumanClusterDetector -> human candidates
                                             |\
ALX serial -> ALXTagObservation ------------| -> ProbabilisticTargetAssociator
Web click -> TargetLockManager -------------|       -> confirmed target cluster
                                                     |                 |
                                     TargetProximityGuard       TargetAwareAPF
                                                     |                 |
                                                     +----> LidarAlxFollowRuntime
                                                                  -> G1MotionBridge
```

The live runtime alone owns target state and G1 calls.  The Web server only
produces bounded control events and consumes status frames; it never sends a
motion command.  The mode reuses the existing ALX reader, FAST-LIO latest-frame
source, point-cloud preprocessing, logging, and G1 bridge through
mode-specific adapters.  It does not reuse the old `alx_follow` path that
turns a tag distance/bearing directly into a motion command.

### 4.1 Point-cloud roles

The body-frame cloud is prepared once and split logically:

- **Tracking ROI:** full-azimuth cylindrical region, radial range `0.25..5.0
  m`, physical height `0.1..2.0 m` above the estimated local ground plane.
- **Avoidance ROI:** conservative local APF region, initially `2.5 m`; it is
  not expanded because tracking has a larger range.

The exact point indices of a currently confirmed target cluster are excluded
only from APF obstacle repulsion.  They are always retained by the target
proximity guard.  Every other cluster, including every other human, remains an
obstacle.

### 4.2 Runtime states

Radar and serial readiness are independent of lock state.  Livox starts before
FAST-LIO after explicit **Start radar**.  ALX must also have a fresh valid
position frame for the configured tag before the state can become
`SENSORS_READY`.  Startup failure, stale data, or read error enters
`SENSOR_FAULT`; there is no automatic retry.

**Start radar** starts or resets the ALX reader, then Livox, then FAST-LIO,
and waits for fresh valid inputs from both sources within the configured
readiness timeout.  **Stop radar** is allowed only while the target lock is
`IDLE`; it verifies zero velocity, clears state, stops FAST-LIO and Livox, and
closes the ALX reader.  The G1 bridge is initialized only after fresh inputs
have made the sensors ready.  A fault requires a later explicit Start-radar
action; it is never recovered by an automatic service retry.

| Lock state | Meaning | Motion output |
|---|---|---|
| `IDLE` | No selected target. | stop |
| `CANDIDATE` | Web click resolved to a current human cluster. | stop |
| `FOLLOWING` | LiDAR and ALX association is uniquely confirmed. | APF command or stop |
| `UNCERTAIN` | Candidates exist, but no unique score satisfies confirmation. | stop only |
| `LOST` | The selected cluster is absent or no longer associable. | immediate stop, then `IDLE` |
| `FAULT` | Sensor, loop, APF, or G1 failure. | stop until explicit recovery/reselection |

`UNCERTAIN` is a bounded, non-moving wait for an ordinary score-margin failure.
It may last only for a YAML-configured short time.  It does not apply to a hard
gate failure.  A missing selected cluster, an empty candidate set, stale or
invalid ALX, tag change, or hard ALX/LiDAR contradiction immediately clears
the lock; no prediction-only state exists.

## 5. Target perception and probabilistic association

### 5.1 Human candidate detector

`HumanClusterDetector` applies voxel filtering followed by Euclidean
clustering.  Each immutable `HumanCandidate` contains per-frame ID, point
indices, centroid, 3D extents, point count, nearest body-envelope clearance,
and geometry acceptance/rejection reason.  Configurable adult height,
width/depth, point-count, and range limits are plausibility filters, not person
recognition.

### 5.2 ALX observation

`ALXTagObservation` is derived from the latest ALX position frame only when:

- its `tag_id` equals configured `target_tag_id`;
- `tag_status` is valid according to configured protocol semantics;
- its range, bearing, elevation, timestamp, and sequence are finite and fresh;
- it is within the configured LiDAR/ALX synchronization window.

The calibration provides the rigid transform between ALX and LiDAR/`body`
frames and measured uncertainty parameters.  It must be recorded and
versioned with the production configuration.  The runtime must not infer these
values from live follow data.  The first release uses range and bearing for
fusion.  It records elevation for diagnostics but does not score it until tag
wearing height and vertical calibration have been validated.

### 5.3 Fusion score

For each current candidate \(i\), the associator computes:

\[
S_i=w_L\log P_L(i)+w_A\log P_A(i)+w_G\log P_G(i)
\]

- \(P_L\) is LiDAR temporal continuity from predicted centre, plausible
  velocity, and compatible dimensions.
- \(P_G\) is human-geometry quality.
- \(P_A\) is ALX consistency.  The candidate position is transformed into
  predicted ALX range/bearing \(h(x_i)\), and compared with the measurement
  \(z_\mathrm{ALX}\):

\[
d^2_{A,i}=(z_\mathrm{ALX}-h(x_i))^T
(R_\mathrm{ALX}+R_{\mathrm{LiDAR},i})^{-1}
(z_\mathrm{ALX}-h(x_i))
\]
\[
\log P_A(i)=-\tfrac12d^2_{A,i}
\]

Range/bearing residuals must use wrapped angles.  `R_ALX` and the projected
candidate covariance must be configurable from measured data, so a noisier
source contributes less without being silently ignored.

### 5.4 Selection and hard gates

`select_target(x, y)` resolves only to the nearest valid current candidate
inside a configured pick radius.  `start_follow` is legal only when the clicked
candidate passes the geometry, ALX validity, and initial fusion gates.

For later frames, exactly one candidate must exceed the confirmation score and
lead the second-best candidate by the configured score margin.  A one-frame
slightly higher score may not replace the lock; it must also pass the temporal
association gate.  An ordinary score or margin miss enters `UNCERTAIN` and
commands zero velocity.  Failing to recover before its deadline clears the
lock.

The following are hard gates and, in the same control tick, command zero,
clear track/candidate state, and return to `IDLE` or `FAULT` as appropriate:

- serial error, ALX timeout, invalid `tag_status`, unexpected `tag_id`, or
  malformed ALX frame;
- stale, malformed, wrong-frame, or unavailable point cloud;
- selected cluster absent, zero candidates, or ambiguous association beyond the
  `UNCERTAIN` limit;
- ALX/LiDAR residual beyond the independently configured hard-contradiction
  threshold;
- operator stop/unlock, controller disconnect, APF failure, loop overrun, or
  G1 bridge fault.

## 6. Target-aware APF follow control

The APF consumes only a confirmed current target cluster and the non-target
avoidance points.  It creates a local velocity reference; it builds no map,
plans no global detour, and never converts a predicted target position into
motion.

Let \(\mathbf p_t\) be the target cluster's body-frame horizontal position,
\(c\) its nearest body-envelope clearance, \(c_d=1.0\,m\) the desired target
clearance, and \(c_s=0.8\,m\) the mandatory target stop clearance.  The target
attraction is the gradient toward the desired standoff ring:

\[
\mathbf F_\mathrm{att}=k_a(c-c_d)
\frac{\mathbf p_t}{\|\mathbf p_t\|}
\]

For every non-target obstacle point or compact obstacle representative \(j\),
let \(q_j\) be its clearance from the inflated G1 body envelope.  With APF
influence distance \(q_I=1.2\,m\), its bounded repulsion is:

\[
\mathbf F_\mathrm{rep}=
\sum_{0.6<q_j<q_I}
k_r\left(\frac1{q_j}-\frac1{q_I}\right)\frac1{q_j^2}
\left(-\frac{\mathbf p_j}{\|\mathbf p_j\|}\right)
\]

The summed repulsion is magnitude-capped before combining it with attraction.
The translational reference is the clipped result of
\(k_v(\mathbf F_\mathrm{att}+\mathbf F_\mathrm{rep})\).  Its yaw reference
is a saturated P controller on the target bearing.  A large bearing error
requires turn-in-place before positive forward travel, preventing an
uncontrolled diagonal approach.  Baseline `vy` is zero; APF may add bounded
lateral motion.

`TargetProximityGuard` has precedence over APF:

- `c <= 0.8 m`: zero command.
- `0.8 m < c < 1.0 m`: it may command only configured, bounded reverse motion
  to restore separation, and only when the rear/rotation safety checks pass.
- `c >= 1.0 m`: forward speed is capped by target error, configured maximum,
  and \((c-c_s)/T_\mathrm{stop}\), with `T_stop = 0.5 s` initially.

APF repulsion may reduce forward motion to zero and create lateral motion, but
may not itself command reverse escape.  This mode uses no fixed minimum
nonzero speed: a small or zero net field is a safe zero-velocity outcome.

## 7. Obstacle safety and command precedence

For non-target obstacles, clearance at or below `0.6 m` is an immediate APF
emergency stop.  Between `0.6 m` and `1.2 m`, APF repulsion and a clearance
slowdown factor limit the reference command.  The body model is inflated; no
threshold is interpreted as a distance merely from the LiDAR origin.

APF local minima, cancellation of attraction and repulsion, an unavailable
safe lateral direction, or an APF time-budget violation produce zero velocity.
The target may remain locked only if every target, point-cloud, and ALX hard
gate is still valid; movement resumes only after a subsequent fresh frame
creates a safe nonzero command.

Effective command precedence is:

```text
physical E-stop / G1 bridge fault
  > stop/unlock or mode not FOLLOWING
  > ALX validity, tag identity, or radar lifecycle fault
  > stale/malformed/wrong-frame cloud
  > target lost, hard ALX/LiDAR contradiction, or expired UNCERTAIN state
  > target mandatory-stop clearance
  > non-target obstacle emergency stop / APF failure
  > APF follow command
  > G1 final validation and watchdog
```

Each fresh cloud may issue at most one nonzero strategy command at `10 Hz`.
A separate `40 Hz` watchdog checks ownership, ALX/cloud/strategy freshness,
runtime health, and G1 dispatch health; it can issue only a zero command.
Cloud, ALX, and strategy age budgets are initially `0.2 s`.  G1 command
duration is YAML-controlled and initially `0.2 s`.

## 8. Web control and observability

The Web view displays current candidate outlines, a clicked `CANDIDATE`, the
confirmed target cluster, non-candidate raw points, target clearance, and the
ALX association state.  It must show the configured tag ID, observed tag ID,
ALX age/status, fusion score, second-best score margin, and stop reason; this
makes identity decisions auditable.

Controls are `Start radar`, `Stop radar`, `Select target`, `Start follow`, and
`Stop and unlock`.  `Start follow` requires `CANDIDATE`, `SENSORS_READY`, a
fresh valid configured tag observation, and a passing initial fusion gate.
`Stop and unlock` is idempotent and has event priority.  The first WebSocket
client is the unique controller; observers are read-only.  Controller
disconnect immediately stops, unlocks, shuts down Livox/FAST-LIO, closes the
ALX reader, and releases control ownership.

Per tick, log mode, radar/serial state, controller ownership, Web event
sequence, configured/observed tag IDs, ALX age and residual, calibration
version, candidate count, candidate scores, selected score/margin,
`UNCERTAIN` age, target clearance, reclassified point count, APF attraction and
repulsion vectors, closest non-target clearance, APF reason, final command,
and G1 dispatch latency.

## 9. Configuration

Add a dedicated `lidar_alx_assisted_follow` configuration section.  It must
not alter ALX-only or pure-LiDAR defaults.  It contains:

- mode selector; ALX serial port/protocol status semantics; required
  `target_tag_id`; ALX freshness and LiDAR/ALX sync limits;
- Livox/FAST-LIO lifecycle and readiness parameters;
- calibration transform/version and ALX/LiDAR covariance parameters;
- tracking/avoidance ROIs; clustering and adult geometry limits; click radius;
- LiDAR temporal gates; fusion weights; confirmation score; score margin;
  ALX/LiDAR hard-contradiction gate; `UNCERTAIN` duration;
- target `1.0 m` desired and `0.8 m` mandatory-stop clearances; APF gains,
  influence range `1.2 m`, repulsion cap, velocity and yaw caps, APF budget;
- non-target obstacle hard clearance `0.6 m`, slowdown range `1.2 m`, command
  horizon, stop-time assumption, `10 Hz` strategy, `40 Hz` watchdog, and Web
  controller policy.

YAML is read only at process start.  All numerical values require offline
replay tuning and low-speed physical calibration before walking tests.

## 10. Verification and acceptance criteria

Automated tests must cover:

- candidate geometry boundaries, Web picking, control-event priority, and all
  lock/radar/serial state transitions;
- ALX frame parsing and configured tag-ID enforcement; stale, invalid-status,
  malformed, and tag-switch immediate-stop paths;
- transform and angle-wrap correctness; covariance-aware likelihood; score
  margin; hard contradiction; bounded `UNCERTAIN`; no prediction-only motion;
- two similar people crossing, where the tagged person remains unique or the
  system stops, and no automatic person switch occurs;
- exact target-index removal from APF while non-target people remain repulsive
  obstacles and target clearance remains protected;
- attraction toward the desired standoff; repulsion direction/cap; no APF
  reverse escape; target-only permitted reverse; 1.2 m slowdown and 0.6 m
  physical-envelope stop; APF local-minimum zero output;
- runtime stops for every sensor, association, APF, loop, controller, and G1
  fault; the 40 Hz watchdog never emits movement;
- Web unique-controller policy, observer rejection, and disconnect stop,
  unlock, and sensor shutdown.

Recorded ROS bags must include a single tagged person, two people crossing,
tagged-person occlusion, tag loss/change, ALX noise/dropout, non-person
clusters, static obstacles, a narrow obstruction causing APF cancellation, and
obstacles on both sides.  Expected outcomes are no motion without explicit
start, no target switch, no movement on a hard gate, no collision-boundary
violation, and no reverse escape.

Physical acceptance proceeds from stationary overlay/stop injection, to open
level low-speed one-person tests, then static obstacles and a second person.
Every walking test requires an independent physical stop and safety observer.
The mode is acceptable only after automated, replay, and low-speed physical
tests show no automatic identity switch and no motion command after a hard
target or sensor-loss condition.
