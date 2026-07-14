# Web-Selected LiDAR + ALX Assisted Human Follow Design

> Status: revised design; ready for implementation planning
> Date: 2026-07-13
> Scope: an independent `lidar_alx_assisted_follow` mode for Unitree G1

## 1. Goal and boundary

This mode lets an operator click a visible human point-cloud location in the
Web view, then press **Start follow**. Livox is the 360-degree primary
perception source: it supplies the current target point set, its temporal
support, target position, and all obstacle points. The chest-mounted ALX
receiver supplies a front-facing 120-degree (`±60°`) wristband observation
that raises or lowers confidence that the Web-selected target point set is the
intended person.

`lidar_alx_assisted_follow` is independent of the existing modes:

- `alx_follow` remains the existing serial-only direct-follow path.
- `lidar_follow`, if deployed, remains pure LiDAR and must not open ALX.
- This mode is implemented by a new, independent orchestration runtime. It
  owns ALX, the reusable Livox/FAST-LIO configuration, locked-target confidence,
  APF, and G1 command orchestration; it must not extend the existing AO radar
  runtime or route ALX range/bearing directly to a motion command.

There is one wristband on site.  Therefore there is no static
`target_tag_id` in YAML.  After a wristband-ON event, the first fresh valid
ALX position frame dynamically binds the session `tag_id`.  The binding is
diagnostic identity metadata only and is cleared on OFF, serial failure, or
protocol failure. The Web click, not tag-ID selection, establishes the target
person.

### 1.1 Confirmed deployment contracts

The following facts have been verified on the deployed robot and are design
inputs rather than open implementation questions:

- G1 DDS communication uses network interface `eth0`.
- The existing Livox and FAST-LIO launch commands are valid for the deployed
  environment.  FAST-LIO publishes the current cloud on
  `/cloud_registered_body` with `frame_id == "body"`; the runtime must wait
  for a fresh valid frame from this interface before declaring sensors ready.
  The commands and paths remain YAML-configured so deployment changes do not
  require code changes.
- ALX-G100 ON/OFF markers, position and heartbeat frame formats, and
  `tag_status` semantics have been verified.  The initial-release policy is
  nevertheless to log and display `tag_status`, not use it as an independent
  hard start/stop gate; identity is established by the Web-selected LiDAR
  target point set and fusion evidence.

### 1.2 Coordinate convention

`body` is the sole robot body coordinate frame for this mode.  Its origin and
axis directions are already defined by the deployed FAST-LIO setup, and it is
the frame of `/cloud_registered_body`.  Do not introduce `base_link`, a
`body`-to-`base_link` transform, or an identity-transform configuration.

All LiDAR target-support coordinates, target and obstacle body-envelope
clearances, APF vectors, ALX overlay points, and the static ALX calibration use `body`.
The ALX calibration is therefore explicitly ALX-to-`body`.  The G1 command
convention must be verified at low speed to align positive `vx`/`vy` with the
defined `body` forward/lateral axes; `yaw_sign` remains the configuration for
the angular-direction calibration.

## 2. Lifecycle and ownership

The wristband controls the process lifecycle; Web controls only target
selection and following.

```text
WRISTBAND_OFF guardian --(ON + first valid position)--> STARTING_SENSORS
STARTING_SENSORS --(Livox + FAST-LIO fresh)-----------> SENSORS_READY
SENSORS_READY --(select + Start follow confirmation)-> FOLLOWING
FOLLOWING --(confidence below stop)-------------------> CONFIDENCE_STOP
CONFIDENCE_STOP --(confidence recovers)---------------> FOLLOWING
FOLLOWING / CONFIDENCE_STOP --(Web Stop)--------------> SENSORS_READY
any active state --(explicit OFF / serial disconnect / persistent protocol error)-->
  WRISTBAND_OFF guardian
```

- `WRISTBAND_OFF` is an in-process, read-only serial guardian.  It commands
  zero, holds no target lock, keeps reading ALX, and waits for the next ON; it
  does not exit just because the wristband is off.
- On ON, bind the first valid fresh position-frame tag ID, start the existing
  Livox launch command and FAST-LIO launch command, wait for fresh cloud
  input, then initialize the G1 bridge.  The bridge commands zero until Web
  explicitly starts follow.
- On explicit OFF, serial disconnect, or persistent malformed-protocol error,
  command zero, unlock, stop FAST-LIO and Livox, clear the session binding,
  and return to `WRISTBAND_OFF` in the same process.
- A healthy ON session with no new *position* frame is not OFF.  It may happen
  when the wearer leaves ALX's 120-degree field.  Keep LiDAR services running
  and let only the ALX evidence decay.
- **Stop follow** always commands zero and unlocks.  It does not stop Livox or
  FAST-LIO while the wristband remains ON.  The next run needs a new click and
  a fresh Start-follow confirmation.
- Disconnect of the Web control-lease holder commands zero and unlocks, but
  does not override the wristband-owned sensor lifecycle. Disconnect of a
  read-only observer has no control effect.

## 3. Architecture

```text
ALX serial -> WristbandLifecycleGuardian -> session tag / ALX observation
                                             |
Livox + FAST-LIO (/cloud_registered_body) -> 360° cloud -> target gate + obstacles
                                             |                         |
                                     ALX-to-body transform ------------> Web ALX overlay
Web control-lease select/start/stop ------> TargetLockManager
                                             |
                         LockedTargetConfidenceEvaluator (LiDAR + ALX evidence)
                                             |
                         confidence state -> TargetAwareAPF
                                             -> CommandSlewRateLimiter
                                             -> CommandOutputArbiter -> G1MotionBridge

40 Hz zero-only watchdog ----------------------------------> CommandOutputArbiter
```

The new orchestration runtime alone owns target state and coordinates the
components. `WristbandLifecycleGuardian`, `TargetLockManager`,
`LockedTargetConfidenceEvaluator`, `TargetKalmanFilter2D`, `TargetAwareAPF`,
and `CommandSlewRateLimiter` communicate through immutable input/output
snapshots and are independently testable. Only `CommandOutputArbiter` calls
`G1MotionBridge`: strategy and watchdog components submit command intents to
it instead. The arbiter latches every mandatory stop so a late nonzero
point-cloud result cannot override zero until a later complete safety check
permits movement. The Web server only emits bounded control events and status
frames; it never emits a motion command. The initial release uses a current body-frame cloud. A two-
dimensional constant-velocity Kalman state, \([x,y,v_x,v_y]\), receives the
confirmed gated-target horizontal centre as its \([x,y]\) measurement. Its
prediction defines the temporal gate; its *current-frame corrected* position
is the smoothed reference for APF attraction and yaw.  A prediction-only
target never creates a nonzero motion command.

The implementation reuses the existing AO Web foundation: the aiohttp app and
static asset serving, WebSocket connection/frame broadcast path, latest-frame
endpoint, point-cloud viewer/frontend build pipeline, and bounded JSON message
parsing.  A mode-specific Web adapter supplies this mode's payload and routes
validated events to `TargetLockManager`; it adds the control lease and the
Select/Start/Stop semantics defined below.  Reuse must not carry over the AO
runtime's observer-only policy, its fixed demonstration command source, or its
avoidance-planner visualization result as a motion or target-selection input.

### 3.1 Cloud roles

- **Tracking:** full-azimuth LiDAR ROI, initially radial `0.25..5.0 m` and
  physical height `0.1..2.0 m` above estimated local ground.
- **Avoidance:** a conservative local APF ROI, initially `2.5 m`.
- Exact indices of the confirmed gated target point set are excluded from APF
  repulsion only. They remain input to the target proximity guard. Every other
  point, including another person, is an obstacle.

## 4. Target state and confidence

| Lock state | Meaning | Command |
|---|---|---|
| `IDLE` | No clicked target location. | zero |
| `CANDIDATE` | Click resolved to a current plausible local target point set. | zero |
| `FOLLOWING` | Current target confidence is at/above `stop`. | APF or zero |
| `CONFIDENCE_STOP` | Below `stop`, but lock retained for recovery. | zero |
| `LOST` | Below `unlock` or in `CONFIDENCE_STOP` too long. | zero, then `IDLE` |
| `FAULT` | Cloud, APF, loop, or G1 health failure. | zero |

`start_follow` is legal only when the clicked local target point set is current,
passes point-support, single-target-shape, and human-geometry filters, a fresh
ALX position is present, the configured
ALX-to-`body` calibration is validated, and normalized confidence exceeds
`start`. The required hysteresis is:

```text
start threshold > stop threshold > unlock threshold
```

When confidence falls below `stop`, the robot stops but keeps the lock and
continues observing.  It unlocks only when confidence drops below `unlock` or
the configured confidence-stop timeout expires.  This prevents ordinary
occlusion, arm motion, and temporary ALX field-of-view loss from forcing an
immediate re-selection.

Hard safety faults bypass that hysteresis: stale/malformed/wrong-frame cloud,
target-gate extraction or total strategy time-budget overrun, APF or loop
failure, G1 bridge fault, explicit wristband OFF, serial disconnect, and
persistent ALX protocol failure command zero immediately.

## 5. Perception and locked-target confidence

### 5.1 Gated LiDAR target extraction

The initial release deliberately performs **no full-cloud Euclidean
clustering, KD-tree candidate enumeration, or automatic human discovery**.
Instead, `GatedTargetPointExtractor` produces one immutable local target
observation for the existing Web lock. This bounds the hot path to shared
point-cloud preprocessing plus vectorized point masks and compact local
statistics.

The cloud is preprocessed once in `body`: finite-point filtering, full tracking
ROI, ground/self removal, and tracking voxel downsampling. It then forks into
the target and avoidance roles; raw or preprocessed full clouds must not be
processed twice for those roles. Ground removal must retain an estimate of the
local ground height for each target-gate location, so human height is evaluated
relative to local ground rather than as an absolute `body`-frame `z` value.

On Web selection, the large purple `web_pick_radius_m` circle is the initial
target gate. The extractor takes current points inside that gate as a local
support set and accepts it only when point count, physical height, horizontal
footprint, and robust centre are plausible. It additionally rejects support
that is too dispersed, too wide, or visibly multi-peaked in a configurable
fixed-resolution horizontal occupancy check. These are conservative rejection
tests, not a mechanism for choosing one person from several; a rejected click
leaves the lock unchanged.

Once locked, each accepted cloud uses a configurable dynamic elliptical target
gate centred on the Kalman prediction. Its axes allow measured centroid jitter
and a bounded per-frame human displacement, but are not expanded to search for
a new person. The extractor uses only current points in this gate, calculates a
robust horizontal centre (median or configured trimmed estimate), point count,
horizontal footprint, 3D extents, current non-smoothed point indices, nearest
body-envelope clearance, and acceptance/rejection reasons. A current local
support set is required: a prediction alone never supplies target points.

For geometric quality, the extractor also derives a relative-ground torso-height
signature from the current support points that lie in the configured torso
height band. It must use robust height quantiles, not the arithmetic mean of
all gate points: for example

\[
H=(q_{25}(h),q_{50}(h),q_{75}(h),q_{90}(h)-q_{10}(h)),
\qquad h=z-z_{\mathrm{ground}}.
\]

The lock establishes a reference `H_ref` only from an accepted, stable target
observation. Later frames compare `H` with `H_ref` using configurable tolerant
residuals. `H_ref` may be updated only from repeatedly stable accepted
observations and never from a rejected or height-inconsistent frame. This
feature is intended to remain stable through a person turning in place while
rejecting gate contents that do not resemble the locked person's torso; it is
supporting evidence only and is not a person identity claim.

The immutable result is `GatedTargetObservation`. It replaces any global
candidate detector and contains no frame-local person ID. The
target gate, point-support limits, robust-centre estimator, dispersion/
multi-peak rejection, maximum innovation, and maximum accepted frame interval
are YAML-configured and replay-tuned.

### 5.2 ALX coordinate and observation model

The ALX receiver is installed near the chest centre, approximately `1.10 m`
above ground; Livox is near head centre at approximately `1.25 m`.  The
initial configuration records these heights but uses a two-dimensional fusion
model: ALX range and azimuth contribute to confidence; elevation is logged for diagnostics
only.  The exact fixed ALX-to-`body` fore/aft offset and yaw must be
measured on the robot and stored as configurable `x_m`, `y_m`, and `yaw_deg`
before walking tests.  No live self-calibration is allowed.

ALX observes the wrist, while a gated LiDAR target observation represents a
human body. The association therefore compares the ALX position with a configurable
body-to-wrist probability region, not a hard comparison to the target-support
centre. Its anisotropic covariance/bias must tolerate normal arm reach and
swing.  ALX `tag_status` has verified vendor semantics and is logged and shown;
by explicit initial-release policy, it is not an independent hard gate.

The backend also uses this same fixed transform to produce a Web-only ALX
overlay point in the current body-frame radar view.  For a fresh ALX position
frame with range \(r\), raw azimuth \(a\), configurable signed azimuth
convention \(s\in\{-1,+1\}\), and configurable zero offset \(a_0\):

\[
\alpha=s\operatorname{deg2rad}(a+a_0),\qquad
\mathbf p_\mathrm{ALX,body}=
\begin{bmatrix}x_m\\y_m\end{bmatrix}+
R(\mathrm{yaw\_deg})
\begin{bmatrix}r\cos\alpha\\r\sin\alpha\end{bmatrix}
\]

This conversion runs in the backend; the Web client receives only the resulting
body-frame point and ALX diagnostics.  The initial release provides no Web
sliders, no automatic fit, no multi-pose sample collection, and no Web-side
configuration persistence.  Operators use the overlay for a quick visual
check, then adjust the static YAML calibration offline for later runs if
needed. The ALX point measures the wrist, so it need only be consistent with
the gated target observation's configured body-to-wrist region, not its centre.

### 5.2.1 ALX-to-cloud synchronization

ALX position frames arrive at approximately `40 Hz`; this does not impose a
control or cloud rate.  Each accepted cloud frame triggers one synchronization
lookup at its monotonic `cloud.received_at_s`.  `ALXLiveSource` keeps a bounded
time-ordered buffer of valid position snapshots and selects the newest snapshot
whose monotonic `alx.received_at <= cloud.received_at_s`.  The selected sample
is used for that cloud's Web overlay and locked-target confidence evaluation only when

\[
0 \le cloud.received\_at\_s-alx.received\_at \le
max\_alx\_cloud\_skew\_s.
\]

The initial release performs no averaging, interpolation, or future-sample
selection.  If no causal ALX sample meets the skew bound, the cloud result is
explicitly `unsynchronized`: it has no fresh yellow-green ALX overlay point
and contributes no fresh ALX confidence evidence. All comparisons use host
`time.monotonic()` receive times; ROS point-cloud header time is retained for
diagnostics but is not compared with ALX receive time.

### 5.3 Simplified locked-target confidence

There is exactly one `GatedTargetObservation` for the existing lock. The
runtime performs no candidate comparison, ranking, second-best margin, or
relative confidence calculation. It calculates only this target's absolute
confidence in the closed interval \([0,1]\):

\[
C=\operatorname{clip}(w_Tc_T+w_Gc_G+w_Ac_A,0,1),
\qquad w_T+w_G+w_A=1
\]

- \(c_T\) is temporal continuity: current gated support is present, Kalman
  innovation and per-frame displacement are bounded, and the gate has not
  expanded to search for a new person.
- \(c_G\) is geometric point-set quality and torso-profile consistency:
  support count, single-target shape, relative-ground torso-height-band and
  robust torso-height-signature consistency with `H_ref`, footprint
  plausibility, and dispersion/multi-peak checks. Height evidence belongs only
  here; it is not duplicated in \(c_T\), and ALX wrist height must not replace
  the LiDAR torso-height signature.
- \(c_A\) is ALX consistency. With a fresh ALX position frame, it is the
  configurable normalized mapping of the range/bearing residual after the
  fixed transform and body-to-wrist model. Wrapped angular residuals are
  mandatory. With no fresh frame, do not calculate a residual or reuse its old
  wrist coordinate; set \(c_A\) to the configured no-observation baseline.

For a fresh ALX sample \(z\), the residual uses configurable covariance:

\[
d^2_A=(z-h)^T(R_\mathrm{ALX}+R_\mathrm{target}+R_\mathrm{wrist})^{-1}(z-h)
\]

`R_target` captures current gated-target centre uncertainty and `R_wrist`
captures body-to-wrist tolerance. The mapping from \(d^2_A\) to \(c_A\), all
three component weights, and the no-observation baseline are YAML-configured
and replay-tuned. The runtime logs \(c_T\), \(c_G\), \(c_A\), and \(C\).
`start`, `stop`, and `unlock` operate directly on \(C\). ALX position absence
alone is not a hard stop or unlock: a continuously LiDAR-confirmed target may
continue following when its absolute confidence remains above `stop`.

There is no automatic target switching: the dynamic gate is anchored to the
existing lock, never searches the full cloud, and never moves to another local
density peak. Missing support, a failed single-target-shape check, a gate miss,
or a competing person entering the gate leads to `CONFIDENCE_STOP`, not a
silent change of person.

### 5.4 Kalman-smoothed follow reference

After, and only after, a current `GatedTargetObservation` has passed the
locked-target confidence evaluation for the existing lock, update
`TargetKalmanFilter2D` with its robust
horizontal centre. It uses the linear constant-velocity state
\([x,y,v_x,v_y]\), a configurable process covariance, measurement covariance,
and the measured frame interval.  The update is `predict -> correct`; its
corrected \([x,y]\) becomes `smoothed_target_reference`.

This reference is deliberately a motion-smoothing output, not a source of
identity or safety truth:

- APF target attraction and target-bearing yaw use
  `smoothed_target_reference` to suppress centroid jitter and resulting G1
  velocity jitter.
- The current, unfiltered gated-target point indices and body-envelope
  clearance remain authoritative for target masking, target proximity stop,
  reverse permission, and all obstacle decisions.
- If the current target point support is unavailable, confidence evaluation fails, the
  innovation exceeds its configurable gate, or the filter is not initialized,
  issue zero as required by the lock state; do not call `predictOnly()` to
  continue following.
- Reset the filter on Web stop/unlock, wristband OFF, hard fault, a new Web
  selection, and every transition to `IDLE`/`LOST`.

## 6. Target-aware APF motion

The controller consumes only a fresh current gated target point set in `FOLLOWING`
and non-target avoidance points.  It is a local APF controller, not a map or
global detour planner.  In the equations below, \(\mathbf p_t\) is the
Kalman-corrected `smoothed_target_reference`; the clearance \(c\) and every
safety decision still come from the current unfiltered point set.

Let \(\mathbf p_t\) be the target horizontal position, \(c\) target
body-envelope clearance, desired clearance \(c_d=1.0\,m\), and target stop
clearance \(c_s=0.8\,m\):

\[
\mathbf F_\mathrm{att}=k_a(c-c_d)\frac{\mathbf p_t}{\|\mathbf p_t\|}
\]

For APF, first reduce non-target avoidance points to fixed-resolution voxels or
compact obstacle representatives, so raw point-cloud density does not multiply
the force.  For each representative \(j\), with body-envelope clearance
\(q_j\), body-frame horizontal position \(\mathbf p_j\), influence
\(q_I=1.2\,m\), and hard stop \(q_s=0.6\,m\):

\[
\mathbf F_\mathrm{rep}=
\sum_{q_s<q_j<q_I} k_r\left(\frac1{q_j}-\frac1{q_I}\right)
\frac1{q_j^2}\left(-\frac{\mathbf p_j}{\|\mathbf p_j\|}\right)
\]

For \(q_j\ge q_I\), that representative contributes exactly zero repulsion.
The direction \(-\mathbf p_j/\|\mathbf p_j\|\) points away from the
obstacle; the magnitude rises rapidly as clearance decreases. Gated-target
points never enter this representative set.

### 6.1 Force-to-command mapping

Cap the accumulated repulsion before adding attraction:

\[
\mathbf F=\mathbf F_\mathrm{att}+
\operatorname{cap}(\mathbf F_\mathrm{rep},F_\mathrm{rep,max})
\]

For ordinary `FOLLOWING` motion, map this virtual force to a body-frame
velocity reference with independently configured forward and lateral gains:

\[
\begin{bmatrix}v_x^{raw}\\v_y^{raw}\end{bmatrix}=
\begin{bmatrix}k_{v_x}&0\\0&k_{v_y}\end{bmatrix}\mathbf F,
\qquad
\omega^{raw}=\operatorname{clip}
\left(k_\omega\operatorname{wrap}(\operatorname{atan2}(p_{t,y},p_{t,x})),
-\omega_\max,\omega_\max\right)
\]

Let \(\theta_t=\operatorname{wrap}(\operatorname{atan2}(p_{t,y},p_{t,x}))\).
If \(|\theta_t|\ge\theta_\mathrm{turn}\), command turn-in-place:
`vx = 0`, `vy = 0`, and `omega = omega_raw`.  Otherwise, before slowdown and
slew limiting:

\[
v_x=\operatorname{clip}(v_x^{raw},0,v_{x,\max}),\qquad
v_y=\operatorname{clip}(v_y^{raw},-v_{y,\max},v_{y,\max}),\qquad
\omega=\omega^{raw}
\]

The nonnegative ordinary `vx` clamp is mandatory: APF repulsion must never
turn into an automatic reverse escape.  Before the command reaches the slew
limiter, validate its finite body-envelope sweep over the command horizon;
if there is no safe lateral/forward direction, command zero.

Let \(q_\min\) be the minimum non-target body-envelope clearance.  Apply the
following explicit translational slowdown to both `vx` and `vy`:

\[
s(q_\min)=\operatorname{clip}
\left(\frac{q_\min-q_s}{q_I-q_s},0,1\right),
\qquad [v_x,v_y]\leftarrow s(q_\min)[v_x,v_y]
\]

Thus obstacles at or beyond `1.2 m` do not change APF motion; at `0.9 m` the
translational reference is half scale; and at/below `0.6 m` it is zero.

Precedence within the motion layer:

- any non-target clearance `<= 0.6 m`: zero.
- `c <= 0.8 m`: zero.
- `0.8 m < c < 1.0 m`: in the first physical walking release, command zero.
  Automatic reverse is disabled. A later release may introduce bounded
  negative `vx` only after independent rear-observability, rear-clearance,
  swept-footprint, and fault-injection validation.
- otherwise, use the turn-in-place gate, force-to-command mapping, and
  `0.6..1.2 m` slowdown above.
- APF local minimum, unavailable safe direction, cancellation, or time-budget
  overrun: zero; APF never commands a reverse escape.

Every accepted fresh cloud is processed once: it performs gated-target
extraction, locked-target confidence evaluation, Kalman correction when confirmed, and APF,
then produces at most one ordinary strategy command intent. The cloud ingress
queue has depth one and
uses latest-frame-wins semantics; it never processes a backlog. Therefore the
ordinary strategy-command frequency equals the actual accepted cloud
frequency, not a fixed `10 Hz`. The runtime measures shared preprocessing,
target-gate extraction, confidence evaluation, APF, and total strategy duration. Any
stage or total-strategy time-budget overrun discards that cloud result and
submits zero; it never permits a late nonzero result. A `40 Hz` watchdog
validates ownership, cloud freshness, strategy age, runtime health, and G1
dispatch health and may submit zero only through `CommandOutputArbiter`.

### 6.2 Command slew-rate limiting

`CommandSlewRateLimiter` sits after APF/target-proximity logic and before the
G1 bridge.  It smooths only ordinary nonzero `FOLLOWING` references.  For each
body-frame component `vx`, `vy`, and `omega`, with measured control interval
`dt`, it limits the change from the last emitted command:

\[
u_{k}=u_{k-1}+\operatorname{clip}
\left(u^{ref}_{k}-u_{k-1},-r^-_u\,dt,r^+_u\,dt\right)
\]

where \(u\in\{v_x,v_y,\omega\}\), \(r^+_u\) is its positive-direction
rate limit and \(r^-_u\) its negative-direction rate limit.  Configuration
must support separate normal acceleration and braking/deceleration limits, so
field tuning can make acceleration gentle while allowing a faster controlled
deceleration.  A sign reversal must decelerate through zero; it must not jump
directly from forward to reverse.

The limiter is not a safety delay.  It is bypassed and reset to zero in the
same control tick for every mandatory-stop event: Web Stop/disconnect,
non-`FOLLOWING` lock state, wristband lifecycle fault, stale/malformed cloud,
G1/APF/watchdog fault, target clearance `<= 0.8 m`, non-target clearance
`<= 0.6 m`, unavailable safe APF direction, or any explicit emergency stop.
Thus a normal reference can ramp down smoothly near desired standoff, while a
safety-required zero is immediate and cannot later resume from stale limiter
state.

## 7. Web and observability

Web controls are **Select target**, **Start follow**, and **Stop follow**.
There is no Web sensor-start/stop control. The server grants one control
lease; only its holder may emit those controls and all other clients are
read-only. Loss of the control-holder connection commands zero and unlocks;
loss of a read-only observer does neither.

The existing AO viewer is the reusable presentation and transport baseline,
not the behavioral contract: retain its WebSocket frame delivery, latest-frame
API, static frontend assets, 360-degree point rendering, and client-message
validation.  Extend its protocol and UI through a mode-specific adapter with
`select_target`, `start_follow`, and `stop_follow` events, lease ownership,
and the payload fields in this section.  Only the backend evaluates a click,
assigns/revokes a lease, or changes lock state; the browser cannot construct a
motion command or declare a selected target valid.

The backend includes `alx_overlay` in every Web frame.  It contains either a
fresh body-frame point or an explicit stale/absent state, plus position age,
range, azimuth, `tag_status`, and session tag ID.  Only a fresh position is
drawn as the existing yellow-green ring.  A stale position must never remain
drawn as a current ALX point; the Web instead shows stale/absent diagnostics.

Web target selection uses a configurable large purple selection circle centred
on the click. The backend uses the circle as the initial local target gate; it
validates the resulting point support, single-target shape, geometry, and ALX
compatibility before creating `CANDIDATE`. It never enumerates global human
clusters or chooses between people. Empty, implausible, over-dispersed, or
multi-peaked support leaves the lock unchanged and returns an explicit status.
The purple selection circle and yellow-green ALX ring are intentionally
distinct: the former is the LiDAR target gate and the latter is the wristband's
transformed position.

Display: 360-degree points, the initial/dynamic target-gate outline, accepted
target-support points, clicked/locked target centre, target clearance,
non-target obstacle points, wristband ON/OFF guardian state, ALX position age
and `tag_status`, dynamically bound session tag ID, \(c_T\)/\(c_G\)/\(c_A\)
confidence components, absolute confidence \(C\), point-support/shape rejection
reason, and final stop reason. ALX's 120-degree sector should be visualized so operators understand
when only LiDAR temporal evidence remains.

Per tick log lifecycle/lock state, controller event sequence, cloud and ALX
ages, session tag ID, tag status, calibration version, raw/preprocessed/
avoidance-representative point counts, target-gate parameters, target-support
count, robust centre, geometry/shape rejection reason, \(c_T\)/\(c_G\)/\(c_A\)
components, absolute confidence \(C\), threshold transition, target clearance, APF vectors/
closest obstacle, APF reference command, slew-limited command, limiter bypass/
reset reason, final command, and G1 latency. Per accepted cloud also log the
shared preprocessing, target-gate extraction, confidence evaluation, APF, and total
strategy durations.

## 8. Configuration

Add a dedicated `lidar_alx_assisted_follow` section without changing existing
ALX-only or pure-LiDAR defaults.  It contains:

- ALX port/protocol, ON/OFF/serial-fault timing, 120-degree FOV metadata,
  fresh-position timeout, `max_alx_cloud_skew_s`, bounded ALX snapshot-buffer
  duration, no-ALX baseline, and position-evidence policy; no static target
  tag ID;
- reusable Livox/FAST-LIO start, readiness, stop, and latest-cloud settings;
- G1 output bridge limits, watchdog/SDK-fault policy, and deployed DDS network
  interface `eth0`;
- fixed ALX-to-body planar calibration (`x_m`, `y_m`, `yaw_deg`), ALX
  azimuth sign and zero offset, height metadata, calibration version,
  validation record, ALX/target/wrist covariances and wrist bias. Missing or
  unvalidated calibration rejects `Start follow`; no default or live
  self-calibration is permitted;
- tracking/avoidance ROIs, shared preprocessing and voxel settings, human
  geometry, initial `web_pick_radius_m`, dynamic-gate axes/displacement rules,
  robust-centre estimator, local support limits, and dispersion/multi-peak
  rejection; local-ground estimation, torso-height band, robust height
  quantiles, `H_ref` update policy, and height-signature residual tolerances;
  per-stage/total strategy time budgets; there is no full-cloud clustering
  setting;
- absolute-confidence weights for \(c_T\), \(c_G\), and \(c_A\), ALX residual-
  to-confidence mapping, no-ALX baseline, `start > stop > unlock`, and
  confidence-stop timeout; no candidate ranking or score-margin settings;
- enabled-by-default `TargetKalmanFilter2D` smoothing, process/measurement
  covariances, innovation gate, maximum accepted frame interval, and reset
  rules; these values affect only the corrected follow reference, never safety
  clearance or identity gates;
- target `1.0 m` desired / `0.8 m` mandatory-stop clearance; first-release
  automatic reverse disabled; APF gains,
  `1.2 m` influence, `0.6 m` non-target hard stop, voxel/representative
  resolution, repulsion cap, `vx`/`vy` gains and caps, yaw gain/cap,
  turn-in-place angle, command horizon, depth-one latest-frame cloud ingress,
  and `40 Hz` zero-only watchdog;
- per-axis `vx`/`vy` acceleration and braking rate limits, `omega` angular
  acceleration and braking limits, accepted `dt` bounds, and limiter reset/
  bypass policy;
- Web control-lease, ownership, and logging policy.

YAML is read at process start.  Numerical values require recorded-bag replay,
then low-speed physical calibration before walking tests.

## 9. Verification and acceptance

Automated tests must cover:

- ON starts sensors and binds the first valid session tag; OFF, serial
  disconnect, and persistent protocol failure zero/unlock/stop sensors then
  retain the read-only guardian for a later ON;
- a healthy ON session with ALX position dropout keeps LiDAR running; expired
  ALX frames no longer calculate or reuse an ALX residual, use the no-ALX baseline, and
  do not alone force a stop merely on leaving `±60°`;
- Web Start requires fresh ALX and `start` confidence; Web Stop or
  control-holder disconnect zero/unlock but do not stop sensors during an ON
  session;
- transform, wrapped-angle, body-to-wrist covariance, calibration-start gate,
  \(c_T\)/\(c_G\)/\(c_A\) and absolute-confidence \(C\) calculation,
  no-ALX baseline, `start > stop > unlock` hysteresis, and the absence of any
  candidate ranking or score-margin path;
- no full-cloud Euclidean clustering or automatic human discovery; prove that
  shared preprocessing runs once per accepted cloud, while target extraction
  uses only vectorized local gates and compact statistics;
- initial purple-gate selection: sufficient point support and plausible
  geometry creates `CANDIDATE`; empty, sparse, over-wide, over-dispersed, or
  multi-peaked support, and ALX incompatibility, leave the lock unchanged;
- dynamic target gate: bounded Kalman-centred displacement, robust-centre
  calculation, innovation rejection, no gate expansion to search for a new
  person, and all non-target points remaining obstacles;
- relative-ground torso-height signature: use robust quantiles rather than a
  raw mean; a person turning in place retains compatible height evidence,
  while low/high non-torso gate contents or a material height-signature change
  lower \(c_G\); rejected evidence must not update `H_ref`;
- target-gate extraction and total-strategy timing: log raw/preprocessed/
  target-support counts and each stage duration; an overrun discards its result
  and produces zero, without blocking the watchdog or allowing a late move;
- two-dimensional constant-velocity Kalman predict/correct behavior, jitter
  reduction of APF target/yaw reference, innovation rejection, and resets;
  prove that `predictOnly`, stale state, or a smoothed value cannot create a
  movement command without a current confidence-confirmed gated target point set;
- confidence below `stop` zeroes but retains lock; lower/expired confidence
  unlocks; no target switch occurs when two people cross or enter the target
  gate together;
- target masking, target clearance, APF attraction/repulsion cap, no APF
  reverse escape, first-release no automatic reverse, zero repulsion at/above
  `1.2 m`, explicit linear `0.6..1.2 m` translational slowdown, `0.6 m`
  body-envelope stop, force-to-command mapping, turn-in-place gate, swept
  command validation, and local-minimum zero;
- slew-rate bounds for ordinary `vx`/`vy`/`omega` changes; sign-reversal
  deceleration through zero; direct-zero bypass and state reset for every
  mandatory-stop path; no stale pre-stop limiter command may resume motion;
- depth-one latest-frame cloud ingress, one strategy intent per accepted cloud,
  stale/wrong-frame cloud, APF/loop/G1 faults, `CommandOutputArbiter` stop
  latch, and watchdog behavior; watchdog never emits movement;
- control-lease authorization, control-holder disconnect stop/unlock, and
  read-only observer disconnect having no control effect;
- backend ALX-polar-to-body conversion, azimuth sign/zero-offset configuration,
  fresh yellow-green overlay versus stale/absent rendering, Web payload ALX
  diagnostics, causal latest-snapshot selection for every cloud, skew-bound
  rejection, and no averaging/interpolation/future-sample selection;
- large purple initial target gate: its point support and rejection reason are
  returned to Web; valid single-target support selects it, while empty,
  implausible, or multi-peaked support returns a status without changing the
  lock.

Recorded bags must include: one person with arm swing, two people crossing,
temporary ALX field-of-view exit while LiDAR remains visible, ALX noise and
dropout during ON, explicit OFF/ON restart, occlusion, a wall/background in
the initial target gate, a second person entering that gate, non-person point
support, and obstacles on both sides. Physical acceptance progresses from
stationary overlay and stop injection to low-speed one-person following, then
obstacles and a second person, with an independent physical stop and observer.
