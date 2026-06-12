# Bidirectional Follow Distance Design

## Goal

Update the follow behavior so the robot maintains a realistic distance from the person instead of only approaching. When the person gets closer than the comfortable range, the robot should reverse slowly; when the person moves farther away, it should move forward slowly.

## Real-World Distance Model

The controller uses a distance band instead of a single exact target. This prevents jitter caused by normal human motion, arm swing, tag movement, and sensor noise.

Recommended distances:

```text
target_distance_m = 1.5
hold_min_distance_m = 1.35
hold_max_distance_m = 1.65
min_safe_distance_m = 0.5
max_follow_distance_m = 3.0
```

Behavior by distance:

```text
distance < 0.5m        -> stop, reason="too_close"
0.5m <= distance < 1.35m -> reverse slowly
1.35m <= distance <= 1.65m -> hold position
1.65m < distance <= 3.0m -> move forward slowly
distance > 3.0m        -> stop for MVP, reason="target_too_far"
```

The 30 cm hold band around 1.5 m reflects normal standing sway and small step adjustments. The robot should not react to every minor movement.

## Control Strategy

The existing "align first, then approach" strategy remains:

1. If the target angle is large, rotate in place and do not move forward or backward.
2. If the target angle is medium, combine turning with reduced forward or backward speed.
3. If the target angle is small, use distance error to decide forward, hold, or reverse.

The distance controller becomes bidirectional:

```text
forward_error = distance_m - hold_max_distance_m
backward_error = distance_m - hold_min_distance_m
```

Forward speed is positive when the person is beyond the hold band. Backward speed is negative when the person is inside the hold band but still outside the minimum safety distance.

Recommended speed limits:

```text
max_forward_vx = 0.20 m/s
max_backward_vx = 0.12 m/s
distance_gain_forward = 0.18
distance_gain_backward = 0.15
```

Backward speed is intentionally lower than forward speed because reversing a humanoid platform near a person has higher practical risk and should feel conservative.

## Safety

Safety remains separate from distance regulation.

The safety manager should stop the robot when:

```text
distance_m < 0.5
data is invalid
data times out
distance or angle jumps exceed configured thresholds
manual stop, serial error, or SDK error is active
```

The controller may also return `too_close` if called directly with a distance below `min_safe_distance_m`, but the pipeline safety layer is the authoritative stop gate.

For the MVP, distances above 3.0 m should not trigger faster chasing. The safer default is to stop until a future search/reacquisition behavior is designed.

## Configuration Impact

`FollowConfig` should gain explicit parameters for the hold band and asymmetric speed limits:

```text
target_distance_m = 1.5
min_safe_distance_m = 0.5
hold_tolerance_m = 0.15
max_forward_vx = 0.20
max_backward_vx = 0.12
max_follow_distance_m = 3.0
distance_gain_forward = 0.18
distance_gain_backward = 0.15
```

The existing `max_vx` can be kept as a compatibility alias or replaced carefully if no external callers depend on it.

`SafetyConfig.min_safe_distance_m` should also default to `0.5` so direct safety checks and controller config agree.

## Testing

Add focused tests for:

- distance below 0.5 m stops as `too_close`
- distance between 0.5 m and 1.35 m produces negative `vx`
- distance in 1.35 m to 1.65 m produces holding output
- distance above 1.65 m produces positive `vx`
- backward speed is clamped to `max_backward_vx`
- large angle still rotates in place with `vx == 0`
- pipeline emits a move command with negative `vx` for valid close-but-safe samples

Existing tests for invalid data, timeout, jumps, and stop command invariants should continue to pass.
