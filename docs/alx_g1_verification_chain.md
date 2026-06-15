# ALX-G1 Verification Chain

This document describes the step-by-step hardware verification procedure before closing the full follow loop.

## Prerequisites

- Jetson Orin connected to G1 via Ethernet
- `unitree_sdk2_python` installed from **GitHub source** (not PyPI) — see Task 0
- Remote controller online and functional
- Safety operator ready with emergency stop

## ⚠️ Stage 0: Manual Boot Preparation (MUST be done by operator)

G1 does **NOT** boot into standing form. An operator must manually prepare the robot
before any SDK motion commands can succeed. This is done via the remote control, NOT
through our bridge software.

Official boot sequence (source: https://support.unitree.com/home/zh/G1_developer/quick_start):

```text
Step 1: Power on G1 → wait ~1 minute → G1 enters zero-torque state
Step 2: Press L1+A (remote)        → G1 enters damping state
Step 3: Hold G1 shoulder + press L1+UP → human assists G1 into preparation state
Step 4: Press R2+A (remote)        → G1 enters motion state, adjusts gait, STANDS UP
```

Verify G1 is standing and stable before proceeding to Stage 1.

If developing/debugging with SDK: press L2+R2 on remote to enter **debug mode**
(stops built-in motion control to avoid command conflicts).

## Stage 1: SDK Import Verification

```bash
python -c "from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient; print('SDK OK')"
```

Expected: No import errors.

## Stage 2: Official SDK Example Test

Run the official example to confirm SDK ↔ G1 communication:

```bash
cd /tmp/unitree_sdk2_python
python3 example/g1/high_level/g1_loco_client_example.py enp2s0
```

Try: `3` (move forward 0.3 m/s for 1 s), then observe the robot moves.

Expected: Robot moves forward briefly, then stops after 1 second (duration=1).

## Stage 3: Bridge Init Only

Run the dedicated G1 check entry point. This does not read ALX data and does not
enter the follow loop:

```bash
python -m robot_follow.real.g1.check \
  --net-iface enp2s0 \
  --init-only \
  --i-understand-this-moves-robot
```

Expected: bridge initializes and closes without sending a move command.

## Stage 4: StopMove Only (No Motion)

Run with real bridge but only send stop:

```bash
python -m robot_follow.real.g1.check \
  --net-iface enp2s0 \
  --stop-only \
  --i-understand-this-moves-robot
```

Expected: StopMove called successfully (code=0), robot stands still.

## Stage 5: Single Move Burst (0.5-1.0s)

Use most conservative parameters:

```bash
python -m robot_follow.real.g1.check \
  --net-iface enp2s0 \
  --single-move \
  --i-understand-this-moves-robot \
  --max-vx 0.05 \
  --max-omega 0.20 \
  --vx 0.05 \
  --omega 0.0 \
  --duration 0.5
```

This sends a single short move command, then stops. It does not read ALX data.

Expected: Robot moves slowly forward for <1s, then stops. On success the bridge
logs the move at DEBUG and emits NO "error code" line — absence of an "error code"
log means the SetVelocity call returned code 0.

## Stage 6: Continuous Move + Watchdog

Test watchdog timeout behavior:

```bash
# Send move, then pause input for >0.5s
# Watchdog should auto-stop
```

Expected: Robot stops automatically after command timeout.

## Stage 7: SDK Error Code Test

Two ways to trigger a non-zero SDK return code (remember: only `SetVelocity`
returns a code — the bridge calls it directly under Option A, so a non-zero code
IS observable; `Move`/`StopMove` would have swallowed it):

1. **G1 not in motion state**: skip Stage 0 (leave G1 in damping/zero-torque), then
   run a single bounded move. The bridge's `SetVelocity` call returns non-zero and
   the bridge sets `sdk_error`.
2. **Link loss**: disconnect Ethernet mid-run while G1 is in motion state.

```bash
# Scenario 1: do NOT complete Stage 0, then send a single bounded move:
python -m robot_follow.real.g1.check \
  --net-iface enp2s0 \
  --single-move \
  --i-understand-this-moves-robot \
  --max-vx 0.05 \
  --vx 0.05 \
  --duration 0.5
# Bridge should log "error code", transition to ERROR, and attempt StopMove.
```

Expected: Bridge logs "error code", transitions to ERROR state, and sends a
best-effort StopMove. Do not use the ALX closed-loop command for this test.

## Stage 8: Yaw Sign Calibration

Test rotation direction:

```bash
# Target at left (azimuth > 0)
# Observe G1 turn direction
```

If G1 turns wrong direction, set `--yaw-sign -1`.

## Stage 9: Full ALX-G1 Closed Loop

Only after all stages pass:

```bash
python -m robot_follow.main \
  --port /dev/ttyUSB0 \
  --net-iface enp2s0 \
  --enable-motion \
  --i-understand-this-moves-robot \
  --max-vx 0.10 \
  --max-omega 0.30
```

Safety operator must be ready at all times.
