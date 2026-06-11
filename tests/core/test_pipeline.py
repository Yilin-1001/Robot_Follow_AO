"""Unit tests for the core follow pipeline."""

import math
import time

import pytest

from robot_follow.core.filtering import TargetFilter
from robot_follow.core.pipeline import FollowPipeline, PipelineStepResult
from robot_follow.core.safety import SafetyManager
from robot_follow.core.types import MotionCommand, TargetSample


def _sample(
    distance_m: float,
    azimuth_deg: float,
    *,
    valid: bool = True,
    timestamp: float | None = None,
) -> TargetSample:
    return TargetSample(
        timestamp=timestamp if timestamp is not None else time.monotonic(),
        distance_m=distance_m,
        azimuth_deg=azimuth_deg,
        valid=valid,
    )


# --------------------------------------------------------------------------- #
# 1. Normal far distance -> move
# --------------------------------------------------------------------------- #
def test_normal_far_distance_moves():
    pipe = FollowPipeline()
    result = pipe.process(_sample(distance_m=2.5, azimuth_deg=0.0))

    assert isinstance(result, PipelineStepResult)
    assert result.command.kind == "move"
    assert result.command.vx > 0.0
    assert result.filtered is not None
    assert result.filtered.initialized
    assert result.verdict is not None
    assert result.verdict.safe


# --------------------------------------------------------------------------- #
# 2. Large angle -> move, omega != 0, vx == 0
# --------------------------------------------------------------------------- #
def test_large_angle_turns_in_place():
    pipe = FollowPipeline()
    result = pipe.process(_sample(distance_m=2.0, azimuth_deg=30.0))

    assert result.command.kind == "move"
    # Large angle: turn in place, no forward motion
    assert result.command.vx == 0.0
    assert result.command.omega != 0.0
    assert result.command.reason == "aligning"


# --------------------------------------------------------------------------- #
# 3. too_close -> stop
# --------------------------------------------------------------------------- #
def test_too_close_stops():
    pipe = FollowPipeline()
    result = pipe.process(_sample(distance_m=0.5, azimuth_deg=0.0))

    assert result.command.kind == "stop"
    assert result.command.reason == "too_close"
    assert result.command.vx == 0.0
    assert result.command.omega == 0.0


# --------------------------------------------------------------------------- #
# 4. distance_jump -> stop
# --------------------------------------------------------------------------- #
def test_distance_jump_stops():
    pipe = FollowPipeline()
    # First frame initializes the filter at 2.0 m (no jump).
    pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0))
    # Second frame jumps by 0.6 m (> 0.5 m threshold).
    result = pipe.process(_sample(distance_m=2.6, azimuth_deg=0.0))

    assert result.command.kind == "stop"
    assert result.command.reason == "distance_jump"
    assert result.filtered is not None
    assert result.filtered.distance_jump == pytest.approx(0.6)


# --------------------------------------------------------------------------- #
# 5. angle_jump -> stop
# --------------------------------------------------------------------------- #
def test_angle_jump_stops():
    pipe = FollowPipeline()
    pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0))
    # Second frame jumps azimuth by 50 deg (> 45 deg threshold).
    result = pipe.process(_sample(distance_m=2.0, azimuth_deg=50.0))

    assert result.command.kind == "stop"
    assert result.command.reason == "angle_jump"
    assert result.filtered is not None
    assert result.filtered.angle_jump == pytest.approx(50.0)


# --------------------------------------------------------------------------- #
# 6. invalid sample -> stop, and does NOT update the filter
# --------------------------------------------------------------------------- #
def test_invalid_sample_stops_and_skips_filter():
    pipe = FollowPipeline()
    # Initialize the filter at 2.0 m.
    pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0))

    # Feed an invalid sample — it must stop and must NOT touch the filter.
    result = pipe.process(_sample(distance_m=99.0, azimuth_deg=99.0, valid=False))

    assert result.command.kind == "stop"
    assert result.command.reason == "data_invalid"
    assert result.filtered is None
    assert result.verdict is not None
    assert not result.verdict.safe
    assert result.verdict.reason == "data_invalid"

    # Proof the filter was not poisoned by the invalid sample: the next valid
    # sample must compute its jump relative to 2.0 m, not 99.0 m.
    after = pipe.process(_sample(distance_m=2.2, azimuth_deg=0.0))
    assert after.filtered.distance_jump == pytest.approx(0.2)
    assert after.command.kind == "move"


def test_nan_sample_stops_and_does_not_update_filter():
    pipe = FollowPipeline(clock=lambda: 100.0)
    result = pipe.process(TargetSample(
        timestamp=100.0,
        distance_m=math.nan,
        azimuth_deg=0.0,
        valid=True,
    ))

    assert result.command.kind == "stop"
    assert result.command.reason == "data_invalid"
    assert result.filtered is None
    assert result.verdict is not None
    assert not result.verdict.safe
    assert not pipe.filter.initialized


def test_nan_sample_does_not_poison_next_valid_sample():
    pipe = FollowPipeline(clock=lambda: 100.0)
    pipe.process(TargetSample(
        timestamp=100.0,
        distance_m=math.nan,
        azimuth_deg=0.0,
        valid=True,
    ))
    result = pipe.process(TargetSample(
        timestamp=100.0,
        distance_m=2.0,
        azimuth_deg=0.0,
        valid=True,
    ))

    assert result.command.kind == "move"
    assert result.filtered is not None
    assert result.filtered.distance_m == 2.0
    assert result.filtered.distance_jump == 0.0


def test_pipeline_manual_stop_stops_before_filter_update():
    pipe = FollowPipeline()
    result = pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0), manual_stop=True)

    assert result.command.kind == "stop"
    assert result.command.reason == "manual_stop"
    assert result.filtered is None
    assert result.verdict is not None
    assert not result.verdict.safe
    assert not pipe.filter.initialized


def test_pipeline_serial_error_stops_before_filter_update():
    pipe = FollowPipeline()
    result = pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0), serial_error=True)

    assert result.command.kind == "stop"
    assert result.command.reason == "serial_error"
    assert result.filtered is None
    assert result.verdict is not None
    assert not result.verdict.safe
    assert not pipe.filter.initialized


def test_pipeline_sdk_error_stops_before_filter_update():
    pipe = FollowPipeline()
    result = pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0), sdk_error=True)

    assert result.command.kind == "stop"
    assert result.command.reason == "sdk_error"
    assert result.filtered is None
    assert result.verdict is not None
    assert not result.verdict.safe
    assert not pipe.filter.initialized


# --------------------------------------------------------------------------- #
# 7. data_timeout -> stop (deterministic via injected clock)
# --------------------------------------------------------------------------- #
def test_data_timeout_stops():
    fake_now = [100.0]

    def fake_clock() -> float:
        return fake_now[0]

    pipe = FollowPipeline(safety=SafetyManager(clock=fake_clock))

    # Fresh sample (timestamp == now) -> safe.
    fresh = pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0, timestamp=100.0))
    assert fresh.command.kind == "move"

    # Advance the clock past the 0.5 s timeout while the sample stays old.
    fake_now[0] = 100.6
    stale = pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0, timestamp=100.0))

    assert stale.command.kind == "stop"
    assert stale.command.reason == "data_timeout"
    assert stale.verdict is not None
    assert not stale.verdict.safe


def test_stale_sample_stops_before_filter_update():
    fake_now = [100.0]

    def fake_clock() -> float:
        return fake_now[0]

    pipe = FollowPipeline(clock=fake_clock)
    pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0, timestamp=100.0))

    fake_now[0] = 100.6
    stale = pipe.process(_sample(distance_m=9.0, azimuth_deg=90.0, timestamp=100.0))

    assert stale.command.kind == "stop"
    assert stale.command.reason == "data_timeout"
    assert stale.filtered is None

    fake_now[0] = 101.0
    fresh = pipe.process(_sample(distance_m=2.2, azimuth_deg=0.0, timestamp=101.0))

    assert fresh.command.kind == "move"
    assert fresh.filtered is not None
    assert fresh.filtered.distance_jump == pytest.approx(0.2)
    assert fresh.filtered.angle_jump == pytest.approx(0.0)


# --------------------------------------------------------------------------- #
# 8. holding and stop are distinguishable
# --------------------------------------------------------------------------- #
def test_holding_is_move_not_stop():
    pipe = FollowPipeline()
    # At the target distance (1.3 m) with 0 deg -> controller holds.
    result = pipe.process(_sample(distance_m=1.3, azimuth_deg=0.0))

    assert result.command.kind == "move"  # holding is still a "move"
    assert result.command.vx == 0.0
    assert result.command.omega == 0.0
    assert result.command.reason == "holding"


def test_holding_vs_stop_distinguishable():
    # A holding frame is kind="move" (zero velocity, reason="holding").
    pipe_hold = FollowPipeline()
    holding = pipe_hold.process(_sample(distance_m=1.3, azimuth_deg=0.0))

    # A stop frame is kind="stop" (e.g. too_close on the first frame).
    pipe_stop = FollowPipeline()
    stopping = pipe_stop.process(_sample(distance_m=0.5, azimuth_deg=0.0))

    assert holding.command.kind == "move"
    assert holding.command.reason == "holding"
    assert stopping.command.kind == "stop"
    assert stopping.command.reason == "too_close"
    # The two MUST be distinguishable by kind, not just by velocity.
    assert holding.command.kind != stopping.command.kind


def test_safe_move_result_keeps_follow_output():
    pipe = FollowPipeline()
    result = pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0))

    assert result.follow is not None
    assert result.follow.vx == result.command.vx
    assert result.follow.vy == result.command.vy
    assert result.follow.omega == result.command.omega
    assert result.follow.reason == result.command.reason


def test_stop_result_has_no_follow_output():
    pipe = FollowPipeline()
    result = pipe.process(_sample(distance_m=0.5, azimuth_deg=0.0))

    assert result.command.kind == "stop"
    assert result.follow is None


# --------------------------------------------------------------------------- #
# Extra: pipeline respects injected clock via convenience arg
# --------------------------------------------------------------------------- #
def test_pipeline_clock_convenience_arg():
    fake_now = [50.0]

    def fake_clock() -> float:
        return fake_now[0]

    pipe = FollowPipeline(clock=fake_clock)
    # Fresh sample -> move.
    r = pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0, timestamp=50.0))
    assert r.command.kind == "move"
    # Stale sample -> data_timeout.
    fake_now[0] = 51.0
    r2 = pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0, timestamp=50.0))
    assert r2.command.kind == "stop"
    assert r2.command.reason == "data_timeout"


# --------------------------------------------------------------------------- #
# Extra: command type is always MotionCommand
# --------------------------------------------------------------------------- #
def test_command_is_motion_command():
    pipe = FollowPipeline()
    result = pipe.process(_sample(distance_m=2.5, azimuth_deg=0.0))
    assert isinstance(result.command, MotionCommand)
    assert result.command.kind in ("move", "stop")


def test_pipeline_exported_from_core_package():
    from robot_follow.core import FollowPipeline as ExportedFollowPipeline
    from robot_follow.core import PipelineStepResult as ExportedPipelineStepResult

    assert ExportedFollowPipeline is FollowPipeline
    assert ExportedPipelineStepResult is PipelineStepResult


def test_pipeline_accepts_target_filter_keyword():
    target_filter = TargetFilter(alpha=1.0)
    pipe = FollowPipeline(target_filter=target_filter)
    result = pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0))

    assert pipe.filter is target_filter
    assert result.filtered is not None
    assert result.filtered.distance_m == 2.0


def test_pipeline_keeps_filter_keyword_compatibility():
    target_filter = TargetFilter(alpha=1.0)
    pipe = FollowPipeline(filter=target_filter)
    result = pipe.process(_sample(distance_m=2.0, azimuth_deg=0.0))

    assert pipe.filter is target_filter
    assert result.filtered is not None
    assert result.filtered.distance_m == 2.0


def test_pipeline_rejects_both_filter_keywords():
    with pytest.raises(ValueError):
        FollowPipeline(
            target_filter=TargetFilter(alpha=1.0),
            filter=TargetFilter(alpha=1.0),
        )
