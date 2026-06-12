"""Tests for sim/runner.py — run_simulation and SimulationResult."""

import pytest

from robot_follow.core.pipeline import FollowPipeline
from robot_follow.core.safety import SafetyManager
from robot_follow.core.types import MotionCommand, TargetSample
from robot_follow.sim.data_source import BuiltinScenarioSource
from robot_follow.sim.mock_motion_bridge import MockMotionBridge
from robot_follow.sim.runner import SimulationResult, run_simulation
from robot_follow.sim.scenarios import BUILTIN_SCENARIOS


class TestSimulationResult:
    def test_empty_result(self):
        res = SimulationResult()
        assert len(res) == 0
        assert res.move_count == 0
        assert res.stop_count == 0
        assert res.last is None
        assert res.kinds == []
        assert res.reasons == []

    def test_counts(self):
        res = SimulationResult(
            frames=[],
            commands=[
                MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.0, reason="a"),
                MotionCommand(kind="move", vx=0.2, vy=0.0, omega=0.0, reason="b"),
                MotionCommand(kind="stop", vx=0.0, vy=0.0, omega=0.0, reason="c"),
            ],
            bridge=MockMotionBridge(),
        )
        assert res.move_count == 2
        assert res.stop_count == 1
        assert len(res) == 3
        assert res.kinds == ["move", "move", "stop"]
        assert res.reasons == ["a", "b", "c"]


class TestRunSimulationDefault:
    def test_accepts_plain_list(self):
        samples = [
            TargetSample(timestamp=1000.0, distance_m=2.5, azimuth_deg=0.0),
            TargetSample(timestamp=1000.05, distance_m=2.4, azimuth_deg=0.0),
        ]
        res = run_simulation(samples)
        assert len(res) == 2
        assert all(c.kind == "move" for c in res.commands)

    def test_accepts_builtin_source(self):
        src = BuiltinScenarioSource("jump")
        res = run_simulation(src)
        assert len(res) == 4

    def test_creates_default_bridge_and_pipeline(self):
        samples = [
            TargetSample(timestamp=1000.0, distance_m=2.0, azimuth_deg=0.0),
        ]
        res = run_simulation(samples)
        assert res.bridge is not None
        assert isinstance(res.bridge, MockMotionBridge)
        assert len(res.frames) == 1
        assert isinstance(res.frames[0], object)


class TestRunSimulationInjectedComponents:
    def test_accepts_prebuilt_bridge(self):
        bridge = MockMotionBridge()
        samples = [
            TargetSample(timestamp=1000.0, distance_m=2.0, azimuth_deg=0.0),
        ]
        res = run_simulation(samples, bridge=bridge)
        assert res.bridge is bridge
        assert bridge.move_count == 1

    def test_accepts_prebuilt_pipeline(self):
        # Build a pipeline with a fixed clock so the sample is fresh.
        pipe = FollowPipeline(clock=lambda: 1000.0)
        samples = [
            TargetSample(timestamp=1000.0, distance_m=0.49, azimuth_deg=0.0),  # too_close
        ]
        res = run_simulation(samples, pipeline=pipe)
        assert res.stop_count == 1
        assert res.last.reason == "too_close"

    def test_uses_passed_pipeline_clock(self):
        # Pass a clock and a pipeline wired to that clock.
        fake_now = [500.0]

        def clock():
            return fake_now[0]

        pipe = FollowPipeline(clock=clock)
        # Sample timestamp matches clock — fresh.
        res1 = run_simulation(
            [TargetSample(timestamp=500.0, distance_m=2.0, azimuth_deg=0.0)],
            pipeline=pipe,
            clock=clock,
        )
        assert res1.move_count == 1

        # Advance clock beyond timeout — stale sample.
        fake_now[0] = 501.0  # > 0.5s timeout
        res2 = run_simulation(
            [TargetSample(timestamp=500.0, distance_m=2.0, azimuth_deg=0.0)],
            pipeline=pipe,
            clock=clock,
        )
        assert res2.stop_count == 1
        assert res2.last.reason == "data_timeout"


class TestRunSimulationScenarioBehaviors:
    """Integration tests running each builtin scenario through the pipeline."""

    def test_straight_approach_all_move(self):
        res = run_simulation(BUILTIN_SCENARIOS["straight_approach"].samples)
        assert res.stop_count == 0
        assert res.move_count == len(res)
        # As distance decreases from 2.5m into the hold band (1.35-1.65m),
        # early frames are tracking_forward and later frames transition to
        # holding or tracking_backward. No safety stops occur on approach.
        assert any(c.reason == "tracking_forward" for c in res.commands)

    def test_straight_approach_vx_decreases(self):
        res = run_simulation(BUILTIN_SCENARIOS["straight_approach"].samples)
        vx_vals = [c.vx for c in res.commands]
        assert vx_vals[0] > vx_vals[-1]  # early vx larger than final vx

    def test_angle_alignment_all_move(self):
        res = run_simulation(BUILTIN_SCENARIOS["angle_alignment"].samples)
        assert res.stop_count == 0
        assert res.move_count == len(res)

    def test_angle_alignment_first_frames_aligning_vx_zero(self):
        res = run_simulation(BUILTIN_SCENARIOS["angle_alignment"].samples)
        # First 5 frames have large angles -> aligning (vx=0)
        for i, cmd in enumerate(res.commands[:5]):
            assert cmd.reason == "aligning"
            assert cmd.vx == 0.0
            assert cmd.omega != 0.0

    def test_angle_alignment_later_frames_turning_or_forward(self):
        res = run_simulation(BUILTIN_SCENARIOS["angle_alignment"].samples)
        # After aligning, we get tracking_turning (medium angle) or tracking_forward
        for cmd in res.commands[5:]:
            assert cmd.kind == "move"
            assert cmd.vx >= 0.0

    def test_too_close_ends_with_stop(self):
        res = run_simulation(BUILTIN_SCENARIOS["too_close"].samples)
        assert res.stop_count >= 1
        assert any(c.reason == "too_close" for c in res.stops)

    def test_too_close_early_frames_are_move(self):
        res = run_simulation(BUILTIN_SCENARIOS["too_close"].samples)
        # Early frames before safety threshold are moves
        assert res.move_count >= 5
        # All distances are below the hold band, so every move reverses
        # (negative vx) under the bidirectional controller.
        for cmd in res.moves:
            assert cmd.vx <= 0.0

    def test_jump_distance_jump_stop(self):
        res = run_simulation(BUILTIN_SCENARIOS["jump"].samples)
        assert res.stop_count == 1
        assert res.last.reason == "distance_jump"

    def test_jump_early_frames_move(self):
        res = run_simulation(BUILTIN_SCENARIOS["jump"].samples)
        assert res.move_count == 3
        for cmd in res.moves:
            assert cmd.vx > 0  # forward motion while stable at 2.0m

    def test_dropout_ends_with_data_invalid_stop(self):
        res = run_simulation(BUILTIN_SCENARIOS["dropout"].samples)
        assert res.stop_count >= 1
        assert all(c.reason == "data_invalid" for c in res.stops)

    def test_dropout_early_frames_move(self):
        res = run_simulation(BUILTIN_SCENARIOS["dropout"].samples)
        assert res.move_count == 2


class TestRunSimulationDropoutMechanism:
    """Verify that dropout is processed as invalid samples, not just truncation."""

    def test_dropout_not_truncated(self):
        sc = BUILTIN_SCENARIOS["dropout"]
        res = run_simulation(sc.samples)
        # Must process all 5 frames (not stop at 2)
        assert len(res) == len(sc.samples) == 5

    def test_dropout_invalid_frames_do_not_update_filter(self):
        # This is tested at the core pipeline level; here we confirm the runner
        # processes invalid frames and the pipeline emits data_invalid.
        res = run_simulation(BUILTIN_SCENARIOS["dropout"].samples)
        # Invalid frames: index 2,3,4
        invalid_frame_indices = [2, 3, 4]
        for i in invalid_frame_indices:
            assert res.frames[i].filtered is None
            assert res.frames[i].command.reason == "data_invalid"


class TestRunSimulationVirtualClock:
    """Verify the runner drives its virtual clock correctly."""

    def test_samples_are_fresh_by_default(self):
        # Use a sample with a fixed timestamp. With the default virtual clock,
        # the runner sets clock.now = sample.timestamp before processing,
        # so the sample is fresh (no timeout).
        sample = TargetSample(timestamp=12345.0, distance_m=2.0, azimuth_deg=0.0)
        res = run_simulation([sample])
        assert res.move_count == 1
        assert res.last.reason != "data_timeout"

    def test_virtual_clock_driven_per_sample(self):
        # Two samples with different timestamps; both should be fresh.
        samples = [
            TargetSample(timestamp=100.0, distance_m=2.0, azimuth_deg=0.0),
            TargetSample(timestamp=200.0, distance_m=2.0, azimuth_deg=0.0),
        ]
        res = run_simulation(samples)
        assert res.move_count == 2


class TestRunSimulationBridgeRecording:
    def test_bridge_history_matches_commands(self):
        src = BuiltinScenarioSource("jump")
        res = run_simulation(src)
        assert len(res.bridge.history) == len(res.commands)
        assert res.bridge.history == res.commands


class TestRunSimulationLen:
    def test_len_matches_sample_count(self):
        for name in ["straight_approach", "jump", "dropout"]:
            src = BuiltinScenarioSource(name)
            res = run_simulation(src)
            assert len(res) == len(src)