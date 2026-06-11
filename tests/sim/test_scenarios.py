"""Tests for sim/scenarios.py — builtin scenario definitions."""

import pytest

from robot_follow.core.types import TargetSample
from robot_follow.sim.scenarios import (
    BUILTIN_SCENARIOS,
    Scenario,
    get_scenario,
    list_scenarios,
)


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------
class TestRegistry:
    def test_list_scenarios_returns_all_names(self):
        names = list_scenarios()
        assert "straight_approach" in names
        assert "angle_alignment" in names
        assert "too_close" in names
        assert "jump" in names
        assert "dropout" in names
        assert len(names) >= 5

    def test_get_scenario_returns_scenario(self):
        sc = get_scenario("straight_approach")
        assert isinstance(sc, Scenario)
        assert sc.name == "straight_approach"
        assert len(sc.samples) > 0

    def test_get_scenario_case_insensitive(self):
        sc = get_scenario("Straight_Approach")
        assert sc.name == "straight_approach"

    def test_get_scenario_unknown_raises(self):
        with pytest.raises(KeyError, match="Unknown scenario"):
            get_scenario("nonexistent")

    def test_builtin_scenarios_dict_has_all(self):
        for name in list_scenarios():
            assert name in BUILTIN_SCENARIOS
            assert isinstance(BUILTIN_SCENARIOS[name], Scenario)


# ---------------------------------------------------------------------------
# Scenario structure
# ---------------------------------------------------------------------------
class TestScenarioStructure:
    def test_all_samples_are_target_sample(self):
        for name in list_scenarios():
            sc = get_scenario(name)
            for s in sc.samples:
                assert isinstance(s, TargetSample)

    def test_all_samples_have_valid_timestamps(self):
        for name in list_scenarios():
            sc = get_scenario(name)
            for s in sc.samples:
                assert s.timestamp > 0

    def test_all_samples_have_non_negative_distance(self):
        for name in list_scenarios():
            sc = get_scenario(name)
            for s in sc.samples:
                assert s.distance_m >= 0

    def test_scenario_is_frozen(self):
        sc = get_scenario("straight_approach")
        with pytest.raises(AttributeError):
            sc.name = "other"  # type: ignore[misc]

    def test_scenario_has_description(self):
        for name in list_scenarios():
            sc = get_scenario(name)
            assert isinstance(sc.description, str)
            assert len(sc.description) > 0


# ---------------------------------------------------------------------------
# Individual scenario content
# ---------------------------------------------------------------------------
class TestStraightApproach:
    def test_has_16_frames(self):
        sc = get_scenario("straight_approach")
        assert len(sc.samples) == 16

    def test_distance_decreases(self):
        sc = get_scenario("straight_approach")
        distances = [s.distance_m for s in sc.samples]
        # Overall decreasing trend
        assert distances[0] > distances[-1]
        assert distances[0] == pytest.approx(2.5)
        # Last 4 frames are at target distance
        assert distances[-4] == pytest.approx(1.3)

    def test_azimuth_zero(self):
        sc = get_scenario("straight_approach")
        assert all(s.azimuth_deg == 0.0 for s in sc.samples)


class TestAngleAlignment:
    def test_has_9_frames(self):
        sc = get_scenario("angle_alignment")
        assert len(sc.samples) == 9

    def test_distance_fixed(self):
        sc = get_scenario("angle_alignment")
        assert all(s.distance_m == 2.0 for s in sc.samples)

    def test_angle_starts_large_ends_small(self):
        sc = get_scenario("angle_alignment")
        assert sc.samples[0].azimuth_deg == pytest.approx(30.0)
        assert sc.samples[-1].azimuth_deg == pytest.approx(0.0)

    def test_all_valid(self):
        sc = get_scenario("angle_alignment")
        assert all(s.valid for s in sc.samples)


class TestTooClose:
    def test_has_multiple_frames(self):
        sc = get_scenario("too_close")
        assert len(sc.samples) >= 5

    def test_distance_decreases(self):
        sc = get_scenario("too_close")
        distances = [s.distance_m for s in sc.samples]
        assert distances[0] > distances[-1]
        # Final distance well below safe minimum
        assert distances[-1] < 0.5

    def test_small_steps_no_big_jump(self):
        """Steps are small enough to avoid distance_jump (max 0.5m)."""
        sc = get_scenario("too_close")
        for i in range(1, len(sc.samples)):
            step = abs(sc.samples[i].distance_m - sc.samples[i - 1].distance_m)
            assert step < 0.5, f"Frame {i} step {step} >= 0.5m"


class TestJump:
    def test_has_4_frames(self):
        sc = get_scenario("jump")
        assert len(sc.samples) == 4

    def test_first_three_stable(self):
        sc = get_scenario("jump")
        for s in sc.samples[:3]:
            assert s.distance_m == pytest.approx(2.0)

    def test_last_frame_big_jump(self):
        sc = get_scenario("jump")
        jump_size = sc.samples[3].distance_m - sc.samples[2].distance_m
        assert jump_size > 1.0  # much bigger than the 0.5m threshold


class TestDropout:
    def test_has_5_frames(self):
        sc = get_scenario("dropout")
        assert len(sc.samples) == 5

    def test_first_two_valid(self):
        sc = get_scenario("dropout")
        assert sc.samples[0].valid is True
        assert sc.samples[1].valid is True

    def test_last_three_invalid(self):
        sc = get_scenario("dropout")
        for s in sc.samples[2:]:
            assert s.valid is False

    def test_dropout_is_not_just_truncation(self):
        """Dropout must produce invalid samples, not just stop iterating."""
        sc = get_scenario("dropout")
        assert len(sc.samples) > 2  # more than just the valid phase
        invalid_count = sum(1 for s in sc.samples if not s.valid)
        assert invalid_count >= 3