"""Tests for sim/data_source.py — BuiltinScenarioSource."""

import pytest

from robot_follow.core.types import TargetSample
from robot_follow.sim.data_source import BuiltinScenarioSource
from robot_follow.sim.scenarios import get_scenario


class TestBuiltinScenarioSource:
    def test_default_scenario_is_straight_approach(self):
        src = BuiltinScenarioSource()
        assert src.name == "straight_approach"

    def test_select_scenario_by_name(self):
        src = BuiltinScenarioSource("angle_alignment")
        assert src.name == "angle_alignment"

    def test_select_scenario_case_insensitive(self):
        src = BuiltinScenarioSource("ANGLE_ALIGNMENT")
        assert src.name == "angle_alignment"

    def test_unknown_scenario_raises(self):
        with pytest.raises(KeyError, match="Unknown scenario"):
            BuiltinScenarioSource("nonexistent")

    def test_accepts_scenario_object(self):
        sc = get_scenario("jump")
        src = BuiltinScenarioSource(sc)
        assert src.scenario is sc

    def test_iterates_target_samples(self):
        src = BuiltinScenarioSource("jump")
        samples = list(src)
        assert len(samples) == 4
        for s in samples:
            assert isinstance(s, TargetSample)

    def test_len_matches_scenario(self):
        src = BuiltinScenarioSource("straight_approach")
        assert len(src) == len(get_scenario("straight_approach").samples)

    def test_iterable_multiple_times(self):
        src = BuiltinScenarioSource("jump")
        first = list(src)
        second = list(src)
        assert first == second
        assert len(first) == len(second) == 4

    def test_samples_property(self):
        src = BuiltinScenarioSource("dropout")
        assert len(src.samples) == 5
        # Last three should be invalid
        assert all(not s.valid for s in src.samples[2:])

    def test_feeds_directly_into_list(self):
        # A source should be usable wherever an iterable is expected.
        src = BuiltinScenarioSource("too_close")
        distances = [s.distance_m for s in src]
        assert distances[0] > distances[-1]