"""Tests for sim/__init__.py (package-level imports)."""

import pytest

from robot_follow.sim import (
    BUILTIN_SCENARIOS,
    MockMotionBridge,
    SimulationResult,
    BuiltinScenarioSource,
    CsvReplaySource,
    get_scenario,
    list_scenarios,
    run_simulation,
)


def test_package_exports_builtins():
    assert BUILTIN_SCENARIOS is not None
    assert len(BUILTIN_SCENARIOS) >= 5


def test_package_exports_bridge():
    bridge = MockMotionBridge()
    assert hasattr(bridge, "dispatch")
    assert hasattr(bridge, "move_count")
    assert hasattr(bridge, "stop_count")


def test_package_exports_result():
    res = SimulationResult()
    assert hasattr(res, "frames")
    assert hasattr(res, "commands")
    assert hasattr(res, "move_count")


def test_package_exports_sources():
    src = BuiltinScenarioSource()
    assert hasattr(src, "__iter__")
    assert hasattr(src, "__len__")


def test_package_exports_helpers():
    assert isinstance(list_scenarios(), list)
    sc = get_scenario("straight_approach")
    assert sc.name == "straight_approach"