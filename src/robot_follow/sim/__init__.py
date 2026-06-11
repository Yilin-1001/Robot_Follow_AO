"""Offline simulation layer for the follow system.

This package wires the hardware-independent core pipeline to simulated data
sources and a recording-only motion bridge, so the full software chain can be
exercised without any ALX-G100 hardware or Unitree G1 SDK.

Contents
--------
- scenarios         : builtin named scenarios (straight_approach, ...)
- data_source       : BuiltinScenarioSource, CsvReplaySource
- mock_motion_bridge: MockMotionBridge (records commands, drives nothing)
- runner            : run_simulation() — source -> core pipeline -> bridge

Constraints
-----------
- No import of unitree_sdk2_python.
- No serial port access.
- No real robot control.
"""

from robot_follow.sim.scenarios import (
    BUILTIN_SCENARIOS,
    Scenario,
    get_scenario,
    list_scenarios,
)
from robot_follow.sim.data_source import BuiltinScenarioSource, CsvReplaySource
from robot_follow.sim.mock_motion_bridge import MockMotionBridge
from robot_follow.sim.runner import SimulationResult, run_simulation

__all__ = [
    "BUILTIN_SCENARIOS",
    "Scenario",
    "get_scenario",
    "list_scenarios",
    "BuiltinScenarioSource",
    "CsvReplaySource",
    "MockMotionBridge",
    "SimulationResult",
    "run_simulation",
]
