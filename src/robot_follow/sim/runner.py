"""Simulation runner: source -> core pipeline -> mock motion bridge.

`run_simulation()` drives a data source through the core FollowPipeline one
frame at a time, dispatching each resulting MotionCommand to a MockMotionBridge.
It returns a SimulationResult holding every per-frame PipelineStepResult and
the recorded bridge.

Determinism
-----------
By default the runner owns a virtual clock that is driven to each sample's
timestamp before processing, so scenario samples are always "fresh" and the
safety manager does not spuriously time out. To exercise data_timeout, pass
both a ``clock`` (a callable you control) and a ``pipeline`` built with that
same clock, then feed samples with stale timestamps — the runner will not
drive a clock it does not own.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Iterable, List, Optional

from robot_follow.core.pipeline import FollowPipeline, PipelineStepResult
from robot_follow.core.types import MotionCommand, TargetSample
from robot_follow.sim.mock_motion_bridge import MockMotionBridge

Clock = Callable[[], float]


class _VirtualClock:
    """A mutable callable clock for deterministic simulation."""

    def __init__(self, start: float = 0.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now


@dataclass
class SimulationResult:
    """Outcome of running a data source through the pipeline.

    Attributes
    ----------
    frames
        Per-frame PipelineStepResult, in processing order.
    commands
        The MotionCommand dispatched on each frame (mirrors frames).
    bridge
        The MockMotionBridge that recorded every command.
    """

    frames: List[PipelineStepResult] = field(default_factory=list)
    commands: List[MotionCommand] = field(default_factory=list)
    bridge: MockMotionBridge = field(default_factory=MockMotionBridge)

    @property
    def moves(self) -> List[MotionCommand]:
        return [c for c in self.commands if c.kind == "move"]

    @property
    def stops(self) -> List[MotionCommand]:
        return [c for c in self.commands if c.kind == "stop"]

    @property
    def move_count(self) -> int:
        return sum(1 for c in self.commands if c.kind == "move")

    @property
    def stop_count(self) -> int:
        return sum(1 for c in self.commands if c.kind == "stop")

    @property
    def last(self) -> Optional[MotionCommand]:
        return self.commands[-1] if self.commands else None

    @property
    def kinds(self) -> List[str]:
        return [c.kind for c in self.commands]

    @property
    def reasons(self) -> List[str]:
        return [c.reason for c in self.commands]

    def __len__(self) -> int:
        return len(self.commands)


def run_simulation(
    source: Iterable[TargetSample],
    *,
    pipeline: Optional[FollowPipeline] = None,
    bridge: Optional[MockMotionBridge] = None,
    clock: Optional[Clock] = None,
) -> SimulationResult:
    """Run a data source through the core pipeline and a mock bridge.

    Parameters
    ----------
    source
        Any iterable of TargetSample (BuiltinScenarioSource, CsvReplaySource,
        or a plain list).
    pipeline
        Optional pre-built FollowPipeline. If omitted, a fresh pipeline is
        built. When both ``pipeline`` and ``clock`` are omitted, the fresh
        pipeline is wired to the runner's virtual clock for determinism.
    bridge
        Optional MockMotionBridge. Defaults to a fresh instance.
    clock
        Optional callable clock. When provided, the runner does NOT drive it
        (the caller controls timing). When omitted, the runner creates and
        drives a virtual clock to each sample's timestamp.

    Returns
    -------
    SimulationResult
    """
    owns_clock = clock is None
    if owns_clock:
        clock = _VirtualClock()

    if pipeline is None:
        # Wire the default pipeline to the clock we are using.
        pipeline = FollowPipeline(clock=clock)

    if bridge is None:
        bridge = MockMotionBridge()

    result = SimulationResult(bridge=bridge)

    for sample in source:
        if owns_clock:
            # Drive the virtual clock so scenario timestamps are fresh.
            clock.now = sample.timestamp  # type: ignore[attr-defined]
        step = pipeline.process(sample)
        bridge.dispatch(step.command)
        result.frames.append(step)
        result.commands.append(step.command)

    return result
