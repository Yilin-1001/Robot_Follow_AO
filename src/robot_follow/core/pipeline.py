"""Core follow pipeline: hardware-independent one-frame processing chain.

The pipeline wires together the three core components (filter, safety,
controller) into a single per-frame processing chain:

    TargetSample
    -> if sample.valid is False: stop, do NOT update filter
    -> TargetFilter.update()
    -> SafetyManager.check()
    -> if unsafe: MotionCommand(kind="stop")
    -> if safe:   FollowController.compute() -> MotionCommand(kind="move")

Key invariants
--------------
* Invalid samples never reach the filter.
* ``stop`` is distinct from ``holding``: a holding frame is
  ``kind="move"`` with ``vx=0, omega=0, reason="holding"``; a stop frame is
  always ``kind="stop"``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional

from robot_follow.core.filtering import FilteredTarget, TargetFilter
from robot_follow.core.follow_controller import FollowConfig, FollowController, FollowOutput
from robot_follow.core.safety import SafetyConfig, SafetyInput, SafetyManager, SafetyVerdict
from robot_follow.core.types import MotionCommand, TargetSample


@dataclass(frozen=True)
class PipelineStepResult:
    """Outcome of processing a single TargetSample.

    Attributes
    ----------
    sample
        The TargetSample that was processed.
    filtered
        FilteredTarget produced by the filter, or None if the sample was
        invalid and skipped the filter.
    verdict
        SafetyVerdict from the safety manager, or None if the sample was
        invalid (the filter/safety/controller were not consulted).
    command
        Final MotionCommand for this frame.
    follow
        Raw FollowOutput from the controller for safe move frames, or None
        when the pipeline stops before or instead of controller output.
    """

    sample: TargetSample
    filtered: Optional[FilteredTarget]
    verdict: Optional[SafetyVerdict]
    command: MotionCommand
    follow: Optional[FollowOutput] = None


def _stop_command(reason: str) -> MotionCommand:
    """Build a stop command with zero velocity and the given reason."""
    return MotionCommand(kind="stop", vx=0.0, vy=0.0, omega=0.0, reason=reason)


def _invalid_result(sample: TargetSample, reason: str) -> PipelineStepResult:
    verdict = SafetyVerdict(safe=False, reason=reason)
    return PipelineStepResult(
        sample=sample,
        filtered=None,
        verdict=verdict,
        command=_stop_command(reason),
    )


class FollowPipeline:
    """One-frame processing chain for the follow behaviour.

    Parameters
    ----------
    target_filter
        Optional TargetFilter. Defaults to TargetFilter().
    filter
        Backward-compatible alias for ``target_filter``. Do not pass both.
    safety
        Optional SafetyManager. Takes precedence over the ``clock`` argument.
        Pass one with an injected ``clock`` for deterministic timeout tests.
    controller
        Optional FollowController. Defaults to FollowController().
    clock
        Optional zero-argument callable returning the current time. Used only
        when ``safety`` is not supplied, to build a SafetyManager with an
        injected clock. Defaults to None (SafetyManager defaults to
        time.monotonic).
    """

    def __init__(
        self,
        target_filter: Optional[TargetFilter] = None,
        safety: Optional[SafetyManager] = None,
        controller: Optional[FollowController] = None,
        clock: Optional[Callable[[], float]] = None,
        *,
        filter: Optional[TargetFilter] = None,
    ) -> None:
        if target_filter is not None and filter is not None:
            raise ValueError("pass either target_filter or filter, not both")
        self._filter = target_filter or filter or TargetFilter()
        if safety is not None:
            self._safety = safety
        elif clock is not None:
            self._safety = SafetyManager(clock=clock)
        else:
            self._safety = SafetyManager()
        self._controller = controller or FollowController()

    @property
    def filter(self) -> TargetFilter:
        return self._filter

    @property
    def safety(self) -> SafetyManager:
        return self._safety

    @property
    def controller(self) -> FollowController:
        return self._controller

    def reset(self) -> None:
        """Reset internal state (the filter). Safety/controller are stateless."""
        self._filter.reset()

    def process(
        self,
        sample: TargetSample,
        *,
        manual_stop: bool = False,
        serial_error: bool = False,
        sdk_error: bool = False,
    ) -> PipelineStepResult:
        """Process one TargetSample and return the per-frame result.

        Invalid samples produce a stop command and do NOT update the filter.
        """
        pre_filter_safety = SafetyInput(
            timestamp=sample.timestamp,
            distance_m=sample.distance_m,
            azimuth_deg=sample.azimuth_deg,
            distance_jump=0.0,
            angle_jump=0.0,
            data_valid=sample.valid,
            manual_stop=manual_stop,
            serial_error=serial_error,
            sdk_error=sdk_error,
        )
        pre_filter_verdict = self._safety.check(pre_filter_safety)
        if not pre_filter_verdict.safe and pre_filter_verdict.reason in {
            "manual_stop",
            "serial_error",
            "sdk_error",
            "data_invalid",
            "data_timeout",
        }:
            return PipelineStepResult(
                sample=sample,
                filtered=None,
                verdict=pre_filter_verdict,
                command=_stop_command(pre_filter_verdict.reason),
            )

        # Step 1: invalid sample -> stop, skip the filter entirely.
        if not sample.valid:
            return _invalid_result(sample, "data_invalid")

        if not all(math.isfinite(value) for value in (
            sample.timestamp,
            sample.distance_m,
            sample.azimuth_deg,
        )):
            return _invalid_result(sample, "data_invalid")

        # Step 2: update the filter with the valid sample.
        filtered = self._filter.update(sample.distance_m, sample.azimuth_deg)

        # Step 3: safety check on the filtered result.
        safety_input = SafetyInput(
            timestamp=sample.timestamp,
            distance_m=filtered.distance_m,
            azimuth_deg=filtered.azimuth_deg,
            distance_jump=filtered.distance_jump,
            angle_jump=filtered.angle_jump,
            data_valid=True,
            manual_stop=manual_stop,
            serial_error=serial_error,
            sdk_error=sdk_error,
        )
        verdict = self._safety.check(safety_input)

        # Step 4: unsafe -> stop (distinct from a safe holding "move").
        if not verdict.safe:
            command = _stop_command(verdict.reason)
            return PipelineStepResult(
                sample=sample,
                filtered=filtered,
                verdict=verdict,
                command=command,
            )

        # Step 4b: target too far -> stop instead of chasing (MVP).
        if filtered.distance_m > self._controller.config.max_follow_distance_m:
            command = _stop_command("target_too_far")
            return PipelineStepResult(
                sample=sample,
                filtered=filtered,
                verdict=verdict,
                command=command,
            )

        # Step 5: safe -> controller -> move.
        output = self._controller.compute(
            filtered.distance_m, filtered.azimuth_deg
        )
        command = MotionCommand(
            kind="move",
            vx=output.vx,
            vy=output.vy,
            omega=output.omega,
            reason=output.reason,
        )
        return PipelineStepResult(
            sample=sample,
            filtered=filtered,
            verdict=verdict,
            command=command,
            follow=output,
        )
