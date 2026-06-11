"""Core layer: hardware-independent algorithms for the follow system.

This module provides:
- TargetSample / MotionCommand types
- TargetFilter with jump detection
- SafetyManager with configurable thresholds
- FollowController for velocity computation
- FollowPipeline orchestration
"""

from robot_follow.core.types import MotionCommand, TargetSample
from robot_follow.core.filtering import FilteredTarget, TargetFilter
from robot_follow.core.safety import SafetyConfig, SafetyInput, SafetyManager, SafetyVerdict
from robot_follow.core.follow_controller import FollowConfig, FollowController, FollowOutput
from robot_follow.core.pipeline import FollowPipeline, PipelineStepResult

__all__ = [
    "TargetSample",
    "MotionCommand",
    "FilteredTarget",
    "TargetFilter",
    "SafetyConfig",
    "SafetyInput",
    "SafetyManager",
    "SafetyVerdict",
    "FollowConfig",
    "FollowController",
    "FollowOutput",
    "FollowPipeline",
    "PipelineStepResult",
]
