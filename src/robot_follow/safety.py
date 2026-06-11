"""Backward-compatible re-export of robot_follow.core.safety.

The implementation now lives in :mod:`robot_follow.core.safety`. This thin
wrapper keeps the legacy ``robot_follow.safety`` import path working.
"""

from robot_follow.core.safety import (
    SafetyConfig,
    SafetyInput,
    SafetyManager,
    SafetyVerdict,
)

__all__ = [
    "SafetyConfig",
    "SafetyInput",
    "SafetyManager",
    "SafetyVerdict",
]
