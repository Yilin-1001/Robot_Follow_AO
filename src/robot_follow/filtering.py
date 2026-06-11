"""Backward-compatible re-export of robot_follow.core.filtering.

The implementation now lives in :mod:`robot_follow.core.filtering`. This thin
wrapper keeps the legacy ``robot_follow.filtering`` import path working.
"""

from robot_follow.core.filtering import (
    FilteredTarget,
    TargetFilter,
)

__all__ = [
    "FilteredTarget",
    "TargetFilter",
]
