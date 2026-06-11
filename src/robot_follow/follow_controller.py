"""Backward-compatible re-export of robot_follow.core.follow_controller.

The implementation now lives in :mod:`robot_follow.core.follow_controller`. This thin
wrapper keeps the legacy ``robot_follow.follow_controller`` import path working.
"""

from robot_follow.core.follow_controller import (
    FollowConfig,
    FollowController,
    FollowOutput,
)

__all__ = [
    "FollowConfig",
    "FollowController",
    "FollowOutput",
]
