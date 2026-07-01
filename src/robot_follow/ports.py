"""Output-side interfaces for robot_follow adapters."""

from __future__ import annotations

from typing import Protocol, Union

from robot_follow.core.types import MotionCommand


class MotionBridge(Protocol):
    """Interface for motion output bridges.

    Both ``MockMotionBridge`` (simulation / dry-run) and ``G1MotionBridge``
    (real robot) implement this protocol. The main loop depends only on this
    interface, so dry-run and real-run share the same control flow.
    """

    @property
    def sdk_error(self) -> bool:
        """Whether an SDK error has occurred (always False for mock bridges)."""
        ...

    @property
    def sdk_timeout(self) -> bool:
        """Whether SDK requests are timing out (always False for mock bridges)."""
        ...

    def init(self) -> None:
        """Initialize the bridge (DDS + SDK client, or no-op for mock)."""
        ...

    def dispatch(self, command: "MotionCommand") -> None:
        """Dispatch a MotionCommand to the appropriate handler."""
        ...

    def move(self, command: "MotionCommand") -> None:
        """Handle a move command."""
        ...

    def stop(self, reason: Union[str, "MotionCommand"]) -> None:
        """Handle a stop command."""
        ...

    def tick(self, now: float | None = None) -> None:
        """Watchdog tick — call once per main-loop iteration."""
        ...

    def close(self) -> None:
        """Clean shutdown."""
        ...
