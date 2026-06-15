"""Mock motion bridge for offline simulation.

A motion bridge is the output side of the pipeline: it receives MotionCommand
instances and is responsible for forwarding them to the actual motion system.

MockMotionBridge simply records every command it receives. It does NOT:

- Import unitree_sdk2_python.
- Open any serial port.
- Control any real robot.

It provides convenience properties for test assertions (move_count, stop_count,
moves, stops, last, etc.).
"""

from __future__ import annotations

from typing import Optional, Union

from robot_follow.core.types import MotionCommand


class MockMotionBridge:
    """Recording-only motion bridge for simulation.

    Usage
    -----
    >>> bridge = MockMotionBridge()
    >>> bridge.dispatch(MotionCommand(kind="move", vx=0.1, vy=0.0, omega=0.0, reason="tracking"))
    >>> bridge.move_count
    1
    >>> bridge.stop_count
    0

    The runner typically calls ``dispatch(command)``. Direct ``move(command)`` and
    ``stop(reason)`` calls are also supported for unit tests.
    """

    def __init__(self) -> None:
        self._history: list[MotionCommand] = []

    # -----------------------------------------------------------------------
    # Core API (as specified in the plan)
    # -----------------------------------------------------------------------
    def move(self, command: MotionCommand) -> None:
        """Record a move command."""
        self._history.append(command)

    def stop(self, reason: Union[str, MotionCommand]) -> None:
        """Record a stop command.

        Parameters
        ----------
        reason
            Either a MotionCommand (preserved directly) or a string reason,
            which is wrapped into a stop MotionCommand with zero velocities.
        """
        if isinstance(reason, MotionCommand):
            self._history.append(reason)
        else:
            self._history.append(
                MotionCommand(kind="stop", vx=0.0, vy=0.0, omega=0.0, reason=reason)
            )

    def dispatch(self, command: MotionCommand) -> None:
        """Dispatch a MotionCommand to the appropriate handler.

        This is the entry point used by `run_simulation`.
        """
        if command.kind == "move":
            self.move(command)
        else:
            # Preserve the original stop command for accurate recording.
            self.stop(command)

    # -----------------------------------------------------------------------
    # MotionBridge protocol (no-op for simulation)
    # -----------------------------------------------------------------------
    @property
    def sdk_error(self) -> bool:
        """Mock bridges never report SDK errors."""
        return False

    def init(self) -> None:
        """No-op: a mock bridge needs no initialization."""
        return None

    def tick(self, now: float | None = None) -> None:
        """No-op: a mock bridge has no watchdog."""
        return None

    def close(self) -> None:
        """No-op: a mock bridge has nothing to clean up (history preserved)."""
        return None

    # -----------------------------------------------------------------------
    # Convenience properties for tests
    # -----------------------------------------------------------------------
    @property
    def history(self) -> list[MotionCommand]:
        """Full chronological record of all dispatched commands."""
        return list(self._history)

    @property
    def moves(self) -> list[MotionCommand]:
        """All move commands."""
        return [c for c in self._history if c.kind == "move"]

    @property
    def stops(self) -> list[MotionCommand]:
        """All stop commands."""
        return [c for c in self._history if c.kind == "stop"]

    @property
    def move_count(self) -> int:
        """Number of move commands."""
        return sum(1 for c in self._history if c.kind == "move")

    @property
    def stop_count(self) -> int:
        """Number of stop commands."""
        return sum(1 for c in self._history if c.kind == "stop")

    @property
    def last(self) -> Optional[MotionCommand]:
        """Most recent command, or None if history is empty."""
        return self._history[-1] if self._history else None

    def reset(self) -> None:
        """Clear the recorded history."""
        self._history.clear()