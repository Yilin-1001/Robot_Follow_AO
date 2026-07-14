"""Runtime state management for LiDAR + ALX Assisted Follow."""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Deque, Dict, List, Optional, Tuple

from robot_follow.lidar_alx_assisted_follow.domain.models import StopKind, StopSnapshot


class ControlEventKind(str, Enum):
    """Types of control events from Web clients."""

    SELECT_TARGET = "select_target"
    START_FOLLOW = "start_follow"
    STOP_FOLLOW = "stop_follow"
    CONTROLLER_DISCONNECT = "controller_disconnect"


@dataclass
class ControlEvent:
    """A control event from a Web client."""

    kind: ControlEventKind
    payload: Dict[str, Any]


class ControlEventInbox:
    """Bounded deque of control events with select_target coalescing.

    Holds up to ``capacity`` events (default 32).
    ``select_target`` events are coalesced: a new one replaces the
    most recent pending ``select_target``, keeping the queue compact.
    """

    def __init__(self, capacity: int = 32) -> None:
        self._capacity = capacity
        self._lock = threading.Lock()
        self._events: Deque[ControlEvent] = Deque()

    def push(self, event: ControlEvent) -> bool:
        """Push a control event into the inbox.

        ``select_target`` events are coalesced: the newest pending
        ``select_target`` is replaced in-place.  Other events always
        append.  If the queue is full, the oldest ``select_target``
        is removed to make room.

        Parameters
        ----------
        event : ControlEvent
            The event to push.

        Returns
        -------
        bool
            True if the event was accepted, False if the queue was
            full and no ``select_target`` could be evicted.
        """
        with self._lock:
            if event.kind == ControlEventKind.SELECT_TARGET:
                # Coalesce: replace the newest existing select_target
                for i in range(len(self._events) - 1, -1, -1):
                    if self._events[i].kind == ControlEventKind.SELECT_TARGET:
                        self._events[i] = event
                        return True

            # If full, try to evict an older select_target
            if len(self._events) >= self._capacity:
                for i in range(len(self._events)):
                    if self._events[i].kind == ControlEventKind.SELECT_TARGET:
                        del self._events[i]
                        break
                else:
                    return False

            self._events.append(event)
            return True

    def pop(self) -> Optional[ControlEvent]:
        """Pop the oldest event from the inbox.

        Returns
        -------
        ControlEvent or None
            The oldest event, or None if empty.
        """
        with self._lock:
            if not self._events:
                return None
            return self._events.popleft()

    def clear(self) -> None:
        """Remove all events from the inbox."""
        with self._lock:
            self._events.clear()

    @property
    def qsize(self) -> int:
        """Number of events currently in the inbox."""
        with self._lock:
            return len(self._events)

    @property
    def empty(self) -> bool:
        """Whether the inbox is empty."""
        with self._lock:
            return not self._events

    def snapshot(self) -> List[Tuple[ControlEventKind, Dict[str, Any]]]:
        """Get an immutable snapshot of all pending events.

        Returns
        -------
        list of (ControlEventKind, dict) tuples
            All events currently in the inbox, oldest first.
        """
        with self._lock:
            return [(e.kind, dict(e.payload)) for e in self._events]


class LatestStatus:
    """Thread-safe, immutable latest status container.

    Holds the single most recent status value.  ``update`` replaces
    the stored value atomically.  ``snapshot`` returns a copy.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._version = 0
        self._status: Dict[str, Any] = {}

    def update(self, status: Dict[str, Any]) -> int:
        """Replace the stored status and bump the version.

        Parameters
        ----------
        status : dict
            New status data (will be shallow-copied internally).

        Returns
        -------
        int
            New version number.
        """
        with self._lock:
            self._version += 1
            self._status = dict(status)
            return self._version

    def snapshot(self) -> Tuple[int, Dict[str, Any]]:
        """Get (version, status_copy) atomically.

        Returns
        -------
        tuple[int, dict]
            Version number and a shallow copy of the status dict.
        """
        with self._lock:
            return (self._version, dict(self._status))


class StopLatch:
    """Thread-safe stop request latch with HARD/SOFT priority.

    Only the control thread (specified at construction) can clear the latch.
    Any thread can request a stop.

    HARD requests dominate SOFT requests.
    """

    def __init__(self, control_thread_id: int) -> None:
        """Initialize latch with the ID of the control thread.

        Parameters
        ----------
        control_thread_id : int
            Thread ID that is allowed to clear the latch.
        """
        self._control_thread_id = control_thread_id
        self._lock = threading.Lock()
        self._requested = False
        self._kind: Optional[StopKind] = None
        self._reason: Optional[str] = None
        self._start_after_sequence = 0
        self._version = 0

    def snapshot(self) -> StopSnapshot:
        """Get immutable snapshot of current latch state."""
        with self._lock:
            return StopSnapshot(
                requested=self._requested,
                kind=self._kind,
                reason=self._reason,
                start_after_sequence=self._start_after_sequence,
                version=self._version,
            )

    def request(self, reason: str, kind: StopKind) -> None:
        """Request a stop from any thread.

        HARD requests cannot be overwritten by SOFT.
        Later HARD requests replace SOFT.

        Parameters
        ----------
        reason : str
            Human-readable stop reason.
        kind : StopKind
            HARD or SOFT priority.
        """
        with self._lock:
            # HARD dominates SOFT
            if self._requested and self._kind == StopKind.HARD and kind == StopKind.SOFT:
                return

            self._requested = True
            self._kind = kind
            self._reason = reason
            self._version += 1

    def clear_for_start(self, start_after_sequence: int) -> None:
        """Clear latch after a valid Start command.

        Can only be called from the control thread.

        Parameters
        ----------
        start_after_sequence : int
            Cloud sequence number after which motion is allowed.

        Raises
        ------
        RuntimeError
            If called from non-control thread.
        """
        if threading.get_ident() != self._control_thread_id:
            raise RuntimeError(
                f"StopLatch.clear_for_start can only be called from control thread "
                f"(expected {self._control_thread_id}, got {threading.get_ident()})"
            )

        with self._lock:
            self._requested = False
            self._kind = None
            self._reason = None
            self._start_after_sequence = start_after_sequence
            self._version += 1

    def bind_control_thread(self, control_thread_id: int) -> None:
        """Bind latch clearing ownership to the running control thread."""
        with self._lock:
            self._control_thread_id = control_thread_id

    def clear_soft_recovery(self) -> None:
        """Clear latch for soft-stop auto-recovery.

        Only callable from the control thread.
        Only clears SOFT stops (HARD requires explicit Start).

        Raises
        ------
        RuntimeError
            If called from non-control thread, or if current stop is HARD.
        """
        if threading.get_ident() != self._control_thread_id:
            raise RuntimeError(
                f"StopLatch.clear_soft_recovery can only be called from "
                f"control thread (expected {self._control_thread_id}, "
                f"got {threading.get_ident()})"
            )

        with self._lock:
            if self._kind == StopKind.HARD:
                raise RuntimeError(
                    "clear_soft_recovery cannot clear a HARD stop; "
                    "use clear_for_start"
                )
            if self._kind != StopKind.SOFT or self._reason not in (
                "confidence_low",
                "alx_stale",
            ):
                raise RuntimeError(
                    "clear_soft_recovery only accepts confidence_low or alx_stale"
                )
            self._requested = False
            self._kind = None
            self._reason = None
            self._version += 1

    def motion_allowed_for(self, sequence: int) -> bool:
        """Check if motion is allowed for a given cloud sequence.

        Parameters
        ----------
        sequence : int
            Cloud sequence number.

        Returns
        -------
        bool
            True if motion is allowed (not stopped and sequence > start_after_sequence).
        """
        with self._lock:
            if self._requested:
                return False
            return sequence > self._start_after_sequence
