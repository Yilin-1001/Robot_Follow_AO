"""Wristband-driven sensor lifecycle management."""

from __future__ import annotations

import time
from enum import Enum
from typing import Optional


class LifecycleState(str, Enum):
    """Sensor lifecycle states."""

    WRISTBAND_OFF = "wristband_off"
    STARTING_SENSORS = "starting_sensors"
    SENSORS_READY = "sensors_ready"
    FAULT = "fault"


class WristbandLifecycleGuardian:
    """Manages sensor lifecycle based on wristband state.

    States:
    - WRISTBAND_OFF: Wristband is off, sensors stopped
    - STARTING_SENSORS: Wristband turned on, starting Livox/FAST-LIO
    - SENSORS_READY: Sensors running and first cloud received
    - FAULT: Sensor startup timeout or critical error
    """

    def __init__(self, ready_timeout_s: float) -> None:
        self._ready_timeout_s = ready_timeout_s
        self._state = LifecycleState.WRISTBAND_OFF
        self._start_time: Optional[float] = None
        self._session_tag_id: Optional[int] = None

    @property
    def state(self) -> LifecycleState:
        """Current lifecycle state."""
        return self._state

    @property
    def session_tag_id(self) -> Optional[int]:
        """Current session tag ID bound from wristband, or None."""
        return self._session_tag_id

    def tick(
        self,
        wristband_on: bool,
        tag_id: Optional[int],
        serial_ok: bool,
        cloud_received: bool,
        now: Optional[float] = None,
    ) -> tuple[LifecycleState, bool, str]:
        """Process one lifecycle tick.

        Parameters
        ----------
        wristband_on : bool
            Whether wristband is currently ON.
        tag_id : int or None
            Current tag ID from ALX, or None if invalid/missing.
        serial_ok : bool
            Whether serial communication is healthy.
        cloud_received : bool
            Whether a valid cloud was received since last tick.
        now : float or None
            Current monotonic time, or None to use time.monotonic().

        Returns
        -------
        tuple[LifecycleState, bool, str]
            (new_state, should_stop_services, transition_reason)
            should_stop_services indicates whether to stop Livox/FAST-LIO.
        """
        if now is None:
            now = time.monotonic()

        old_state = self._state

        # WRISTBAND_OFF: Wait for wristband ON
        if self._state == LifecycleState.WRISTBAND_OFF:
            if wristband_on and serial_ok and tag_id is not None:
                # Bind tag and start sensors
                self._session_tag_id = tag_id
                self._state = LifecycleState.STARTING_SENSORS
                self._start_time = now
                return (self._state, False, "wristband_on")
            return (self._state, False, "")

        # STARTING_SENSORS: Wait for first cloud or timeout
        if self._state == LifecycleState.STARTING_SENSORS:
            # Check for wristband OFF or serial error
            if not wristband_on or not serial_ok:
                self._state = LifecycleState.WRISTBAND_OFF
                self._session_tag_id = None
                return (self._state, True, "wristband_off_or_serial_error")

            # Check for timeout
            if self._start_time is not None:
                elapsed = now - self._start_time
                if elapsed > self._ready_timeout_s:
                    self._state = LifecycleState.FAULT
                    self._session_tag_id = None
                    return (self._state, True, "ready_timeout")

            # Check for first cloud
            if cloud_received:
                self._state = LifecycleState.SENSORS_READY
                return (self._state, False, "sensors_ready")

            return (self._state, False, "")

        # SENSORS_READY: Normal operation
        if self._state == LifecycleState.SENSORS_READY:
            # Check for wristband OFF or serial error
            if not wristband_on or not serial_ok:
                self._state = LifecycleState.WRISTBAND_OFF
                self._session_tag_id = None
                return (self._state, True, "wristband_off_or_serial_error")

            # Tag change is not allowed in this prototype
            if tag_id is not None and tag_id != self._session_tag_id:
                self._state = LifecycleState.WRISTBAND_OFF
                self._session_tag_id = None
                return (self._state, True, "tag_changed")

            return (self._state, False, "")

        # FAULT: Requires restart
        if self._state == LifecycleState.FAULT:
            if not wristband_on:
                # Reset to WRISTBAND_OFF when wristband goes off
                self._state = LifecycleState.WRISTBAND_OFF
                self._session_tag_id = None
                return (self._state, True, "fault_reset")

            return (self._state, True, "fault_active")

        # Should not reach here
        return (self._state, False, "")

    def reset(self) -> None:
        """Reset lifecycle to WRISTBAND_OFF."""
        self._state = LifecycleState.WRISTBAND_OFF
        self._session_tag_id = None
        self._start_time = None