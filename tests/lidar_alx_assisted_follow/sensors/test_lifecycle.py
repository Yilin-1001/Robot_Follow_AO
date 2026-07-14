"""Tests for wristband-driven lifecycle (Task 2 Step 4)."""

from __future__ import annotations

import time

import pytest

from robot_follow.lidar_alx_assisted_follow.sensors.lifecycle import (
    LifecycleState,
    WristbandLifecycleGuardian,
)


class TestWristbandLifecycleTransitions:
    """Test lifecycle state transitions."""

    def test_wristband_off_is_initial_state(self) -> None:
        """Guardian starts in WRISTBAND_OFF state."""
        guardian = WristbandLifecycleGuardian(ready_timeout_s=10.0)
        assert guardian.state == LifecycleState.WRISTBAND_OFF
        assert guardian.session_tag_id is None

    def test_wristband_on_starts_sensors(self) -> None:
        """Wristband ON with valid tag enters STARTING_SENSORS."""
        guardian = WristbandLifecycleGuardian(ready_timeout_s=10.0)

        state, should_stop, reason = guardian.tick(
            wristband_on=True,
            tag_id=100,
            serial_ok=True,
            cloud_received=False,
            now=0.0,
        )

        assert state == LifecycleState.STARTING_SENSORS
        assert not should_stop
        assert reason == "wristband_on"
        assert guardian.session_tag_id == 100

    def test_starting_sensors_becomes_ready_on_first_cloud(self) -> None:
        """First cloud in STARTING_SENSORS transitions to SENSORS_READY."""
        guardian = WristbandLifecycleGuardian(ready_timeout_s=10.0)

        # Wristband ON
        guardian.tick(True, 100, True, False, now=0.0)

        # First cloud received
        state, should_stop, reason = guardian.tick(
            wristband_on=True,
            tag_id=100,
            serial_ok=True,
            cloud_received=True,
            now=1.0,
        )

        assert state == LifecycleState.SENSORS_READY
        assert not should_stop
        assert reason == "sensors_ready"

    def test_ready_timeout_enters_fault(self) -> None:
        """Timeout in STARTING_SENSORS enters FAULT."""
        guardian = WristbandLifecycleGuardian(ready_timeout_s=2.0)

        # Wristband ON
        guardian.tick(True, 100, True, False, now=0.0)

        # No cloud, timeout
        state, should_stop, reason = guardian.tick(
            wristband_on=True,
            tag_id=100,
            serial_ok=True,
            cloud_received=False,
            now=3.0,  # > 2.0s timeout
        )

        assert state == LifecycleState.FAULT
        assert should_stop
        assert reason == "ready_timeout"
        assert guardian.session_tag_id is None

    def test_wristband_off_in_starting_stops_services(self) -> None:
        """Wristband OFF in STARTING_SENSORS returns to WRISTBAND_OFF."""
        guardian = WristbandLifecycleGuardian(ready_timeout_s=10.0)

        # Wristband ON
        guardian.tick(True, 100, True, False, now=0.0)

        # Wristband OFF
        state, should_stop, reason = guardian.tick(
            wristband_on=False,
            tag_id=None,
            serial_ok=True,
            cloud_received=False,
            now=1.0,
        )

        assert state == LifecycleState.WRISTBAND_OFF
        assert should_stop
        assert reason == "wristband_off_or_serial_error"
        assert guardian.session_tag_id is None

    def test_serial_error_in_ready_stops_services(self) -> None:
        """Serial error in SENSORS_READY returns to WRISTBAND_OFF."""
        guardian = WristbandLifecycleGuardian(ready_timeout_s=10.0)

        # Transition to READY
        guardian.tick(True, 100, True, False, now=0.0)
        guardian.tick(True, 100, True, True, now=1.0)

        # Serial error
        state, should_stop, reason = guardian.tick(
            wristband_on=True,
            tag_id=100,
            serial_ok=False,  # Serial error
            cloud_received=False,
            now=2.0,
        )

        assert state == LifecycleState.WRISTBAND_OFF
        assert should_stop
        assert reason == "wristband_off_or_serial_error"
        assert guardian.session_tag_id is None

    def test_tag_change_in_ready_unbinds(self) -> None:
        """Tag ID change in SENSORS_READY is not allowed."""
        guardian = WristbandLifecycleGuardian(ready_timeout_s=10.0)

        # Transition to READY with tag 100
        guardian.tick(True, 100, True, False, now=0.0)
        guardian.tick(True, 100, True, True, now=1.0)

        # Different tag appears
        state, should_stop, reason = guardian.tick(
            wristband_on=True,
            tag_id=999,  # Different tag
            serial_ok=True,
            cloud_received=False,
            now=2.0,
        )

        assert state == LifecycleState.WRISTBAND_OFF
        assert should_stop
        assert reason == "tag_changed"
        assert guardian.session_tag_id is None

    def test_fault_resets_when_wristband_off(self) -> None:
        """FAULT state resets to WRISTBAND_OFF when wristband goes off."""
        guardian = WristbandLifecycleGuardian(ready_timeout_s=2.0)

        # Enter FAULT
        guardian.tick(True, 100, True, False, now=0.0)
        guardian.tick(True, 100, True, False, now=3.0)
        assert guardian.state == LifecycleState.FAULT

        # Wristband OFF
        state, should_stop, reason = guardian.tick(
            wristband_on=False,
            tag_id=None,
            serial_ok=True,
            cloud_received=False,
            now=4.0,
        )

        assert state == LifecycleState.WRISTBAND_OFF
        assert should_stop
        assert reason == "fault_reset"

    def test_no_transition_without_valid_tag(self) -> None:
        """Wristband ON with no tag_id stays in WRISTBAND_OFF."""
        guardian = WristbandLifecycleGuardian(ready_timeout_s=10.0)

        state, should_stop, reason = guardian.tick(
            wristband_on=True,
            tag_id=None,  # No valid tag
            serial_ok=True,
            cloud_received=False,
            now=0.0,
        )

        assert state == LifecycleState.WRISTBAND_OFF
        assert not should_stop
        assert reason == ""