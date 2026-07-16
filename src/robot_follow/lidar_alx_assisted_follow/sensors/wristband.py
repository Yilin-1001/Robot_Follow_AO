"""ROS-independent wristband control-state contract.

The ALX serial parser is the authority for this state.  ROS only transports
the value from the sole serial owner to the assisted-follow runtime.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


class WristbandTopicState(IntEnum):
    """Wire values used on ``std_msgs/UInt8`` wristband topic."""

    UNKNOWN = 0
    ON = 1
    OFF = 2
    SERIAL_FAULT = 3


@dataclass(frozen=True)
class WristbandStatus:
    """Most recently received wristband control status."""

    state: WristbandTopicState
    received_at: Optional[float]

    @property
    def enabled(self) -> Optional[bool]:
        """Return the explicit ON/OFF value, or None when unavailable."""
        if self.state == WristbandTopicState.ON:
            return True
        if self.state == WristbandTopicState.OFF:
            return False
        return None
