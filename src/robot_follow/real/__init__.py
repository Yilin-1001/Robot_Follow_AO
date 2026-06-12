"""Real hardware-facing adapters for the follow system.

This package contains integrations that may open serial ports, use robot SDKs,
or otherwise talk to physical devices. Core algorithms and offline simulation
must not depend on this package.
"""

from robot_follow.real.alx_source import ALXLiveSource, ALXSnapshot

__all__ = [
    "ALXLiveSource",
    "ALXSnapshot",
]
