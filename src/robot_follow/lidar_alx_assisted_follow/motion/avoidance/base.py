"""Protocol implemented by each local avoidance mode."""

from __future__ import annotations

from typing import Protocol

from .types import AvoidanceInput, AvoidanceResult


class AvoidanceStrategy(Protocol):
    """Compute a safe local follow request for one fresh cloud."""

    mode: str

    def compute(self, input_data: AvoidanceInput) -> AvoidanceResult:
        """Return the mode's motion request for ``input_data``."""
        ...
