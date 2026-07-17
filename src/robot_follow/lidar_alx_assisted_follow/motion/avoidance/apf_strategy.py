"""Adapter exposing the legacy APF algorithm as an avoidance mode."""

from __future__ import annotations

from dataclasses import dataclass

from robot_follow.lidar_alx_assisted_follow.config.schema import (
    APFConfig,
    OutputMotionConfig,
)
from robot_follow.lidar_alx_assisted_follow.motion.apf import compute_apf

from .types import AvoidanceInput, AvoidanceResult


@dataclass(frozen=True)
class APFAvoidanceStrategy:
    """Current continuous APF implementation behind the generic interface."""

    apf_config: APFConfig
    output_config: OutputMotionConfig
    mode: str = "apf"

    def compute(self, input_data: AvoidanceInput) -> AvoidanceResult:
        return compute_apf(input_data, self.apf_config, self.output_config)
