"""Construction of supported local avoidance modes."""

from __future__ import annotations

from robot_follow.lidar_alx_assisted_follow.config.schema import (
    APFConfig,
    AvoidanceConfig,
    OutputMotionConfig,
)

from .apf_strategy import APFAvoidanceStrategy
from .base import AvoidanceStrategy


def build_avoidance_strategy(
    config: AvoidanceConfig,
    apf_config: APFConfig,
    output_config: OutputMotionConfig,
) -> AvoidanceStrategy:
    """Build the configured supported strategy.

    ``grid_astar`` is intentionally not accepted yet: enabling an unimplemented
    planner on a moving robot must fail configuration validation, rather than
    silently falling back to APF.
    """
    if config.mode == "apf":
        return APFAvoidanceStrategy(apf_config, output_config)
    raise AssertionError(f"validated avoidance mode is unsupported: {config.mode}")
