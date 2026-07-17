"""Tests for selection of local obstacle-avoidance modes."""

from __future__ import annotations

import numpy as np

from robot_follow.lidar_alx_assisted_follow.config.schema import (
    APFConfig,
    AvoidanceConfig,
    OutputMotionConfig,
)
from robot_follow.lidar_alx_assisted_follow.domain.models import PreprocessedCloud
from robot_follow.lidar_alx_assisted_follow.motion.avoidance import (
    APFAvoidanceStrategy,
    AvoidanceInput,
    build_avoidance_strategy,
)


def test_factory_builds_apf_as_default_supported_mode() -> None:
    strategy = build_avoidance_strategy(
        AvoidanceConfig(), APFConfig(repulsion_gain=0.0), OutputMotionConfig()
    )

    assert isinstance(strategy, APFAvoidanceStrategy)
    assert strategy.mode == "apf"


def test_apf_mode_uses_shared_avoidance_contract() -> None:
    strategy = build_avoidance_strategy(
        AvoidanceConfig(), APFConfig(repulsion_gain=0.0), OutputMotionConfig()
    )
    points = np.array([[1.5, 0.0, 0.2]], dtype=np.float64)
    result = strategy.compute(
        AvoidanceInput(
            cloud=PreprocessedCloud(points, np.array([0], dtype=np.int64)),
            target_center_xy=(1.5, 0.0),
            target_mask=np.array([True]),
            sequence=1,
        )
    )

    assert result.stop_reason is None
    assert result.intent.reason == "follow"
