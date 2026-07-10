from __future__ import annotations

import numpy as np

from robot_follow.ao.app.config import AppConfig
from robot_follow.ao.avoidance.planner import StatefulAvoidancePlanner
from robot_follow.ao.core.types import PointCloudFrame
from robot_follow.ao.web.frames import ConstantCommandSource, build_visualization_payload


def test_build_visualization_payload_uses_filtered_points_and_decision():
    cfg = AppConfig.default()
    frame = PointCloudFrame(
        timestamp=10.0,
        frame_id="body",
        points_xyz=np.array(
            [[0.30, 0.0, -0.2], [0.35, 0.0, -0.2], [4.0, 0.0, -0.2], [0.3, 0.0, -2.5]],
            dtype=np.float64,
        ),
        valid=True,
        empty_is_clear=False,
        reason="fast_lio_body",
    )
    source = ConstantCommandSource(source="scripted", vx=0.18)

    payload = build_visualization_payload(
        frame,
        command_source=source,
        config=cfg,
        now_s=10.1,
        min_points=2,
        max_points=100,
    )

    assert payload["raw_count"] == 4
    assert payload["filtered_count"] == 2
    assert payload["points"] == [[0.3, 0.0, 0.2], [0.35, 0.0, 0.2]]
    assert payload["zones"]["front"]["blocked"] is True
    assert payload["decision"]["kind"] == "avoid_left"
    assert payload["decision"]["safe_command"]["vx"] == cfg.avoidance.forward_vx_when_avoiding_mps
    assert payload["decision"]["safe_command"]["vy"] == cfg.avoidance.lateral_vy_mps


def test_build_visualization_payload_stops_on_stale_frame():
    cfg = AppConfig.default()
    frame = PointCloudFrame(
        timestamp=1.0,
        frame_id="body",
        points_xyz=np.array([[0.30, 0.0, -0.2]], dtype=np.float64),
        valid=True,
        empty_is_clear=False,
        reason="fast_lio_body",
    )
    source = ConstantCommandSource(source="scripted", vx=0.18)

    payload = build_visualization_payload(
        frame,
        command_source=source,
        config=cfg,
        now_s=2.0,
        min_points=1,
        max_points=100,
    )

    assert payload["radar_valid"] is False
    assert payload["decision"]["kind"] == "stop"
    assert payload["decision"]["reason"] == "pointcloud_timeout"


def test_live_frame_uses_monotonic_receive_time_for_staleness_and_clears_points():
    cfg = AppConfig.default()
    frame = PointCloudFrame(
        timestamp=1_800_000_000.0,
        frame_id="body",
        points_xyz=np.array([[0.30, 0.0, -0.2]], dtype=np.float64),
        valid=True,
        empty_is_clear=False,
        reason="fast_lio_body",
        received_at_s=10.0,
    )

    payload = build_visualization_payload(
        frame,
        command_source=ConstantCommandSource(),
        config=cfg,
        now_s=10.0 + cfg.radar.timeout_s + 0.01,
        min_points=1,
    )

    assert payload["radar_valid"] is False
    assert payload["radar_age_s"] == cfg.radar.timeout_s + 0.01
    assert payload["points"] == []
    assert payload["decision"]["reason"] == "pointcloud_timeout"


def test_build_visualization_payload_can_simulate_sdk_stateful_early_avoidance():
    cfg = AppConfig.default()
    planner = StatefulAvoidancePlanner(cfg)
    frame = PointCloudFrame(
        timestamp=10.0,
        frame_id="body",
        points_xyz=np.column_stack(
                (
                    np.full(12, 0.85),
                    np.linspace(-0.3, 0.3, 12),
                    np.full(12, -0.25),
                )
            ),
        valid=True,
        empty_is_clear=False,
        reason="sim_sdk",
    )
    source = ConstantCommandSource(source="scripted", vx=0.18)

    payload = build_visualization_payload(
        frame,
        command_source=source,
        config=cfg,
        planner=planner,
        now_s=10.0,
        min_points=2,
        max_points=100,
    )

    assert payload["zone_bounds"]["front"]["x_max"] == cfg.zones.bounds["front"].x_max
    assert payload["zones"]["front"]["occupied"] is True
    assert payload["zones"]["front"]["blocked"] is False
    assert payload["decision"]["kind"] in {"avoid_left", "avoid_right"}
    assert abs(payload["decision"]["safe_command"]["vy"]) == cfg.avoidance.lateral_vy_mps
