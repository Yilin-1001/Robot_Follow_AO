from __future__ import annotations

import json

import numpy as np

from robot_follow.ao.core.types import (
    AvoidanceDecision,
    DecisionKind,
    Direction,
    MotionCommand,
    ObstacleReport,
    PointCloudFrame,
    ZoneResult,
)
from robot_follow.ao.web.serialization import (
    point_cloud_to_json_points,
    visualization_payload,
    zone_to_dict,
    zone_bounds_to_dict,
)
from robot_follow.ao.avoidance.zones import default_zone_bounds


def _zone(name, *, count=0, distance=None, occupied=False, blocked=False):
    return ZoneResult(
        name=name,
        point_count=count,
        min_x_m=distance,
        min_distance_m=distance,
        occupied=occupied,
        blocked=blocked,
    )


def test_point_cloud_to_json_points_limits_and_rounds_xyz():
    frame = PointCloudFrame(
        timestamp=1.0,
        frame_id="body",
        points_xyz=np.array(
            [
                [0.123456, 0.0, 0.2],
                [0.4, -0.5, 0.6],
                [0.7, 0.8, 0.9],
            ],
            dtype=np.float64,
        ),
        valid=True,
        empty_is_clear=False,
    )

    assert point_cloud_to_json_points(frame, max_points=2) == [
        [0.123, 0.0, 0.2],
        [0.4, -0.5, 0.6],
    ]


def test_zone_to_dict_preserves_decision_fields():
    data = zone_to_dict(_zone("front", count=12, distance=0.35, occupied=True, blocked=True))
    assert data == {
        "name": "front",
        "point_count": 12,
        "min_distance_m": 0.35,
        "min_x_m": 0.35,
        "occupied": True,
        "blocked": True,
    }


def test_zone_bounds_to_dict_serializes_boxes_for_frontend():
    data = zone_bounds_to_dict(default_zone_bounds())

    assert data["front"]["x_max"] == 1.0
    assert data["front"]["y_min"] == -0.35


def test_visualization_payload_is_json_serializable():
    frame = PointCloudFrame(
        timestamp=2.0,
        frame_id="body",
        points_xyz=np.array([[0.3, 0.1, 0.2]], dtype=np.float64),
        valid=True,
        empty_is_clear=False,
    )
    report = ObstacleReport(
        timestamp=2.0,
        valid=True,
        front=_zone("front", count=10, distance=0.3, occupied=True, blocked=True),
        left=_zone("left"),
        right=_zone("right"),
        rear=_zone("rear"),
        rotation=_zone("rotation", count=10, distance=0.3, occupied=True, blocked=True),
        emergency_stop=True,
    )
    desired = MotionCommand.move(2.0, source="scripted", vx=0.18, reason="forward")
    decision = AvoidanceDecision(
        timestamp=2.0,
        safe_command=MotionCommand.move(
            2.0,
            source="avoidance",
            vx=0.03,
            vy=0.08,
            reason="avoid_left",
        ),
        decision_kind=DecisionKind.AVOID_LEFT,
        recommended_direction=Direction.LEFT,
        triggered_safety_rules=("front_blocked",),
        reason="avoid_left",
    )

    payload = visualization_payload(
        raw_frame=frame,
        filtered_frame=frame,
        report=report,
        desired_command=desired,
        decision=decision,
        raw_count=1,
        filtered_count=1,
        now_s=2.05,
        max_points=10,
        zone_bounds=default_zone_bounds(),
    )

    json.dumps(payload)
    assert payload["frame_id"] == "body"
    assert payload["radar_age_s"] == 0.05
    assert payload["points"] == [[0.3, 0.1, 0.2]]
    assert payload["zones"]["front"]["blocked"] is True
    assert payload["zone_bounds"]["front"]["x_max"] == 1.0
    assert payload["decision"]["kind"] == "avoid_left"
    assert payload["decision"]["safe_command"]["vy"] == 0.08
