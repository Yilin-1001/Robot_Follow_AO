from __future__ import annotations

from typing import Optional

import numpy as np


def _round_optional(value: Optional[float], digits: int = 3):
    if value is None:
        return None
    return round(float(value), digits)


def point_cloud_to_json_points(frame, *, max_points: int = 2000) -> list:
    points = np.asarray(frame.points_xyz, dtype=np.float64)
    if points.size == 0:
        return []
    finite = points[np.isfinite(points).all(axis=1)]
    limited = finite[:max_points]
    return np.round(limited, 3).tolist()


def motion_command_to_dict(command) -> dict:
    return {
        "timestamp": float(command.timestamp),
        "kind": command.kind.value,
        "source": command.source,
        "vx": round(float(command.vx), 3),
        "vy": round(float(command.vy), 3),
        "omega": round(float(command.omega), 3),
        "reason": command.reason,
    }


def zone_to_dict(zone) -> dict:
    return {
        "name": zone.name,
        "point_count": int(zone.point_count),
        "min_distance_m": _round_optional(zone.min_distance_m),
        "min_x_m": _round_optional(zone.min_x_m),
        "occupied": bool(zone.occupied),
        "blocked": bool(zone.blocked),
    }


def zone_bounds_to_dict(bounds) -> dict:
    return {
        name: {
            "x_min": round(float(box.x_min), 3),
            "x_max": round(float(box.x_max), 3),
            "y_min": round(float(box.y_min), 3),
            "y_max": round(float(box.y_max), 3),
            "z_min": round(float(box.z_min), 3),
            "z_max": round(float(box.z_max), 3),
        }
        for name, box in bounds.items()
    }


def obstacle_report_to_dict(report) -> dict:
    return {
        "timestamp": float(report.timestamp),
        "valid": bool(report.valid),
        "emergency_stop": bool(report.emergency_stop),
        "reason": report.reason,
        "zones": {
            "front": zone_to_dict(report.front),
            "left": zone_to_dict(report.left),
            "right": zone_to_dict(report.right),
            "rear": zone_to_dict(report.rear),
            "rotation": zone_to_dict(report.rotation),
        },
    }


def avoidance_decision_to_dict(decision) -> dict:
    return {
        "timestamp": float(decision.timestamp),
        "kind": decision.decision_kind.value,
        "recommended_direction": decision.recommended_direction.value,
        "slowdown_ratio": round(float(decision.slowdown_ratio), 3),
        "rules": list(decision.triggered_safety_rules),
        "reason": decision.reason,
        "safe_command": motion_command_to_dict(decision.safe_command),
    }


def visualization_payload(
    *,
    raw_frame,
    filtered_frame,
    report,
    desired_command,
    decision,
    raw_count: int,
    filtered_count: int,
    now_s: float,
    max_points: int = 2000,
    zone_bounds=None,
) -> dict:
    report_data = obstacle_report_to_dict(report)
    age_reference = (
        raw_frame.timestamp
        if raw_frame.received_at_s is None
        else raw_frame.received_at_s
    )
    payload = {
        "timestamp": float(filtered_frame.timestamp),
        "frame_id": filtered_frame.frame_id,
        "radar_valid": bool(report.valid),
        "radar_age_s": round(max(0.0, float(now_s) - float(age_reference)), 3),
        "raw_count": int(raw_count),
        "filtered_count": int(filtered_count),
        "points": point_cloud_to_json_points(filtered_frame, max_points=max_points),
        "zones": report_data["zones"],
        "report": report_data,
        "desired_command": motion_command_to_dict(desired_command),
        "decision": avoidance_decision_to_dict(decision),
    }
    if zone_bounds is not None:
        payload["zone_bounds"] = zone_bounds_to_dict(zone_bounds)
    return payload
