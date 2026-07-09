from __future__ import annotations

import json


CLIENT_MESSAGE_TYPES = {"select_target", "toggle_target_lock", "confirm_target", "cancel_target"}


def frame_message(payload: dict) -> dict:
    return {"type": "frame", "payload": dict(payload)}


def status_message(message: str, *, level: str = "info") -> dict:
    return {"type": "status", "payload": {"level": level, "message": message}}


def parse_client_message(text: str) -> dict:
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError("client message must be valid JSON") from exc
    if not isinstance(data, dict):
        raise ValueError("client message must be an object")
    msg_type = data.get("type")
    if msg_type not in CLIENT_MESSAGE_TYPES:
        raise ValueError(f"unsupported client message type: {msg_type!r}")
    if msg_type in {"select_target", "toggle_target_lock"}:
        return {
            "type": msg_type,
            "x": float(data["x"]),
            "y": float(data["y"]),
        }
    if msg_type == "confirm_target":
        return {"type": "confirm_target"}
    return {"type": "cancel_target"}
