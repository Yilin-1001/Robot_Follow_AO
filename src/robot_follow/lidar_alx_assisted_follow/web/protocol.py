"""Web protocol validation for LiDAR + ALX Assisted Follow."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Optional


class MessageType(str, Enum):
    """Client message types."""

    SELECT_TARGET = "select_target"
    START_FOLLOW = "start_follow"
    STOP_FOLLOW = "stop_follow"


@dataclass
class ClientMessage:
    """Parsed client message."""

    type: MessageType
    x: Optional[float] = None
    y: Optional[float] = None


def parse_client_message(raw: str, max_bytes: int) -> ClientMessage:
    """Parse and validate a client message.

    Parameters
    ----------
    raw : str
        Raw JSON message string.
    max_bytes : int
        Maximum allowed message size in bytes.

    Returns
    -------
    ClientMessage
        Parsed message.

    Raises
    ------
    ValueError
        If message is invalid (too large, bad JSON, wrong schema, etc.)
    """
    # Check size
    if len(raw.encode("utf-8")) > max_bytes:
        raise ValueError("message_too_large: message exceeds max_bytes")

    # Parse JSON
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as e:
        raise ValueError(f"invalid_json: {e}") from e

    # Validate structure
    if not isinstance(data, dict):
        raise ValueError("invalid_schema: message must be a JSON object")

    if "type" not in data:
        raise ValueError("invalid_schema: missing 'type' key")

    msg_type = data.get("type")
    if not isinstance(msg_type, str):
        raise ValueError("invalid_schema: 'type' must be a string")

    # Validate per-type schema
    if msg_type == MessageType.SELECT_TARGET.value:
        return _parse_select_target(data)
    elif msg_type == MessageType.START_FOLLOW.value:
        return _parse_start_follow(data)
    elif msg_type == MessageType.STOP_FOLLOW.value:
        return _parse_stop_follow(data)
    else:
        raise ValueError(f"invalid_schema: unknown type '{msg_type}'")


def _parse_select_target(data: Dict[str, Any]) -> ClientMessage:
    """Parse select_target message.

    Expected schema: {"type":"select_target","x":<float>,"y":<float>}
    No extra keys allowed.
    """
    expected_keys = {"type", "x", "y"}
    actual_keys = set(data.keys())

    if actual_keys != expected_keys:
        raise ValueError(
            f"invalid_schema: select_target must have exactly keys {expected_keys}, "
            f"got {actual_keys}"
        )

    x = data.get("x")
    y = data.get("y")

    if not isinstance(x, (int, float)):
        raise ValueError("invalid_schema: 'x' must be a number")
    if not isinstance(y, (int, float)):
        raise ValueError("invalid_schema: 'y' must be a number")

    # Validate coordinates
    validate_select_target(x=float(x), y=float(y))

    return ClientMessage(type=MessageType.SELECT_TARGET, x=float(x), y=float(y))


def _parse_start_follow(data: Dict[str, Any]) -> ClientMessage:
    """Parse start_follow message.

    Expected schema: {"type":"start_follow"}
    No extra keys allowed.
    """
    expected_keys = {"type"}
    actual_keys = set(data.keys())

    if actual_keys != expected_keys:
        raise ValueError(
            f"invalid_schema: start_follow must have exactly keys {expected_keys}, "
            f"got {actual_keys}"
        )

    return ClientMessage(type=MessageType.START_FOLLOW)


def _parse_stop_follow(data: Dict[str, Any]) -> ClientMessage:
    """Parse stop_follow message.

    Expected schema: {"type":"stop_follow"}
    No extra keys allowed.
    """
    expected_keys = {"type"}
    actual_keys = set(data.keys())

    if actual_keys != expected_keys:
        raise ValueError(
            f"invalid_schema: stop_follow must have exactly keys {expected_keys}, "
            f"got {actual_keys}"
        )

    return ClientMessage(type=MessageType.STOP_FOLLOW)


def validate_select_target(x: float, y: float) -> None:
    """Validate select_target coordinates.

    Parameters
    ----------
    x : float
        X coordinate in meters.
    y : float
        Y coordinate in meters.

    Raises
    ------
    ValueError
        If coordinates are invalid.
    """
    # Check finite
    if not math.isfinite(x):
        raise ValueError(f"x must be finite, got {x}")
    if not math.isfinite(y):
        raise ValueError(f"y must be finite, got {y}")

    # Check range (±10 meters is reasonable for this prototype)
    if not (-10.0 <= x <= 10.0):
        raise ValueError(f"x out of range [-10, 10]: {x}")
    if not (-10.0 <= y <= 10.0):
        raise ValueError(f"y out of range [-10, 10]: {y}")


def validate_start_follow(payload: Dict[str, Any]) -> None:
    """Validate start_follow payload (no payload expected)."""
    pass


def validate_stop_follow(payload: Dict[str, Any]) -> None:
    """Validate stop_follow payload (no payload expected)."""
    pass
