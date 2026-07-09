from __future__ import annotations

import pytest

from robot_follow.ao.web.protocol import (
    frame_message,
    parse_client_message,
    status_message,
)


def test_frame_message_wraps_visualization_payload():
    payload = {"timestamp": 1.0, "points": [[0.3, 0.0, 0.2]]}

    msg = frame_message(payload)

    assert msg == {
        "type": "frame",
        "payload": {"timestamp": 1.0, "points": [[0.3, 0.0, 0.2]]},
    }


def test_status_message_has_text_and_level():
    assert status_message("waiting_for_frame", level="warn") == {
        "type": "status",
        "payload": {"level": "warn", "message": "waiting_for_frame"},
    }


def test_parse_client_message_accepts_future_target_selection():
    msg = parse_client_message('{"type":"select_target","x":1.2,"y":-0.3}')

    assert msg == {"type": "select_target", "x": 1.2, "y": -0.3}


def test_parse_client_message_accepts_target_lock_toggle():
    msg = parse_client_message('{"type":"toggle_target_lock","x":1.2,"y":-0.3}')

    assert msg == {"type": "toggle_target_lock", "x": 1.2, "y": -0.3}


def test_parse_client_message_rejects_unknown_type():
    with pytest.raises(ValueError, match="unsupported client message type"):
        parse_client_message('{"type":"direct_cmd","vx":1.0}')
