"""Tests for Web protocol validation."""

from __future__ import annotations

import pytest

from robot_follow.lidar_alx_assisted_follow.web.protocol import (
    ClientMessage,
    MessageType,
    parse_client_message,
    validate_select_target,
    validate_start_follow,
    validate_stop_follow,
)


class TestParseClientMessage:
    """Tests for parse_client_message."""

    def test_parse_valid_select_target(self) -> None:
        """Valid select_target message parses correctly."""
        raw = '{"type":"select_target","x":1.5,"y":-0.3}'
        msg = parse_client_message(raw, max_bytes=4096)
        assert msg.type == MessageType.SELECT_TARGET
        assert msg.x == 1.5
        assert msg.y == -0.3

    def test_parse_valid_start_follow(self) -> None:
        """Valid start_follow message parses correctly."""
        raw = '{"type":"start_follow"}'
        msg = parse_client_message(raw, max_bytes=4096)
        assert msg.type == MessageType.START_FOLLOW

    def test_parse_valid_stop_follow(self) -> None:
        """Valid stop_follow message parses correctly."""
        raw = '{"type":"stop_follow"}'
        msg = parse_client_message(raw, max_bytes=4096)
        assert msg.type == MessageType.STOP_FOLLOW

    def test_reject_invalid_json(self) -> None:
        """Invalid JSON raises ValueError."""
        with pytest.raises(ValueError, match="invalid_json"):
            parse_client_message("{not json}", max_bytes=4096)

    def test_reject_message_too_large(self) -> None:
        """Message exceeding max_bytes raises ValueError."""
        raw = '{"type":"select_target","x":1.0,"y":2.0}'  # 37 bytes
        with pytest.raises(ValueError, match="message_too_large"):
            parse_client_message(raw, max_bytes=10)

    def test_reject_missing_type(self) -> None:
        """Missing type key raises ValueError."""
        raw = '{"x":1.0,"y":2.0}'
        with pytest.raises(ValueError, match="invalid_schema"):
            parse_client_message(raw, max_bytes=4096)

    def test_reject_unknown_type(self) -> None:
        """Unknown type raises ValueError."""
        raw = '{"type":"unknown_command"}'
        with pytest.raises(ValueError, match="invalid_schema"):
            parse_client_message(raw, max_bytes=4096)

    def test_reject_extra_keys_in_start_follow(self) -> None:
        """Extra keys in start_follow raises ValueError."""
        raw = '{"type":"start_follow","extra":1}'
        with pytest.raises(ValueError, match="invalid_schema"):
            parse_client_message(raw, max_bytes=4096)

    def test_reject_missing_x_in_select_target(self) -> None:
        """Missing x in select_target raises ValueError."""
        raw = '{"type":"select_target","y":0.5}'
        with pytest.raises(ValueError, match="invalid_schema"):
            parse_client_message(raw, max_bytes=4096)

    def test_reject_non_finite_x(self) -> None:
        """Non-finite x in select_target raises ValueError."""
        raw = '{"type":"select_target","x":Infinity,"y":0.5}'
        with pytest.raises(ValueError, match="x must be finite"):
            parse_client_message(raw, max_bytes=4096)

    def test_reject_out_of_range_click(self) -> None:
        """Click outside valid range raises ValueError."""
        raw = '{"type":"select_target","x":15.0,"y":0.5}'
        with pytest.raises(ValueError, match="x out of range"):
            parse_client_message(raw, max_bytes=4096)


class TestValidateSelectTarget:
    """Tests for validate_select_target."""

    def test_valid_coordinates(self) -> None:
        """Valid coordinates pass validation."""
        validate_select_target(x=1.5, y=-0.3)

    def test_reject_nan_x(self) -> None:
        """NaN x raises ValueError."""
        with pytest.raises(ValueError, match="x must be finite"):
            import math
            validate_select_target(x=math.nan, y=0.0)

    def test_reject_inf_y(self) -> None:
        """Inf y raises ValueError."""
        with pytest.raises(ValueError, match="y must be finite"):
            import math
            validate_select_target(x=0.0, y=math.inf)

    def test_reject_x_too_large(self) -> None:
        """x > 10.0 raises ValueError."""
        with pytest.raises(ValueError, match="x.*out of range"):
            validate_select_target(x=10.5, y=0.0)

    def test_reject_x_too_small(self) -> None:
        """x < -10.0 raises ValueError."""
        with pytest.raises(ValueError, match="x.*out of range"):
            validate_select_target(x=-10.5, y=0.0)

    def test_reject_y_too_large(self) -> None:
        """y > 10.0 raises ValueError."""
        with pytest.raises(ValueError, match="y.*out of range"):
            validate_select_target(x=0.0, y=10.5)

    def test_reject_y_too_small(self) -> None:
        """y < -10.0 raises ValueError."""
        with pytest.raises(ValueError, match="y.*out of range"):
            validate_select_target(x=0.0, y=-10.5)


class TestValidateStartFollow:
    """Tests for validate_start_follow."""

    def test_accepts_no_payload(self) -> None:
        """start_follow accepts no payload."""
        validate_start_follow(payload={})


class TestValidateStopFollow:
    """Tests for validate_stop_follow."""

    def test_accepts_no_payload(self) -> None:
        """stop_follow accepts no payload."""
        validate_stop_follow(payload={})


class TestClientMessage:
    """Tests for ClientMessage dataclass."""

    def test_select_target_has_coordinates(self) -> None:
        """SELECT_TARGET message has x and y."""
        msg = ClientMessage(type=MessageType.SELECT_TARGET, x=1.5, y=-0.3)
        assert msg.x == 1.5
        assert msg.y == -0.3

    def test_start_follow_has_no_coordinates(self) -> None:
        """START_FOLLOW message has None for x and y."""
        msg = ClientMessage(type=MessageType.START_FOLLOW)
        assert msg.x is None
        assert msg.y is None
