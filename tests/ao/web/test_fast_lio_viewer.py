from __future__ import annotations

from robot_follow.ao.web.fast_lio_viewer import parse_args


def test_fast_lio_viewer_defaults_match_backpack_use():
    args = parse_args([])

    assert args.host == "0.0.0.0"
    assert args.port == 8080
    assert args.hz == 20.0
    assert args.max_points == 2000
    assert args.topic == "/cloud_registered_body"
    assert args.expected_frame_id == "body"
    assert args.allow_non_body_frame is False
