from __future__ import annotations

from robot_follow.ao.web.sim_viewer import moving_front_obstacle_frames, parse_args


def test_moving_front_obstacle_frames_produces_20hz_timestamps():
    frames = list(moving_front_obstacle_frames(start_s=1.0, count=3, hz=20.0))

    assert [frame.timestamp for frame in frames] == [1.0, 1.05, 1.1]
    assert frames[0].frame_id == "body"
    assert frames[0].points_xyz.shape[1] == 3
    assert 1.0 < frames[0].points_xyz[:, 0].min()


def test_sim_viewer_defaults_for_web_simulation():
    args = parse_args([])

    assert args.host == "0.0.0.0"
    assert args.port == 8080
    assert args.hz == 20.0
