from robot_follow.ao.app.config import AppConfig


def test_default_app_config_uses_live_body_frame_roll_transform() -> None:
    config = AppConfig.default()

    assert config.transform.apply_static_transform is True
    assert config.transform.roll_deg == 180.0
    assert config.transform.pitch_deg == 0.0
    assert config.transform.yaw_deg == 0.0
    assert config.transform.target_frame == "base_link"
    assert config.radar.pointcloud_topic == "/cloud_registered_body"
    assert config.radar.target_frame == "base_link"
    assert config.radar.voxel_size_m == 0.08


def test_default_app_config_matches_radar_live_dry_run_safety_geometry() -> None:
    config = AppConfig.default()

    assert config.roi.x_min == -1.2
    assert config.roi.x_max == 3.0
    assert config.roi.y_min == -1.8
    assert config.roi.y_max == 1.8
    assert config.roi.z_min == -1.2
    assert config.roi.z_max == 1.8
    assert config.roi.ground_z_max == -1.2

    assert config.self_filter.enabled is True
    assert config.self_filter.x_min == -0.35
    assert config.self_filter.x_max == 0.20
    assert config.self_filter.y_min == -0.45
    assert config.self_filter.y_max == 0.45
    assert config.self_filter.z_min == -0.5
    assert config.self_filter.z_max == 1.5

    assert config.zones.min_points == 10
    assert config.zones.emergency_stop_distance_m == 0.45
    assert config.zones.slowdown_distance_m == 1.2
    assert config.zones.rotation_emergency_stop_distance_m == 0.45
    assert config.zones.bounds["front"].x_min == 0.25
    assert config.zones.bounds["front"].x_max == 1.2
    assert config.zones.bounds["front"].y_min == -0.40
    assert config.zones.bounds["front"].y_max == 0.40
    assert config.zones.bounds["left"].y_min == 0.35
    assert config.zones.bounds["right"].y_max == -0.35
    assert config.zones.bounds["rotation"].x_min == -0.45
    assert config.zones.bounds["rotation"].z_min == -0.25


def test_default_app_config_matches_radar_live_dry_run_avoidance() -> None:
    config = AppConfig.default()

    assert config.avoidance.lateral_vy_mps == 0.12
    assert config.avoidance.forward_vx_when_avoiding_mps == 0.03
    assert config.avoidance.min_slowdown_ratio == 0.25
    assert config.avoidance.max_vx == 0.20
    assert config.avoidance.max_vy == 0.15
    assert config.avoidance.max_omega == 0.50
    assert config.avoidance.stateful_enabled is True
    assert config.avoidance.avoid_clear_required_frames == 4
    assert config.avoidance.avoid_clearance_hold_s == 0.4
    assert config.avoidance.side_switch_cooldown_s == 1.0
    assert config.avoidance.side_danger_distance_m == 0.35
    assert config.avoidance.side_score_min_margin_m == 0.08
    assert config.avoidance.tie_break_side == "left"
