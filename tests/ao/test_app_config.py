from robot_follow.ao.app.config import AppConfig


def test_default_app_config_uses_live_body_frame_roll_transform() -> None:
    config = AppConfig.default()

    assert config.transform.apply_static_transform is True
    assert config.transform.roll_deg == 180.0
