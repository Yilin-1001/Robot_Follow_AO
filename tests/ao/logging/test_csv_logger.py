from datetime import datetime
from pathlib import Path

from robot_follow.ao.logging.csv_logger import AORadarCSVLogger, AORadarLogRow, timestamped_log_path


def test_csv_logger_writes_required_latency_fields(tmp_path) -> None:
    path = tmp_path / "ao.csv"
    logger = AORadarCSVLogger(path)

    logger.write(
        AORadarLogRow(
            timestamp=10.0,
            alx_age_ms=5.0,
            command_age_ms=3.0,
            cloud_age_ms=20.0,
            loop_dt_ms=50.0,
            ao_process_ms=4.0,
            g1_dispatch_ms=2.0,
            raw_command_kind="move",
            raw_command_reason="follow",
            safe_command_kind="stop",
            safe_command_reason="front_blocked",
            radar_state="running",
            wristband_gate_state="on",
            raw_azimuth_deg=31.0,
            filtered_azimuth_deg=28.5,
            g1_sent_omega=-0.3,
            distance_m=1.25,
            tag_status=3,
            robot_yaw_deg=42.5,
            robot_yaw_rate_rps=-0.27,
        )
    )
    logger.close()

    text = path.read_text(encoding="utf-8")

    assert "alx_age_ms,command_age_ms,cloud_age_ms" in text
    assert "raw_azimuth_deg,filtered_azimuth_deg,g1_sent_omega" in text
    assert "distance_m,tag_status,robot_yaw_deg,robot_yaw_rate_rps" in text
    assert "1.25,3,42.5,-0.27" in text
    assert "front_blocked" in text


def test_timestamped_log_path_preserves_directory_and_suffix() -> None:
    path = timestamped_log_path(
        Path("logs/ao_radar_frames.csv"),
        time_format="%Y%m%d_%H%M%S",
        now=datetime(2026, 7, 9, 12, 34, 56),
    )

    assert path == Path("logs/ao_radar_frames_20260709_123456.csv")
