from datetime import datetime
from pathlib import Path

from robot_follow.ao.logging.csv_logger import AORadarCSVLogger, AORadarLogRow, timestamped_log_path


def test_csv_logger_writes_required_40hz_evidence_fields(tmp_path) -> None:
    path = tmp_path / "ao.csv"
    logger = AORadarCSVLogger(path)

    logger.write(
        AORadarLogRow(
            timestamp=10.0,
            alx_age_ms=5.0,
            command_age_ms=3.0,
            cloud_age_ms=20.0,
            loop_dt_ms=25.0,
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
            raw_command_generated_at_ms=2.0,
            safe_command_generated_at_ms=5.0,
            g1_response_at_ms=8.0,
            total_latency_ms=6.0,
        )
    )
    logger.close()

    lines = path.read_text(encoding="utf-8").splitlines()
    header = lines[0].split(",")

    for field in [
        "timestamp",
        "loop_dt_ms",
        "ao_process_ms",
        "g1_dispatch_ms",
        "raw_command_kind",
        "raw_command_reason",
        "safe_command_kind",
        "safe_command_reason",
        "command_age_ms",
        "cloud_age_ms",
        "raw_command_generated_at_ms",
        "safe_command_generated_at_ms",
        "g1_response_at_ms",
        "total_latency_ms",
    ]:
        assert field in header
    assert "25.0" in lines[1]
    assert "front_blocked" in lines[1]


def test_timestamped_log_path_preserves_directory_and_suffix() -> None:
    path = timestamped_log_path(
        Path("logs/ao_radar_frames.csv"),
        time_format="%Y%m%d_%H%M%S",
        now=datetime(2026, 7, 9, 12, 34, 56),
    )

    assert path == Path("logs/ao_radar_frames_20260709_123456.csv")
