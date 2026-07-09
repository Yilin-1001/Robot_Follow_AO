from robot_follow.ao.logging.csv_logger import AORadarCSVLogger, AORadarLogRow


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
        )
    )
    logger.close()

    text = path.read_text(encoding="utf-8")

    assert "alx_age_ms,command_age_ms,cloud_age_ms" in text
    assert "front_blocked" in text