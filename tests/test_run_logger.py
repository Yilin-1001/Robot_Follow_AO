from __future__ import annotations

from datetime import datetime
from pathlib import Path

import pytest

from robot_follow.run_logger import (
    build_run_log_paths,
    parse_args,
    robot_args_with_csv,
    slugify,
    strip_csv_args,
    write_archive,
)


def test_slugify_filename_safe():
    assert slugify("Local Lab") == "local_lab"
    assert slugify("ALX dry-run #1") == "alx_dry_run_1"


def test_slugify_rejects_empty():
    with pytest.raises(ValueError):
        slugify("!!!")


def test_build_run_log_paths_include_env_type_and_time():
    paths = build_run_log_paths(
        logs_root=Path("logs"),
        test_env="local",
        test_type="alx_dry_run",
        timestamp=datetime(2026, 6, 15, 16, 5, 30),
    )

    assert paths.run_id == "20260615_160530_local_alx_dry_run"
    assert paths.run_dir == Path("logs/runs/20260615_160530_local_alx_dry_run")
    assert paths.csv_path.name == "20260615_160530_local_alx_dry_run_data.csv"
    assert paths.console_log_path.name == "20260615_160530_local_alx_dry_run_console.log"
    assert paths.command_path.name == "20260615_160530_local_alx_dry_run_command.txt"
    assert paths.metadata_path.name == "20260615_160530_local_alx_dry_run_metadata.json"
    assert paths.archive_path == Path("logs/20260615_160530_local_alx_dry_run.zip")


def test_robot_args_with_csv_replaces_user_csv():
    args = robot_args_with_csv(
        ["--config", "configs/jetson_g1_safe.yaml", "--csv", "old.csv"],
        Path("logs/runs/run/run_data.csv"),
    )

    assert args == [
        "--config",
        "configs/jetson_g1_safe.yaml",
        "--csv",
        "logs/runs/run/run_data.csv",
    ]


def test_strip_csv_args_handles_equals_form():
    assert strip_csv_args(["--duration", "1", "--csv=old.csv"]) == ["--duration", "1"]


def test_write_archive_collects_run_artifacts(tmp_path):
    paths = build_run_log_paths(
        logs_root=tmp_path,
        test_env="local",
        test_type="alx_dry_run",
        timestamp=datetime(2026, 6, 15, 16, 5, 30),
    )
    paths.run_dir.mkdir(parents=True)
    paths.csv_path.write_text("csv", encoding="utf-8")
    paths.console_log_path.write_text("console", encoding="utf-8")
    paths.command_path.write_text("command", encoding="utf-8")
    paths.metadata_path.write_text("metadata", encoding="utf-8")

    write_archive(paths)

    assert paths.archive_path.exists()


def test_parse_args_zip_is_opt_in():
    args = parse_args(["--test-env", "local", "--test-type", "alx", "--", "--duration", "1"])
    assert args.zip is False

    args_with_zip = parse_args(["--test-env", "local", "--test-type", "alx", "--zip"])
    assert args_with_zip.zip is True
