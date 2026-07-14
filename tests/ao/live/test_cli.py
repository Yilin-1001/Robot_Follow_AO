from pathlib import Path

import pytest

from robot_follow.ao.live.cli import (
    MockMotionBridge,
    WristbandStatusPrinter,
    build_runtime_from_config,
)
from robot_follow.ao.live.loop_timing import LoopTimingConfig


def test_real_g1_requires_explicit_confirmation(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text(
        "ao_radar:\n"
        "  output:\n"
        "    mode: g1\n"
        "    require_real_g1_confirmation: true\n"
        "    confirm_real_g1: false\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="confirm_real_g1"):
        build_runtime_from_config(path)


def test_g1_bridge_uses_configured_network_interface(tmp_path: Path, monkeypatch) -> None:
    path = tmp_path / "config.yaml"
    path.write_text(
        "ao_radar:\n"
        "  output:\n"
        "    mode: g1\n"
        "    require_real_g1_confirmation: true\n"
        "    confirm_real_g1: true\n"
        "    network_interface: eth0\n",
        encoding="utf-8",
    )
    captured = {}

    class FakeG1MotionBridge:
        def __init__(self, network_interface: str, config) -> None:
            captured["network_interface"] = network_interface
            captured["config"] = config

        def close(self) -> None:
            pass

    monkeypatch.setattr(
        "robot_follow.ao.live.cli.G1MotionBridge",
        FakeG1MotionBridge,
    )

    build_runtime_from_config(path)

    assert captured["network_interface"] == "eth0"
    assert captured["config"].max_abs_vx == 0.10
    assert captured["config"].max_abs_omega == 0.30


def test_runtime_uses_follow_safety_filter_and_g1_yaml_settings(tmp_path: Path, monkeypatch) -> None:
    path = tmp_path / "config.yaml"
    path.write_text(
        "ao_radar:\n"
        "  output:\n"
        "    mode: g1\n"
        "    require_real_g1_confirmation: true\n"
        "    confirm_real_g1: true\n"
        "  filter:\n"
        "    alpha: 0.60\n"
        "  follow:\n"
        "    target_distance_m: 1.2\n"
        "    max_forward_vx: 0.16\n"
        "  safety:\n"
        "    min_safe_distance_m: 0.35\n"
        "  g1:\n"
        "    max_vx: 0.07\n"
        "    max_omega: 0.18\n",
        encoding="utf-8",
    )
    captured = {}

    class FakeG1MotionBridge:
        def __init__(self, network_interface: str, config) -> None:
            captured["config"] = config

        def close(self) -> None:
            pass

    monkeypatch.setattr("robot_follow.ao.live.cli.G1MotionBridge", FakeG1MotionBridge)

    runtime = build_runtime_from_config(path)

    assert runtime.pipeline.filter.alpha == 0.60
    assert runtime.pipeline.controller.config.target_distance_m == 1.2
    assert runtime.pipeline.controller.config.max_forward_vx == 0.16
    assert runtime.pipeline.safety.config.min_safe_distance_m == 0.35
    assert captured["config"].max_abs_vx == 0.07
    assert captured["config"].max_abs_omega == 0.18


def test_runtime_accepts_ordered_config_overlays(tmp_path: Path) -> None:
    base = tmp_path / "base.yaml"
    base.write_text(
        "follow:\n"
        "  target_distance_m: 1.8\n"
        "ao_radar:\n"
        "  output:\n"
        "    mode: mock\n",
        encoding="utf-8",
    )
    overlay = tmp_path / "overlay.yaml"
    overlay.write_text(
        "ao_radar:\n"
        "  follow:\n"
        "    target_distance_m: 1.2\n",
        encoding="utf-8",
    )

    runtime = build_runtime_from_config([base, overlay])

    assert runtime.pipeline.controller.config.target_distance_m == 1.2


def test_mock_mode_uses_mock_bridge(tmp_path: Path, monkeypatch) -> None:
    path = tmp_path / "config.yaml"
    path.write_text(
        "ao_radar:\n"
        "  output:\n"
        "    mode: mock\n",
        encoding="utf-8",
    )

    def fail_g1_bridge(*args, **kwargs):
        raise AssertionError("G1 bridge should not be constructed in mock mode")

    monkeypatch.setattr("robot_follow.ao.live.cli.G1MotionBridge", fail_g1_bridge)

    runtime = build_runtime_from_config(path)

    assert isinstance(runtime.bridge, MockMotionBridge)


def test_rejects_unknown_output_mode(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text(
        "ao_radar:\n"
        "  output:\n"
        "    mode: dry_run\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="mode"):
        build_runtime_from_config(path)


def test_ao_cli_40hz_period_is_25ms() -> None:
    config = LoopTimingConfig(target_hz=40.0)

    assert config.period_s == 0.025


def test_wristband_status_printer_prints_each_main_thread_transition_once(capsys) -> None:
    class State:
        def __init__(self, enabled) -> None:
            self.enabled = enabled

    class Source:
        def __init__(self) -> None:
            self.enabled = None

        def wristband_state(self):
            return State(self.enabled)

    source = Source()
    printer = WristbandStatusPrinter()

    printer.emit_if_changed(source)
    source.enabled = True
    printer.emit_if_changed(source)
    printer.emit_if_changed(source)
    source.enabled = False
    printer.emit_if_changed(source)

    assert capsys.readouterr().out == "[WRISTBAND] ON\n[WRISTBAND] OFF\n"


def test_planner_applies_configured_avoidance_omega_limit(tmp_path: Path) -> None:
    """Planner must clamp omega to ao_radar.avoidance.max_omega, not the
    hardcoded 0.8 default nor the larger follow/g1 limits."""
    path = tmp_path / "config.yaml"
    path.write_text(
        "ao_radar:\n"
        "  output:\n"
        "    mode: mock\n"
        "  follow:\n"
        "    max_omega: 3.0\n"
        "  g1:\n"
        "    max_omega: 3.0\n"
        "  avoidance:\n"
        "    max_vx: 0.5\n"
        "    max_vy: 0.2\n"
        "    max_omega: 1.2\n",
        encoding="utf-8",
    )

    runtime = build_runtime_from_config(path)

    from robot_follow.ao.live.command_envelope import CommandEnvelope
    from robot_follow.ao.core.types import ObstacleReport
    from robot_follow.core.types import MotionCommand

    raw = MotionCommand(kind="move", vx=0.0, vy=0.0, omega=3.0, reason="aligning")
    envelope = CommandEnvelope(command=raw, generated_at=10.0, source="robot_follow")
    clear_report = ObstacleReport.clear(timestamp=10.0)

    safe = runtime.planner.filter_command(envelope, clear_report, now=10.01)

    assert safe.kind == "move"
    assert safe.omega == pytest.approx(1.2)
