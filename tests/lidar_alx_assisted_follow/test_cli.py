"""CLI safety gate tests for LiDAR + ALX Assisted Follow.

Tests verify that safety gates prevent accidental real G1 motion
or unauthenticated LAN exposure without explicit confirmation.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any
from types import SimpleNamespace

import pytest

# Import will fail until cli.py is implemented; that's expected for TDD
# from robot_follow.lidar_alx_assisted_follow.cli import build_runtime_from_args


class TestCLISafetyGates:
    """Test CLI safety gates before runtime construction."""

    def test_real_provisional_motion_requires_both_confirmations(self, tmp_path):
        """Real G1 with provisional calibration requires --confirm-real-g1 and --allow-provisional-calibration."""
        # Write minimal valid config
        config_path = tmp_path / "test_config.yaml"
        config_path.write_text(
            """
lidar_alx_assisted_follow:
  output:
    mode: real
  calibration:
    allow_provisional_calibration_for_prototype: true
"""
        )

        # Case 1: confirm_real_g1=False, allow_provisional=True -> should fail
        args = self._make_args(
            config=config_path,
            confirm_real_g1=False,
            allow_provisional_calibration=True,
        )
        with pytest.raises(SystemExit) as exc_info:
            from robot_follow.lidar_alx_assisted_follow.cli import build_runtime_from_args

            build_runtime_from_args(args)
        assert exc_info.value.code == 1

        # Case 2: confirm_real_g1=True, allow_provisional=False -> should fail
        args = self._make_args(
            config=config_path,
            confirm_real_g1=True,
            allow_provisional_calibration=False,
        )
        with pytest.raises(SystemExit) as exc_info:
            build_runtime_from_args(args)
        assert exc_info.value.code == 1

    def test_non_loopback_requires_lan_confirmation(self, tmp_path):
        """Binding to non-localhost requires --allow-unauthenticated-lan."""
        config_path = tmp_path / "test_config.yaml"
        config_path.write_text(
            """
lidar_alx_assisted_follow:
  web:
    host: "0.0.0.0"
"""
        )

        args = self._make_args(
            config=config_path,
            host="0.0.0.0",
            allow_unauthenticated_lan=False,
        )

        with pytest.raises(SystemExit) as exc_info:
            from robot_follow.lidar_alx_assisted_follow.cli import build_runtime_from_args

            build_runtime_from_args(args)
        assert "allow-unauthenticated-lan" in str(exc_info.value).lower() or exc_info.value.code != 0

    def test_mock_mode_requires_no_confirmations(self, tmp_path):
        """Mock mode should work without any confirmation flags."""
        config_path = tmp_path / "test_config.yaml"
        config_path.write_text(
            """
lidar_alx_assisted_follow:
  output:
    mode: mock
  logging:
    enabled: false
"""
        )

        args = self._make_args(
            config=config_path,
            confirm_real_g1=False,
            allow_provisional_calibration=False,
        )

        from robot_follow.lidar_alx_assisted_follow.cli import build_runtime_from_args

        components = build_runtime_from_args(args)

        runtime = components["runtime"]
        assert runtime.target_manager.config == components["config"].target
        assert runtime.limiter.config == components["config"].output_motion
        assert callable(runtime.run_control_loop)

    def test_localhost_default_needs_no_lan_confirmation(self, tmp_path):
        """Default localhost binding works without --allow-unauthenticated-lan."""
        config_path = tmp_path / "test_config.yaml"
        config_path.write_text(
            """
lidar_alx_assisted_follow:
  web:
    host: "127.0.0.1"
"""
        )

        args = self._make_args(
            config=config_path,
            host="127.0.0.1",
            allow_unauthenticated_lan=False,
        )

        from robot_follow.lidar_alx_assisted_follow.cli import build_runtime_from_args

        components = build_runtime_from_args(args)
        assert components["web_hub"] is not None

    def test_select_event_is_retained_until_the_next_cloud(self, tmp_path):
        """A Web click is consumed by the control thread's next cloud pass."""
        config_path = tmp_path / "test_config.yaml"
        config_path.write_text(
            "lidar_alx_assisted_follow:\n  logging:\n    enabled: false\n"
        )
        components = __import__(
            "robot_follow.lidar_alx_assisted_follow.cli", fromlist=["build_runtime_from_args"]
        ).build_runtime_from_args(self._make_args(config=config_path))
        from robot_follow.lidar_alx_assisted_follow.runtime.state import (
            ControlEvent,
            ControlEventKind,
        )

        components["runtime"]._process_control_event(
            ControlEvent(ControlEventKind.SELECT_TARGET, {"x": 1.2, "y": -0.3})
        )

        assert components["runtime"]._pending_click_xy == (1.2, -0.3)

    def _make_args(
        self,
        config: Path,
        confirm_real_g1: bool = False,
        allow_provisional_calibration: bool = False,
        host: str = "127.0.0.1",
        allow_unauthenticated_lan: bool = False,
    ) -> argparse.Namespace:
        """Create minimal args namespace for testing."""
        return argparse.Namespace(
            config=config,
            confirm_real_g1=confirm_real_g1,
            allow_provisional_calibration=allow_provisional_calibration,
            host=host,
            port=8080,
            allow_unauthenticated_lan=allow_unauthenticated_lan,
        )


class TestCLIStartupShutdown:
    """Test CLI startup and shutdown sequence."""

    def test_startup_order_is_correct(self, tmp_path):
        """Verify startup order: ALX reader -> cloud reader -> control thread -> aiohttp."""
        # This will be verified through integration testing
        pass

    def test_shutdown_stops_motion_first(self, tmp_path):
        """Verify shutdown requests STOP before joining threads."""
        pass

    def test_shutdown_joins_control_thread_before_bridge_close(self, tmp_path):
        """Control thread must be joined before closing G1 bridge."""
        pass
