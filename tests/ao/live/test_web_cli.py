"""Tests for the integrated AO radar web CLI."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from typing import Optional
from unittest.mock import MagicMock, patch

import pytest


@dataclass(frozen=True)
class MockWristbandState:
    enabled: Optional[bool] = None
    updated_at: Optional[float] = None


class MockCloud:
    def __init__(self, timestamp=0.0):
        self.timestamp = timestamp
        self.frame_id = "body"
        self.valid = True
        self.empty_is_clear = False
        self.reason = "mock"
        self.received_at_s = timestamp
        self.points_xyz = []


class MockCloudSource:
    def __init__(self, cloud=None):
        self._cloud = cloud

    def latest(self):
        return self._cloud


class MockRuntime:
    def __init__(self, wristband_state=None, radar_state="stopped", cloud=None):
        self._wristband_state = wristband_state
        self._radar_state = radar_state
        self.cloud_source = MockCloudSource(cloud)
        self._web_planner = None

    def _get_wristband_state(self):
        return self._wristband_state

    def _radar_state(self):
        return self._radar_state

    def tick(self):
        pass


# Mock the serial module before importing web_cli
sys.modules["serial"] = MagicMock()

from robot_follow.ao.app.config import AppConfig
from robot_follow.ao.web.server import VisualizerHub


def test_update_web_hub_wristband_off_shows_status():
    """手环OFF时，web hub应显示wristband_off状态。"""
    from robot_follow.ao.live.web_cli import _update_web_hub

    hub = VisualizerHub()
    runtime = MockRuntime(wristband_state=False, radar_state="running")
    config = AppConfig.default()

    _update_web_hub(runtime, hub, config, max_points=100, now_s=100.0)

    latest = hub.latest()
    assert latest is not None
    assert latest["type"] == "status"
    assert latest["payload"]["text"] == "wristband_off"


def test_update_web_hub_wristband_unknown_shows_status():
    """手环状态未知时，web hub应显示wristband_unknown。"""
    from robot_follow.ao.live.web_cli import _update_web_hub

    hub = VisualizerHub()
    runtime = MockRuntime(wristband_state=None, radar_state="stopped")
    config = AppConfig.default()

    _update_web_hub(runtime, hub, config, max_points=100, now_s=100.0)

    latest = hub.latest()
    assert latest is not None
    assert latest["type"] == "status"
    assert latest["payload"]["text"] == "wristband_unknown"


def test_update_web_hub_radar_not_running_shows_radar_state():
    """雷达未运行时，web hub应显示当前雷达状态。"""
    from robot_follow.ao.live.web_cli import _update_web_hub

    hub = VisualizerHub()
    runtime = MockRuntime(wristband_state=True, radar_state="stopped")
    config = AppConfig.default()

    _update_web_hub(runtime, hub, config, max_points=100, now_s=100.0)

    latest = hub.latest()
    assert latest is not None
    assert latest["type"] == "status"
    assert latest["payload"]["text"] == "radar_stopped"


def test_update_web_hub_cloud_missing_shows_not_ready():
    """雷达运行但点云为None时，应显示radar_not_ready。"""
    from robot_follow.ao.live.web_cli import _update_web_hub

    hub = VisualizerHub()
    runtime = MockRuntime(wristband_state=True, radar_state="running", cloud=None)
    config = AppConfig.default()

    _update_web_hub(runtime, hub, config, max_points=100, now_s=100.0)

    latest = hub.latest()
    assert latest is not None
    assert latest["type"] == "status"
    assert latest["payload"]["text"] == "radar_not_ready"


def test_update_web_hub_wristband_on_and_cloud_ready_sends_frame():
    """手环ON + 雷达运行 + 点云有效时，应发送frame消息。"""
    from robot_follow.ao.live.web_cli import _update_web_hub

    hub = VisualizerHub()
    cloud = MockCloud(timestamp=99.9)
    runtime = MockRuntime(wristband_state=True, radar_state="running", cloud=cloud)
    config = AppConfig.default()

    _update_web_hub(runtime, hub, config, max_points=100, now_s=100.0)

    latest = hub.latest()
    assert latest is not None
    assert latest["type"] == "frame"