"""Unit tests for the staged G1 hardware check CLI helpers."""

from __future__ import annotations

import pytest

from robot_follow.real.g1 import check


def test_validate_network_interface_accepts_available_name(monkeypatch):
    monkeypatch.setattr(check, "available_network_interfaces", lambda: ["eth0", "lo"])

    check.validate_network_interface("eth0")


def test_validate_network_interface_reports_available_names(monkeypatch):
    monkeypatch.setattr(check, "available_network_interfaces", lambda: ["eth0", "lo"])

    with pytest.raises(SystemExit) as exc_info:
        check.validate_network_interface("enp2s0")

    message = str(exc_info.value)
    assert "--net-iface 'enp2s0' is not an available network interface" in message
    assert "Available interfaces: eth0, lo" in message
    assert "ip -br link" in message


def test_validate_network_interface_allows_sdk_to_report_if_listing_fails(monkeypatch):
    monkeypatch.setattr(check, "available_network_interfaces", lambda: [])

    check.validate_network_interface("enp2s0")
