"""Regression tests for the adapter from the existing ALX source."""

from __future__ import annotations

from types import SimpleNamespace

from robot_follow.real.alx_source import ALXSnapshot as LiveALXSnapshot


def test_live_adapter_preserves_source_received_time_and_error_state() -> None:
    """The prototype must consume the existing source's snapshot API verbatim."""
    from robot_follow.lidar_alx_assisted_follow.cli import ALXLiveReaderAdapter

    source = SimpleNamespace(
        wristband_state=lambda: SimpleNamespace(enabled=True),
        snapshot=lambda: LiveALXSnapshot(
            timestamp=12.0,
            distance_m=1.2,
            azimuth_deg=10.0,
            elevation_deg=0.0,
            tag_status=0,
            anchor_id=1,
            tag_id=42,
            fps=20.0,
            received_at=12.3,
            heartbeat_at=12.2,
            error=None,
        ),
    )

    snapshot = ALXLiveReaderAdapter(source).latest_snapshot()

    assert snapshot is not None
    assert snapshot.received_at == 12.3
    assert snapshot.tag_id == 42
    assert snapshot.error is False
