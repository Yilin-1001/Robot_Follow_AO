from robot_follow.ao.live.fast_lio_source import FastLIOPointCloudSource, PointCloudFrame


def test_latest_frame_replaces_previous_frame() -> None:
    source = FastLIOPointCloudSource(topic="/cloud_registered_body")

    source.update_for_test(PointCloudFrame(timestamp=10.0, points=[(1.0, 0.0, 0.0)]))
    source.update_for_test(PointCloudFrame(timestamp=10.1, points=[(2.0, 0.0, 0.0)]))

    latest = source.latest()

    assert latest is not None
    assert latest.timestamp == 10.1
    assert latest.points == [(2.0, 0.0, 0.0)]


def test_frame_age_and_staleness() -> None:
    frame = PointCloudFrame(timestamp=10.0, points=[])

    assert frame.age_s(now=10.25) == 0.25
    assert frame.is_stale(now=10.6, max_age_s=0.5) is True
    assert frame.is_stale(now=10.4, max_age_s=0.5) is False