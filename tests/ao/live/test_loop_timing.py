import pytest

from robot_follow.ao.live.loop_timing import LoopTiming, LoopTimingConfig


class FakeClock:
    def __init__(self, values):
        self._values = iter(values)

    def __call__(self):
        return next(self._values)


def test_40hz_config_has_25ms_period():
    config = LoopTimingConfig(target_hz=40.0)

    assert config.period_s == 0.025


def test_sleep_until_next_tick_uses_remaining_25ms_period():
    sleeps = []
    timing = LoopTiming(
        config=LoopTimingConfig(target_hz=40.0),
        clock=FakeClock([10.010]),
        sleeper=sleeps.append,
    )

    requested = timing.sleep_until_next_tick(started_at=10.000)

    assert requested == pytest.approx(0.015)
    assert sleeps == [pytest.approx(0.015)]


def test_overrun_does_not_sleep():
    sleeps = []
    timing = LoopTiming(
        config=LoopTimingConfig(target_hz=40.0),
        clock=FakeClock([10.035]),
        sleeper=sleeps.append,
    )

    requested = timing.sleep_until_next_tick(started_at=10.000)

    assert requested == 0.0
    assert sleeps == []


def test_rejects_non_positive_target_hz():
    with pytest.raises(ValueError, match="target_hz"):
        LoopTimingConfig(target_hz=0.0)


def test_web_10hz_throttle_allows_ao_40hz_ticks_between_updates():
    # Web publishing at 10 Hz (0.1 s period) must not slow the 40 Hz AO tick
    # cadence: multiple AO ticks happen between web updates. The predicate is
    # imported from production so the test locks the real throttle behavior.
    # NOTE: started_at=10.101 (not 10.100) avoids a float-precision edge where
    # 10.1 - 10.0 == 0.0999... < 0.1; 10.101 is clearly past the web period.
    from robot_follow.ao.live.web_cli import _should_update_web

    web_period_s = 0.1

    assert _should_update_web(
        last_web_update_at=None, started_at=10.000, web_period_s=web_period_s
    )
    assert not _should_update_web(
        last_web_update_at=10.000, started_at=10.025, web_period_s=web_period_s
    )
    assert not _should_update_web(
        last_web_update_at=10.000, started_at=10.075, web_period_s=web_period_s
    )
    assert _should_update_web(
        last_web_update_at=10.000, started_at=10.101, web_period_s=web_period_s
    )
