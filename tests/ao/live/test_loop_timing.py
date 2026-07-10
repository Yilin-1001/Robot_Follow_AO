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
