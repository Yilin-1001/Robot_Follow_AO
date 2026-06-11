"""Unit tests for the target filter."""

from robot_follow.core.filtering import FilteredTarget, TargetFilter


def test_first_sample_initialises():
    f = TargetFilter()
    result = f.update(distance_m=2.5, azimuth_deg=10.0)
    assert isinstance(result, FilteredTarget)
    assert result.initialized
    assert result.distance_m == 2.5
    assert result.azimuth_deg == 10.0
    assert result.distance_jump == 0.0
    assert result.angle_jump == 0.0


def test_not_initialized_before_first_update():
    f = TargetFilter()
    assert not f.initialized


def test_subsequent_update_applies_smoothing():
    f = TargetFilter(alpha=0.5)
    f.update(distance_m=2.0, azimuth_deg=10.0)
    result = f.update(distance_m=3.0, azimuth_deg=20.0)
    # With alpha=0.5: new = old + 0.5 * (raw - old)
    assert result.distance_m == 2.5
    assert result.azimuth_deg == 15.0
    assert result.distance_jump == 1.0
    assert result.angle_jump == 10.0


def test_alpha_must_be_valid():
    try:
        TargetFilter(alpha=0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for alpha=0.0")

    try:
        TargetFilter(alpha=1.5)
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for alpha=1.5")


def test_reset_clears_state():
    f = TargetFilter()
    f.update(distance_m=2.0, azimuth_deg=5.0)
    assert f.initialized
    f.reset()
    assert not f.initialized


def test_reset_then_update_reinitialises():
    f = TargetFilter()
    f.update(distance_m=2.0, azimuth_deg=5.0)
    f.reset()
    result = f.update(distance_m=3.0, azimuth_deg=15.0)
    assert result.distance_m == 3.0
    assert result.azimuth_deg == 15.0
    assert result.distance_jump == 0.0
    assert result.angle_jump == 0.0