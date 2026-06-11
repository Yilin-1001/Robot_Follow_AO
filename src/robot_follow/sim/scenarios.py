"""Builtin offline simulation scenarios.

Each scenario is a deterministic, named sequence of TargetSample frames
designed to exercise a specific behaviour of the core pipeline:

* straight_approach — target walks straight toward the robot (approach only)
* angle_alignment   — large azimuth that decreases to zero (turn-then-go)
* too_close         — gradual approach enters the safety zone (stop)
* jump              — a sudden large distance change (distance_jump stop)
* dropout           — signal drops out via invalid samples (data_invalid stop)

All scenarios use fixed timestamps starting at 1000.0 s with a 0.05 s step.
When run through the default `run_simulation()` runner, the virtual clock
is driven to match each sample's timestamp, so samples are fresh by default.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict

from robot_follow.core.types import TargetSample

# ---------------------------------------------------------------------------
# Timestamp helpers
# ---------------------------------------------------------------------------
_BASE_TS = 1000.0
_STEP_S = 0.05


def _ts(i: int) -> float:
    """Return a deterministic timestamp for frame index i."""
    return _BASE_TS + i * _STEP_S


# ---------------------------------------------------------------------------
# Scenario builders
# ---------------------------------------------------------------------------
def _straight_approach() -> list[TargetSample]:
    """Distance 2.5m → ~1.3m over ~12 frames, azimuth fixed at 0deg.

    Ends with a small plateau at 1.3m to let the filter converge closer
    to the target distance. All frames should be "move" commands with
    decreasing vx; the final frames have very small forward velocity.
    """
    samples: list[TargetSample] = []
    # Descent phase: 2.5 → 1.3 over 12 frames
    n_descent = 12
    start = 2.5
    end = 1.3
    for i in range(n_descent):
        d = start - (start - end) * (i / (n_descent - 1))
        samples.append(TargetSample(timestamp=_ts(i), distance_m=d, azimuth_deg=0.0))
    # Plateau: 4 frames at exactly 1.3m to let the filter catch up
    for j in range(4):
        idx = n_descent + j
        samples.append(
            TargetSample(timestamp=_ts(idx), distance_m=1.3, azimuth_deg=0.0)
        )
    return samples


def _angle_alignment() -> list[TargetSample]:
    """Distance ~2.0m fixed, azimuth 30deg → 0deg over 9 frames.

    Early frames have abs(azimuth) > 20deg → aligning (turn in place).
    Mid frames are in the "medium angle" band → tracking_turning.
    Late frames fall in the deadzone → tracking_forward.
    """
    angles = [30.0, 26.0, 22.0, 18.0, 14.0, 10.0, 6.0, 2.0, 0.0]
    samples: list[TargetSample] = []
    for i, az in enumerate(angles):
        samples.append(
            TargetSample(timestamp=_ts(i), distance_m=2.0, azimuth_deg=az)
        )
    return samples


def _too_close() -> list[TargetSample]:
    """Gradual approach from 1.2m down to 0.48m with small steps (~0.08m).

    The steps are small enough to avoid triggering distance_jump (>0.5m),
    but the filtered distance eventually drops below min_safe_distance
    (0.75m), causing a "too_close" stop.
    """
    start = 1.2
    end = 0.48
    step = 0.08
    distances: list[float] = []
    d = start
    while d >= end:
        distances.append(round(d, 3))
        d -= step
    samples: list[TargetSample] = []
    for i, dval in enumerate(distances):
        samples.append(
            TargetSample(timestamp=_ts(i), distance_m=dval, azimuth_deg=0.0)
        )
    return samples


def _jump() -> list[TargetSample]:
    """Stable at 2.0m for 3 frames, then a sudden jump to 3.5m (Δ = 1.5m).

    The large raw jump exceeds max_distance_jump_m (0.5m), causing
    a distance_jump stop on the fourth frame.
    """
    distances = [2.0, 2.0, 2.0, 3.5]
    samples: list[TargetSample] = []
    for i, dval in enumerate(distances):
        samples.append(
            TargetSample(timestamp=_ts(i), distance_m=dval, azimuth_deg=0.0)
        )
    return samples


def _dropout() -> list[TargetSample]:
    """Two valid frames followed by three invalid frames (valid=False).

    Invalid samples cause the pipeline to emit data_invalid stops and
    skip the filter. The runner keeps ticking, so the dropout period
    is processed as multiple stop frames.
    """
    samples: list[TargetSample] = []
    # Valid phase
    for i in range(2):
        samples.append(
            TargetSample(timestamp=_ts(i), distance_m=2.0, azimuth_deg=0.0, valid=True)
        )
    # Dropout phase (invalid samples)
    for j in range(3):
        idx = 2 + j
        samples.append(
            TargetSample(
                timestamp=_ts(idx), distance_m=2.0, azimuth_deg=0.0, valid=False
            )
        )
    return samples


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------
_BUILDERS: Dict[str, Callable[[], list[TargetSample]]] = {
    "straight_approach": _straight_approach,
    "angle_alignment": _angle_alignment,
    "too_close": _too_close,
    "jump": _jump,
    "dropout": _dropout,
}

_DESCRIPTIONS: Dict[str, str] = {
    "straight_approach": "Distance 2.5m → 1.3m, azimuth 0°, approach behaviour.",
    "angle_alignment": "Distance ~2.0m, azimuth 30° → 0°, turn-then-go.",
    "too_close": "Gradual approach enters the safety zone (stop).",
    "jump": "Sudden large distance jump (distance_jump stop).",
    "dropout": "Signal dropout via invalid samples (data_invalid stop).",
}


@dataclass(frozen=True)
class Scenario:
    """A named, deterministic simulation scenario.

    Attributes
    ----------
    name
        Canonical scenario name.
    description
        One-line summary.
    samples
        Tuple of TargetSample frames.
    """

    name: str
    description: str
    samples: tuple[TargetSample, ...]


def _build_registry() -> Dict[str, Scenario]:
    """Build the frozen registry of builtin scenarios."""
    registry: Dict[str, Scenario] = {}
    for key, builder in _BUILDERS.items():
        samples = tuple(builder())
        registry[key] = Scenario(
            name=key, description=_DESCRIPTIONS.get(key, ""), samples=samples
        )
    return registry


BUILTIN_SCENARIOS: Dict[str, Scenario] = _build_registry()


def list_scenarios() -> list[str]:
    """Return the list of builtin scenario names."""
    return sorted(BUILTIN_SCENARIOS.keys())


def get_scenario(name: str) -> Scenario:
    """Return a Scenario by name (case-insensitive).

    Raises
    ------
    KeyError
        If no builtin scenario matches the name.
    """
    key = name.lower().replace("-", "_").replace(" ", "_")
    if key not in BUILTIN_SCENARIOS:
        raise KeyError(f"Unknown scenario: {name}. Available: {list_scenarios()}")
    return BUILTIN_SCENARIOS[key]