"""Terminal logger for the GUI simulation.

Two log categories:

* **FRAME** – rate-limited (~10 Hz by default), summarises every field.
* **EVENT** – emitted immediately on meaningful state transitions (target
  entering / leaving FOV, command changes).

The logging logic is pure Python with no GUI dependency so it can be
unit-tested without opening a window.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, List, Optional


# ---------------------------------------------------------------------------
# Per-frame state snapshot
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class FrameState:
    """Immutable snapshot of one simulation frame for the logger."""

    t: float
    dt: float
    target_x: float
    target_y: float
    distance_m: float
    azimuth_deg: float
    valid: bool
    in_fov: bool
    command_kind: str
    command_reason: str
    vx: float
    omega: float
    robot_x: float
    robot_y: float
    robot_heading_deg: float


# ---------------------------------------------------------------------------
# SimLogger
# ---------------------------------------------------------------------------

class SimLogger:
    """Terminal logger for the GUI simulation.

    Parameters
    ----------
    threshold_s
        Minimum interval between successive FRAME logs (default 0.1 → ~10 Hz).
    emit
        Callable that receives each log line (default ``print``).  Injecting
        a custom callable makes unit testing straightforward.
    """

    def __init__(
        self,
        threshold_s: float = 0.1,
        emit: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.threshold_s = threshold_s
        self._emit = emit or print
        self.verbose: bool = False

        self._last_frame_t: float = -1e9
        self._prev_kind: Optional[str] = None
        self._prev_reason: Optional[str] = None
        self._prev_valid: Optional[bool] = None
        self._prev_in_fov: Optional[bool] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def log_frame(self, state: FrameState, *, force: bool = False) -> None:
        """Process one frame: emit FRAME log (rate-limited) and EVENT logs.

        Parameters
        ----------
        state
            The current frame snapshot.
        force
            If ``True``, emit the FRAME log regardless of rate-limiting.
        """
        # --- Event detection (always runs) --------------------------------
        self._check_events(state)

        # --- Frame log (rate-limited) -------------------------------------
        should_log = force or self.verbose or (state.t - self._last_frame_t >= self.threshold_s)
        if should_log:
            self._emit_frame(state)
            self._last_frame_t = state.t

        # Track previous state for next frame's event detection.
        self._prev_kind = state.command_kind
        self._prev_reason = state.command_reason
        self._prev_valid = state.valid
        self._prev_in_fov = state.in_fov

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _check_events(self, state: FrameState) -> None:
        """Emit EVENT lines when state transitions occur."""
        prev = self._prev_valid
        prev_fov = self._prev_in_fov
        prev_reason = self._prev_reason

        # Target left FOV (was valid/in_fov, now not)
        if prev is not None and prev_fov is not None:
            if (prev and prev_fov) and not (state.valid and state.in_fov):
                self._emit(
                    f"EVENT target_left_fov "
                    f"azimuth={state.azimuth_deg:.1f}deg "
                    f"distance={state.distance_m:.2f}m"
                )

            # Target reacquired
            if (not (prev and prev_fov)) and (state.valid and state.in_fov):
                self._emit(
                    f"EVENT target_reacquired "
                    f"azimuth={state.azimuth_deg:.1f}deg "
                    f"distance={state.distance_m:.2f}m"
                )

        # Command changed
        if prev_reason is not None:
            prev_kind = self._prev_kind or ""
            if (
                state.command_kind != prev_kind
                or state.command_reason != prev_reason
            ):
                self._emit(
                    f"EVENT command_changed "
                    f"from={prev_kind}/{prev_reason} "
                    f"to={state.command_kind}/{state.command_reason}"
                )

    def _emit_frame(self, s: FrameState) -> None:
        """Format and emit a FRAME log line."""
        self._emit(
            f"FRAME t={s.t:.2f} dt={s.dt:.3f} "
            f"target=({s.target_x:.2f},{s.target_y:.2f})m "
            f"distance={s.distance_m:.2f}m "
            f"azimuth={s.azimuth_deg:.1f}deg "
            f"valid={s.valid} in_fov={s.in_fov} "
            f"command={s.command_kind} reason={s.command_reason} "
            f"vx={s.vx:.3f} omega={s.omega:.4f} "
            f"robot=({s.robot_x:.2f},{s.robot_y:.2f},{s.robot_heading_deg:.1f}deg)"
        )
