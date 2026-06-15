"""Headless live ALX-G100 simulation runner.

This module keeps the real ALX-G100 input path but replaces any real robot
output with MockMotionBridge recording. It is intended for reliable follow
logic testing without Tkinter GUI scheduling or drawing overhead.
"""

from __future__ import annotations

import argparse
import csv
import logging
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Optional, TextIO

from robot_follow.core import FollowPipeline, MotionCommand, TargetSample
from robot_follow.core.pipeline import PipelineStepResult
from robot_follow.ports import MotionBridge
from robot_follow.real.alx_source import ALXLiveSource, ALXSnapshot
from robot_follow.sim.mock_motion_bridge import MockMotionBridge

Clock = Callable[[], float]
Sleeper = Callable[[float], None]
Emitter = Callable[[str], None]

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class LiveALXConfig:
    """Configuration for the headless live ALX simulation loop."""

    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    hz: float = 40.0
    stale_timeout_s: float = 0.5
    duration_s: Optional[float] = None
    frame_limit: Optional[int] = None
    print_every_s: float = 0.5
    csv_path: Optional[Path] = None


@dataclass(frozen=True)
class LiveALXFrame:
    """One processed live ALX frame."""

    index: int
    elapsed_s: float
    status: str
    sample: TargetSample
    step: PipelineStepResult
    alx_fps: float = 0.0
    alx_age_ms: float = 0.0
    target_x_m: float = 0.0
    target_y_m: float = 0.0


@dataclass
class LiveALXRunResult:
    """Summary of a headless live ALX simulation run."""

    frames: list[LiveALXFrame] = field(default_factory=list)
    bridge: MotionBridge = field(default_factory=MockMotionBridge)

    @property
    def move_count(self) -> int:
        return self.bridge.move_count if isinstance(self.bridge, MockMotionBridge) else 0

    @property
    def stop_count(self) -> int:
        return self.bridge.stop_count if isinstance(self.bridge, MockMotionBridge) else 0

    @property
    def last(self) -> Optional[MotionCommand]:
        return self.bridge.last if isinstance(self.bridge, MockMotionBridge) else None


def target_xy_from_sample(sample: TargetSample) -> tuple[float, float]:
    """Return target coordinates in the ALX receiver frame.

    The receiver is treated as the origin. ``+x`` is straight ahead and
    ``+y`` follows the sign convention of the ALX azimuth angle.
    """
    azimuth_rad = math.radians(sample.azimuth_deg)
    return (
        sample.distance_m * math.cos(azimuth_rad),
        sample.distance_m * math.sin(azimuth_rad),
    )


def sample_from_snapshot(
    snapshot: Optional[ALXSnapshot],
    *,
    now: float,
    stale_timeout_s: float,
) -> tuple[TargetSample, str, bool, float, float]:
    """Convert latest ALX snapshot into pipeline input.

    Returns ``(sample, status, serial_error, alx_fps, alx_age_ms)``.
    """
    if snapshot is None:
        return (
            TargetSample(
                timestamp=now,
                distance_m=0.0,
                azimuth_deg=0.0,
                valid=False,
            ),
            "waiting_for_position",
            False,
            0.0,
            0.0,
        )

    age_s = snapshot.age_s(now)
    age_ms = age_s * 1000.0 if snapshot.timestamp > 0 else 0.0

    if snapshot.error is not None:
        return (
            TargetSample(
                timestamp=now,
                distance_m=snapshot.distance_m,
                azimuth_deg=snapshot.azimuth_deg,
                valid=False,
            ),
            f"error: {snapshot.error}",
            True,
            snapshot.fps,
            age_ms,
        )

    sample = TargetSample(
        timestamp=snapshot.timestamp,
        distance_m=snapshot.distance_m,
        azimuth_deg=snapshot.azimuth_deg,
        valid=True,
    )
    if age_s > stale_timeout_s:
        return sample, f"stale {age_ms:.0f}ms", False, snapshot.fps, age_ms
    return sample, "live", False, snapshot.fps, age_ms


def _format_frame(frame: LiveALXFrame) -> str:
    command = frame.step.command
    return (
        f"frame={frame.index:05d} t={frame.elapsed_s:7.3f}s "
        f"status={frame.status} alx_fps={frame.alx_fps:5.1f} "
        f"age={frame.alx_age_ms:6.1f}ms "
        f"d={frame.sample.distance_m:5.2f}m "
        f"az={frame.sample.azimuth_deg:6.1f}deg "
        f"xy=({frame.target_x_m:+5.2f},{frame.target_y_m:+5.2f})m "
        f"valid={frame.sample.valid} "
        f"cmd={command.kind}/{command.reason} "
        f"vx={command.vx:+.3f} omega={command.omega:+.4f}"
    )


def _write_csv_header(writer: csv.DictWriter) -> None:
    writer.writeheader()


def _write_csv_frame(writer: csv.DictWriter, frame: LiveALXFrame) -> None:
    command = frame.step.command
    writer.writerow(
        {
            "frame": frame.index,
            "elapsed_s": f"{frame.elapsed_s:.6f}",
            "status": frame.status,
            "alx_fps": f"{frame.alx_fps:.3f}",
            "alx_age_ms": f"{frame.alx_age_ms:.3f}",
            "timestamp": f"{frame.sample.timestamp:.6f}",
            "distance_m": f"{frame.sample.distance_m:.6f}",
            "azimuth_deg": f"{frame.sample.azimuth_deg:.6f}",
            "target_x_m": f"{frame.target_x_m:.6f}",
            "target_y_m": f"{frame.target_y_m:.6f}",
            "valid": frame.sample.valid,
            "command_kind": command.kind,
            "command_reason": command.reason,
            "vx": f"{command.vx:.6f}",
            "vy": f"{command.vy:.6f}",
            "omega": f"{command.omega:.6f}",
        }
    )


def create_motion_bridge(args: argparse.Namespace) -> "MotionBridge":
    """Create the appropriate motion bridge based on CLI flags.

    Returns a MotionBridge (MockMotionBridge or G1MotionBridge). Real motion via
    G1MotionBridge requires both ``--enable-motion`` AND
    ``--i-understand-this-moves-robot``. The G1 modules are imported lazily
    inside the enable-motion branch so a dry-run never touches them; the SDK
    itself is further deferred to ``G1MotionBridge.init()``.
    """
    from robot_follow.sim.mock_motion_bridge import MockMotionBridge

    if not args.enable_motion:
        logger.warning("DRY-RUN MODE: Using MockMotionBridge, robot will NOT move")
        return MockMotionBridge()

    if not args.i_understand_this_moves_robot:
        logger.error("Real motion requires --i-understand-this-moves-robot consent")
        raise SystemExit(1)

    from robot_follow.real.g1 import G1BridgeConfig, G1MotionBridge

    logger.warning("REAL MOTION ENABLED: Robot will move!")
    cfg_kwargs = {"yaw_sign": args.yaw_sign}
    if args.max_vx is not None:
        cfg_kwargs["max_abs_vx"] = args.max_vx
    if args.max_omega is not None:
        cfg_kwargs["max_abs_omega"] = args.max_omega
    cfg = G1BridgeConfig(**cfg_kwargs)
    return G1MotionBridge(args.net_iface, config=cfg)


def run_live_alx_simulation(
    config: LiveALXConfig,
    *,
    source: Optional[ALXLiveSource] = None,
    pipeline: Optional[FollowPipeline] = None,
    bridge: Optional[MotionBridge] = None,
    emit: Emitter = print,
    clock: Clock = time.monotonic,
    sleep: Sleeper = time.sleep,
    csv_file: Optional[TextIO] = None,
) -> LiveALXRunResult:
    """Run real ALX input through the follow pipeline without robot output."""
    if config.hz <= 0.0 or not math.isfinite(config.hz):
        raise ValueError(f"hz must be finite and > 0, got {config.hz}")
    if config.duration_s is None and config.frame_limit is None:
        emit("Running until Ctrl-C. This records commands only; it does not move the robot.")

    owns_source = source is None
    if source is None:
        source = ALXLiveSource(
            port=config.port,
            baudrate=config.baudrate,
            timeout=min(0.05, 1.0 / config.hz),
        )
    if pipeline is None:
        pipeline = FollowPipeline(clock=clock)
    if bridge is None:
        bridge = MockMotionBridge()

    result = LiveALXRunResult(bridge=bridge)
    csv_handle: Optional[TextIO] = None
    writer: Optional[csv.DictWriter] = None

    if csv_file is not None:
        writer = _csv_writer(csv_file)
        _write_csv_header(writer)
    elif config.csv_path is not None:
        config.csv_path.parent.mkdir(parents=True, exist_ok=True)
        csv_handle = config.csv_path.open("w", newline="", encoding="utf-8")
        writer = _csv_writer(csv_handle)
        _write_csv_header(writer)

    period_s = 1.0 / config.hz
    start_s = clock()
    next_due_s = start_s
    last_print_s = -math.inf

    try:
        bridge.init()
        source.start()
        frame_index = 0
        while True:
            now = clock()
            elapsed_s = now - start_s
            if config.duration_s is not None and elapsed_s >= config.duration_s:
                break
            if config.frame_limit is not None and frame_index >= config.frame_limit:
                break

            snapshot = source.snapshot()
            sample, status, serial_error, alx_fps, alx_age_ms = sample_from_snapshot(
                snapshot,
                now=now,
                stale_timeout_s=config.stale_timeout_s,
            )
            target_x_m, target_y_m = target_xy_from_sample(sample)
            step = pipeline.process(
                sample,
                serial_error=serial_error,
                sdk_error=bridge.sdk_error,
            )
            bridge.dispatch(step.command)
            bridge.tick(now)

            frame = LiveALXFrame(
                index=frame_index,
                elapsed_s=elapsed_s,
                status=status,
                sample=sample,
                step=step,
                alx_fps=alx_fps,
                alx_age_ms=alx_age_ms,
                target_x_m=target_x_m,
                target_y_m=target_y_m,
            )
            result.frames.append(frame)
            if writer is not None:
                _write_csv_frame(writer, frame)

            if elapsed_s - last_print_s >= config.print_every_s:
                emit(_format_frame(frame))
                last_print_s = elapsed_s

            frame_index += 1
            next_due_s += period_s
            now = clock()
            if next_due_s < now:
                missed = math.floor((now - next_due_s) / period_s) + 1
                next_due_s += missed * period_s
            sleep(max(0.0, next_due_s - now))
    except KeyboardInterrupt:
        emit("Interrupted by user.")
    finally:
        try:
            bridge.stop("shutdown")
        except Exception as e:
            emit(f"bridge shutdown stop failed: {e}")
        try:
            bridge.close()
        except Exception as e:
            emit(f"bridge close failed: {e}")
        source.stop()
        if csv_handle is not None:
            csv_handle.close()

    emit(
        f"summary frames={len(result.frames)} moves={result.move_count} "
        f"stops={result.stop_count} last={result.last}"
    )
    return result


def _csv_writer(handle: TextIO) -> csv.DictWriter:
    return csv.DictWriter(
        handle,
        fieldnames=[
            "frame",
            "elapsed_s",
            "status",
            "alx_fps",
            "alx_age_ms",
            "timestamp",
            "distance_m",
            "azimuth_deg",
            "target_x_m",
            "target_y_m",
            "valid",
            "command_kind",
            "command_reason",
            "vx",
            "vy",
            "omega",
        ],
    )


def _parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run real ALX-G100 input through the follow pipeline without GUI "
            "or real robot output."
        )
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="ALX serial port.")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--hz", type=float, default=40.0, help="Pipeline rate.")
    parser.add_argument("--duration", type=float, default=None, help="Run seconds.")
    parser.add_argument("--frames", type=int, default=None, help="Run frame count.")
    parser.add_argument("--stale-timeout", type=float, default=0.5)
    parser.add_argument("--print-every", type=float, default=0.5)
    parser.add_argument("--csv", type=Path, default=None, help="Optional CSV log path.")

    # Motion output switches (G1 hardware safety gating). These are parsed and
    # validated here; bridge selection via create_motion_bridge is wired in a
    # later task. Without --enable-motion the runner must use MockMotionBridge
    # (no real robot motion).
    parser.add_argument("--dry-run", action="store_true", default=False,
                        help="Use MockMotionBridge (no real robot motion).")
    parser.add_argument("--enable-motion", action="store_true", default=False,
                        help="Enable G1MotionBridge (requires --i-understand-this-moves-robot).")
    parser.add_argument("--i-understand-this-moves-robot", action="store_true", default=False,
                        help="Explicit consent: the robot WILL physically move.")
    parser.add_argument("--net-iface", default="enp2s0",
                        help="DDS network interface for G1 SDK.")
    parser.add_argument("--max-vx", type=float, default=None,
                        help="Override max forward/backward velocity (m/s).")
    parser.add_argument("--max-omega", type=float, default=None,
                        help="Override max rotation velocity (rad/s).")
    parser.add_argument("--yaw-sign", type=int, default=1, choices=[1, -1],
                        help="Yaw sign calibration (+1 or -1).")

    args = parser.parse_args(argv)
    if args.dry_run and args.enable_motion:
        parser.error("--dry-run and --enable-motion are mutually exclusive")
    return args


def main(argv: Optional[Iterable[str]] = None) -> None:
    args = _parse_args(argv)
    config = LiveALXConfig(
        port=args.port,
        baudrate=args.baudrate,
        hz=args.hz,
        stale_timeout_s=args.stale_timeout,
        duration_s=args.duration,
        frame_limit=args.frames,
        print_every_s=args.print_every,
        csv_path=args.csv,
    )
    bridge = create_motion_bridge(args)
    run_live_alx_simulation(config, bridge=bridge)


if __name__ == "__main__":
    main()
