"""Dedicated G1 hardware verification entry point.

This module is a thin manual-probing tool for staged G1 hardware validation
(see docs/alx_g1_verification_chain.md). It is deliberately separate from the
follow loop:

- It does NOT read ALX data.
- It does NOT run run_live_alx_simulation / the follow loop.
- It only exercises G1MotionBridge directly: init, stop, or one short bounded
  move.

Every invocation requires explicit consent (``--i-understand-this-moves-robot``)
because every mode that reaches ``init()`` connects to the real DDS/SDK path.
"""

from __future__ import annotations

import argparse
import logging
import socket
import time

from robot_follow.core.types import MotionCommand
from robot_follow.real.g1 import G1BridgeConfig, G1MotionBridge

logger = logging.getLogger(__name__)


def available_network_interfaces() -> list[str]:
    """Return OS network interface names visible to this process."""
    try:
        return sorted(name for _, name in socket.if_nameindex())
    except OSError as exc:
        logger.warning("Unable to list network interfaces: %s", exc)
        return []


def validate_network_interface(net_iface: str) -> None:
    """Fail before SDK init if the requested DDS interface is not present."""
    interfaces = available_network_interfaces()
    if not interfaces or net_iface in interfaces:
        return

    available = ", ".join(interfaces)
    raise SystemExit(
        f"--net-iface {net_iface!r} is not an available network interface. "
        f"Available interfaces: {available}. "
        "Run `ip -br link` on the robot PC and rerun with the interface "
        "connected to the G1, for example `--net-iface eth0`."
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Staged G1 hardware verification: init-only, stop-only, or a single "
            "short bounded move. Does NOT read ALX data or run the follow loop."
        )
    )
    parser.add_argument("--net-iface", default="enp2s0",
                        help="DDS network interface for G1 SDK (default: enp2s0).")
    # Modes (mutually-tolerated: if several are set, the first matching branch
    # in main() wins — init-only < stop-only < single-move).
    parser.add_argument("--init-only", action="store_true", default=False,
                        help="Initialize the bridge, then close. Sends no move.")
    parser.add_argument("--stop-only", action="store_true", default=False,
                        help="Initialize, send one StopMove, then close.")
    parser.add_argument("--single-move", action="store_true", default=False,
                        help="Initialize, send one short bounded Move, StopMove, close.")
    # Consent gate.
    parser.add_argument("--i-understand-this-moves-robot", action="store_true",
                        default=False,
                        help="Explicit consent required for ALL modes here.")
    # Velocity clamps passed straight into G1BridgeConfig. Defaults mirror
    # G1BridgeConfig so a bare invocation is always valid; the verification
    # chain passes more conservative values explicitly.
    parser.add_argument("--max-vx", type=float, default=G1BridgeConfig().max_abs_vx,
                        help="max_abs_vx clamp (m/s). Defaults to G1BridgeConfig.")
    parser.add_argument("--max-omega", type=float, default=G1BridgeConfig().max_abs_omega,
                        help="max_abs_omega clamp (rad/s). Defaults to G1BridgeConfig.")
    # Single-move command values (only used under --single-move). Defaults are
    # the conservative first-motion values from the verification chain.
    parser.add_argument("--vx", type=float, default=0.05,
                        help="Forward velocity for --single-move (m/s, default 0.05).")
    parser.add_argument("--omega", type=float, default=0.0,
                        help="Yaw velocity for --single-move (rad/s, default 0.0).")
    parser.add_argument("--duration", type=float, default=0.5,
                        help="Hold time for --single-move (s, default 0.5).")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    if not args.i_understand_this_moves_robot:
        raise SystemExit("--i-understand-this-moves-robot is required")

    validate_network_interface(args.net_iface)

    cfg = G1BridgeConfig(max_abs_vx=args.max_vx, max_abs_omega=args.max_omega)
    bridge = G1MotionBridge(args.net_iface, config=cfg)
    try:
        bridge.init()
        if args.init_only:
            logger.info("g1 check: init-only complete")
            return
        if args.stop_only:
            logger.info("g1 check: stop-only")
            bridge.stop("g1_check_stop_only")
            return
        if args.single_move:
            logger.info(
                "g1 check: single move vx=%.3f omega=%.3f duration=%.2f",
                args.vx, args.omega, args.duration,
            )
            bridge.dispatch(MotionCommand(
                kind="move",
                vx=args.vx,
                vy=0.0,
                omega=args.omega,
                reason="g1_check_single_move",
            ))
            time.sleep(args.duration)
            bridge.stop("g1_check_single_move_complete")
            return
        logger.info("g1 check: no mode selected; init-only behavior")
    finally:
        bridge.stop("g1_check_shutdown")
        bridge.close()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
