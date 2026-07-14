"""CLI entry point for LiDAR + ALX Assisted Follow prototype.

This is the composition root that wires together all components
and enforces safety gates before runtime construction.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

from robot_follow.lidar_alx_assisted_follow.config.loader import load_prototype_config
from robot_follow.lidar_alx_assisted_follow.config.adapters import (
    build_radar_service_config,
    build_g1_bridge_config,
)
from robot_follow.lidar_alx_assisted_follow.domain.models import StopKind
from robot_follow.lidar_alx_assisted_follow.runtime.state import (
    ControlEventInbox,
    ControlEventKind,
    LatestStatus,
    StopLatch,
)
from robot_follow.lidar_alx_assisted_follow.runtime.controller import (
    LidarALXAssistedFollowRuntime,
    ALXReaderProtocol,
)
from robot_follow.lidar_alx_assisted_follow.runtime.csv_log import PrototypeCSVLogger
from robot_follow.lidar_alx_assisted_follow.sensors.cloud import SequencedFastLIOPointCloudSource
from robot_follow.lidar_alx_assisted_follow.sensors.alx import ALXSnapshot, ALXSnapshotBuffer
from robot_follow.lidar_alx_assisted_follow.sensors.lifecycle import WristbandLifecycleGuardian
from robot_follow.lidar_alx_assisted_follow.web.server import PrototypeWebHub, create_app
from robot_follow.real.g1.bridge import G1MotionBridge
from robot_follow.real.alx_source import ALXLiveSource

logger = logging.getLogger(__name__)


# Mock bridge for testing without G1 hardware
class MockMotionBridge:
    """Mock output for full dry-runs without touching the G1 SDK."""

    def __init__(self) -> None:
        self.commands: list[Any] = []
        self._initialized = False
        self._closed = False

    def init(self) -> None:
        """Initialize mock bridge."""
        if self._initialized:
            return
        self._initialized = True
        print("[PROTOTYPE_OUTPUT] mode=mock initialized", flush=True)

    def dispatch(self, command: Any) -> None:
        """Record command for testing."""
        self.commands.append(command)

    def tick(self, *, now: float) -> None:
        """No-op for mock."""
        pass

    def close(self) -> None:
        """Close mock bridge."""
        if self._closed:
            return
        self._closed = True
        print("[PROTOTYPE_OUTPUT] mode=mock closed", flush=True)

    @property
    def state(self) -> str:
        """Return mock state."""
        return "INITIALIZED" if self._initialized else "STOPPED"

    def is_ok(self) -> bool:
        """Mock bridge is always OK."""
        return True


class ALXLiveReaderAdapter(ALXReaderProtocol):
    """Adapter from ALXLiveSource to ALXReaderProtocol."""

    def __init__(self, source: ALXLiveSource) -> None:
        self._source = source

    def latest_snapshot(self):
        """Get latest snapshot from live source."""
        from robot_follow.lidar_alx_assisted_follow.sensors.alx import ALXSnapshot
        state = self._source.wristband_state()

        if not state.enabled:
            return None

        # Convert ALXLiveSource state to ALXSnapshot
        return ALXSnapshot(
            received_at=time.monotonic(),
            tag_id=state.tag_id or 0,
            distance_m=state.distance or 0.0,
            azimuth_deg=state.azimuth or 0.0,
            elevation_deg=state.elevation or 0.0,
            error_code=state.error_code or 0,
            enabled=state.enabled,
        )

    def is_ok(self) -> bool:
        """Check if serial communication is healthy."""
        return self._source.is_ok()


def build_runtime_from_args(args: argparse.Namespace) -> dict[str, Any]:
    """Build runtime components from parsed CLI arguments.

    Enforces safety gates:
    - Real G1 with provisional calibration requires both confirmations
    - Non-loopback binding requires LAN confirmation

    Parameters
    ----------
    args : argparse.Namespace
        Parsed CLI arguments.

    Returns
    -------
    dict[str, Any]
        Dictionary of runtime components.

    Raises
    ------
    SystemExit
        If safety gates are not satisfied.
    """
    # Load configuration
    config = load_prototype_config(args.config)

    # Safety gate 1: Real G1 with provisional calibration
    output_mode = config.output.mode
    allow_provisional = config.calibration.allow_provisional_calibration_for_prototype
    confirm_real_g1 = args.confirm_real_g1
    allow_provisional_flag = args.allow_provisional_calibration

    if output_mode == "real":
        if not confirm_real_g1:
            logger.error(
                "Real G1 output requires --confirm-real-g1 flag.\n"
                "This flag confirms you are ready for physical robot motion."
            )
            raise SystemExit(1)

        if allow_provisional and not allow_provisional_flag:
            logger.error(
                "Configuration has provisional calibration enabled.\n"
                "Provisional calibration requires --allow-provisional-calibration flag.\n"
                "WARNING: Provisional calibration has NOT been verified with real hardware."
            )
            raise SystemExit(1)

    # Safety gate 2: Non-loopback binding
    host = args.host if args.host else config.web.host
    if host != "127.0.0.1" and host != "localhost":
        if not args.allow_unauthenticated_lan:
            logger.error(
                f"Non-loopback binding '{host}' requires --allow-unauthenticated-lan.\n"
                "WARNING: This exposes the control interface to the local network "
                "WITHOUT authentication. Use only in trusted environments."
            )
            raise SystemExit(1)

        # Print LAN risk warning
        print(
            "\n"
            "=" * 70 + "\n"
            "WARNING: UNAUTHENTICATED LAN ACCESS ENABLED\n"
            "=" * 70 + "\n"
            f"Web interface will be accessible at {host}:{args.port}\n"
            "Anyone on the local network can connect and control the robot.\n"
            "Use only in trusted, isolated networks.\n"
            "=" * 70 + "\n",
            file=sys.stderr,
            flush=True,
        )

    # Build components
    components = {}

    # Stop latch (thread-safe)
    control_thread_id = threading.get_ident()
    stop_latch = StopLatch(control_thread_id=control_thread_id)
    components["stop_latch"] = stop_latch

    # Event inbox and status
    event_inbox = ControlEventInbox(capacity=config.runtime.control_event_capacity)
    components["event_inbox"] = event_inbox

    latest_status = LatestStatus()
    components["latest_status"] = latest_status

    # Cloud source
    cloud_source = SequencedFastLIOPointCloudSource()
    components["cloud_source"] = cloud_source

    # ALX buffer
    alx_buffer = ALXSnapshotBuffer(
        alx_config=config.alx,
        calibration_config=config.calibration,
        buffer_duration_s=1.0,
    )
    components["alx_buffer"] = alx_buffer

    # CSV logger
    csv_logger: Optional[PrototypeCSVLogger] = None
    if config.logging.enabled:
        csv_logger = PrototypeCSVLogger(
            csv_path=config.logging.csv_path,
            enabled=True,
            flush_interval_s=config.logging.flush_interval_s,
        )
        components["csv_logger"] = csv_logger

    # Motion bridge
    if output_mode == "mock":
        bridge = MockMotionBridge()
    else:
        bridge_config = build_g1_bridge_config(config)
        bridge = G1MotionBridge(bridge_config)
    components["bridge"] = bridge

    # ALX reader (will be created in main, this is just for type hints)
    components["alx_reader"] = None  # Placeholder, set in main()

    # Lifecycle guardian
    lifecycle = WristbandLifecycleGuardian(
        ready_timeout_s=config.sensors.ready_timeout_s,
    )
    components["lifecycle"] = lifecycle

    # Runtime
    runtime = LidarALXAssistedFollowRuntime(
        config=config,
        cloud_source=cloud_source,
        alx_buffer=alx_buffer,
        stop_latch=stop_latch,
        event_inbox=event_inbox,
        latest_status=latest_status,
        bridge=bridge,
        lifecycle=lifecycle,
        csv_logger=csv_logger,
        alx_reader=None,  # Set in main()
    )
    components["runtime"] = runtime

    # Web hub
    web_hub = PrototypeWebHub(
        stop_latch=stop_latch,
        event_inbox=event_inbox,
        latest_status=latest_status,
        max_clients=config.web.max_clients,
        max_events_per_second=config.web.max_events_per_second,
    )
    components["web_hub"] = web_hub

    return components


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    """Parse command-line arguments.

    Parameters
    ----------
    argv : Optional[list[str]]
        Argument list, defaults to sys.argv[1:].

    Returns
    -------
    argparse.Namespace
        Parsed arguments.
    """
    parser = argparse.ArgumentParser(
        description="LiDAR + ALX Assisted Follow Prototype",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Safety Flags:
  --confirm-real-g1            Required for real G1 motion output
  --allow-provisional-calibration  Required if calibration is provisional
  --allow-unauthenticated-lan  Required for non-localhost web binding

Examples:
  # Mock mode (no hardware)
  robot-follow-lidar-alx-assisted-follow config.yaml

  # Real G1 with provisional calibration
  robot-follow-lidar-alx-assisted-follow config.yaml \\
      --confirm-real-g1 --allow-provisional-calibration

  # LAN access
  robot-follow-lidar-alx-assisted-follow config.yaml \\
      --host 0.0.0.0 --allow-unauthenticated-lan
        """,
    )

    parser.add_argument(
        "config",
        type=Path,
        help="Path to YAML configuration file",
    )

    parser.add_argument(
        "--confirm-real-g1",
        action="store_true",
        help="Confirm readiness for real G1 motion (required for mode=g1)",
    )

    parser.add_argument(
        "--allow-provisional-calibration",
        action="store_true",
        help="Allow operation with provisional calibration (not verified)",
    )

    parser.add_argument(
        "--host",
        type=str,
        default=None,
        help="Web server host (overrides config, default: 127.0.0.1)",
    )

    parser.add_argument(
        "--port",
        type=int,
        default=None,
        help="Web server port (overrides config)",
    )

    parser.add_argument(
        "--allow-unauthenticated-lan",
        action="store_true",
        help="Allow non-localhost binding (WARNING: no authentication)",
    )

    parser.add_argument(
        "--log-level",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        default="INFO",
        help="Logging level",
    )

    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    """Main entry point for LiDAR + ALX Assisted Follow CLI.

    Parameters
    ----------
    argv : Optional[list[str]]
        Argument list, defaults to sys.argv[1:].

    Returns
    -------
    int
        Exit code (0 for success, non-zero for error).
    """
    args = parse_args(argv)

    # Configure logging
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    try:
        # Build components (with safety gates)
        components = build_runtime_from_args(args)

        # Extract components
        config = load_prototype_config(args.config)
        stop_latch = components["stop_latch"]
        event_inbox = components["event_inbox"]
        latest_status = components["latest_status"]
        cloud_source = components["cloud_source"]
        alx_buffer = components["alx_buffer"]
        bridge = components["bridge"]
        lifecycle = components["lifecycle"]
        runtime = components["runtime"]
        web_hub = components["web_hub"]
        csv_logger = components.get("csv_logger")

        # Create ALX reader
        alx_live_source = ALXLiveSource(
            port=config.alx.port,
            baudrate=config.alx.baudrate,
        )
        alx_reader = ALXLiveReaderAdapter(alx_live_source)
        runtime._alx_reader = alx_reader

        # Shutdown coordination
        shutdown_requested = threading.Event()

        def handle_shutdown(signum, frame):
            """Signal handler for graceful shutdown."""
            logger.info(f"Received signal {signum}, initiating shutdown...")
            shutdown_requested.set()

        signal.signal(signal.SIGINT, handle_shutdown)
        signal.signal(signal.SIGTERM, handle_shutdown)

        # Startup sequence
        logger.info("Starting LiDAR + ALX Assisted Follow Prototype...")
        logger.info(f"Output mode: {config.output.mode}")
        logger.info(f"Web host: {args.host or config.web.host}")
        logger.info(f"Web port: {args.port or config.web.port}")

        # 1. Start ALX reader
        logger.info("Starting ALX serial reader...")
        alx_live_source.start()

        # 2. Initialize bridge
        logger.info("Initializing motion bridge...")
        bridge.init()

        # 3. Start control thread
        logger.info("Starting control thread...")
        control_thread = threading.Thread(
            target=runtime.run_control_loop,
            name="control",
            daemon=False,
        )
        control_thread.start()

        # 4. Build and start web server
        import asyncio
        from aiohttp import web

        app = create_app(web_hub)
        host = args.host if args.host else config.web.host
        port = args.port if args.port else config.web.port

        logger.info(f"Starting web server at {host}:{port}...")

        async def run_web():
            """Run web server with shutdown handling."""
            runner = web.AppRunner(app)
            await runner.setup()
            site = web.TCPSite(runner, host, port)
            await site.start()

            logger.info(f"Web server running at http://{host}:{port}")

            # Wait for shutdown signal
            while not shutdown_requested.is_set():
                await asyncio.sleep(0.1)

            logger.info("Shutting down web server...")
            await runner.cleanup()

        # Run web server in separate thread
        web_thread = threading.Thread(
            target=lambda: asyncio.run(run_web()),
            name="web",
            daemon=False,
        )
        web_thread.start()

        logger.info("Startup complete. Press Ctrl+C to shutdown.")

        # Wait for shutdown
        while not shutdown_requested.is_set():
            shutdown_requested.wait(timeout=0.5)

    except SystemExit:
        # Re-raise SystemExit from safety gates
        raise
    except Exception as e:
        logger.exception(f"Startup failed: {e}")
        return 1

    # Shutdown sequence
    logger.info("Initiating graceful shutdown...")

    try:
        # 1. Request STOP
        logger.info("Requesting STOP...")
        stop_latch.request("shutdown_requested", StopKind.HARD)

        # 2. Join control thread
        logger.info("Waiting for control thread to finish...")
        control_thread.join(timeout=5.0)
        if control_thread.is_alive():
            logger.warning("Control thread did not shut down cleanly")

        # 3. Close bridge
        logger.info("Closing motion bridge...")
        bridge.close()

        # 4. Stop services (sensors)
        logger.info("Stopping sensor services...")
        # lifecycle.stop_services() would be called here if implemented

        # 5. Stop ALX reader
        logger.info("Stopping ALX reader...")
        alx_live_source.stop()

        # 6. Close CSV logger
        if csv_logger:
            logger.info("Closing CSV logger...")
            csv_logger.close()

        # 7. Join web thread
        logger.info("Waiting for web server to finish...")
        web_thread.join(timeout=2.0)
        if web_thread.is_alive():
            logger.warning("Web thread did not shut down cleanly")

        # 8. Close cloud source
        logger.info("Closing cloud source...")
        # cloud_source.close() would be called here if implemented

        logger.info("Shutdown complete")
        return 0

    except Exception as e:
        logger.exception(f"Error during shutdown: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
