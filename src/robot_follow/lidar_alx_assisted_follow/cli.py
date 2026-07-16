"""CLI entry point for LiDAR + ALX Assisted Follow prototype.

This is the composition root that wires together all components
and enforces safety gates before runtime construction.
"""

from __future__ import annotations

import argparse
import logging
import signal
import subprocess
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
from robot_follow.lidar_alx_assisted_follow.sensors.alx import ALXSnapshot
from robot_follow.lidar_alx_assisted_follow.sensors.alx_topic import (
    ALXPositionBuffer,
    ALXPositionTopicSource,
)
from robot_follow.lidar_alx_assisted_follow.sensors.lifecycle import WristbandLifecycleGuardian
from robot_follow.lidar_alx_assisted_follow.tracking.target import TargetLockManager
from robot_follow.lidar_alx_assisted_follow.motion.limiter import CommandSlewRateLimiter
from robot_follow.lidar_alx_assisted_follow.web.server import (
    WEB_SHUTDOWN_TIMEOUT_S,
    PrototypeWebHub,
    create_app,
)
from robot_follow.real.g1.bridge import G1MotionBridge
from robot_follow.ao.live.radar_service_manager import RadarServiceManager

logger = logging.getLogger(__name__)


class ManagedALXRosNode:
    """Run the sole ALX serial owner as a child of the assisted-follow CLI.

    Keeping the ROS publisher in its own process preserves its independent
    ROS2 context and the single-owner rule for ``/dev/ttyUSB*``.  The parent
    owns its lifecycle, so normal operation needs only one user command.
    """

    def __init__(self, config_path: Path) -> None:
        self._config_path = config_path.resolve()
        self._process: Optional[subprocess.Popen[object]] = None

    @property
    def command(self) -> list[str]:
        """Return the exact child command, using the parent's Python runtime."""
        return [
            sys.executable,
            "-m",
            "robot_follow.lidar_alx_assisted_follow.alx_ros_node",
            str(self._config_path),
        ]

    def start(self) -> None:
        """Start the ALX ROS publisher once."""
        if self.is_running:
            return
        try:
            self._process = subprocess.Popen(self.command)
        except OSError as exc:
            raise RuntimeError("Could not start managed ALX ROS publisher") from exc

    @property
    def return_code(self) -> Optional[int]:
        """Return the child exit code, or ``None`` while it is running."""
        return None if self._process is None else self._process.poll()

    @property
    def is_running(self) -> bool:
        """Whether the managed publisher is still alive."""
        return self._process is not None and self.return_code is None

    def stop(self, timeout_s: float = 3.0) -> None:
        """Terminate the publisher and wait for it to release the serial port."""
        process = self._process
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            logger.warning("Managed ALX ROS publisher did not exit; killing it")
            process.kill()
            process.wait(timeout=timeout_s)


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

    def tick(self, now: Optional[float] = None) -> None:
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
    """Legacy adapter retained for compatibility; production no longer uses it."""

    def __init__(self, source: Any) -> None:
        self._source = source

    def latest_snapshot(self):
        """Get latest snapshot from live source."""
        from robot_follow.lidar_alx_assisted_follow.sensors.alx import ALXSnapshot
        if self._source.wristband_state().enabled is not True:
            return None

        live_snapshot = self._source.snapshot()
        if live_snapshot is None:
            return None

        return ALXSnapshot(
            received_at=live_snapshot.received_at,
            tag_id=live_snapshot.tag_id,
            distance_m=live_snapshot.distance_m,
            azimuth_deg=live_snapshot.azimuth_deg,
            elevation_deg=live_snapshot.elevation_deg,
            error=live_snapshot.error is not None,
        )

    def is_ok(self) -> bool:
        """Check if serial communication is healthy."""
        snapshot = self._source.snapshot()
        return snapshot is None or snapshot.error is None

    def wristband_status(self):
        """Expose legacy serial wristband state through the topic-reader API."""
        from robot_follow.lidar_alx_assisted_follow.sensors.wristband import (
            WristbandStatus,
            WristbandTopicState,
        )

        snapshot = self._source.snapshot()
        if snapshot is not None and snapshot.error is not None:
            state = WristbandTopicState.SERIAL_FAULT
        else:
            enabled = self._source.wristband_state().enabled
            if enabled is True:
                state = WristbandTopicState.ON
            elif enabled is False:
                state = WristbandTopicState.OFF
            else:
                state = WristbandTopicState.UNKNOWN
        return WristbandStatus(state, received_at=time.monotonic())

    def is_wristband_status_fresh(self, now: Optional[float] = None) -> bool:
        del now
        return self.is_ok() and self._source.wristband_state().updated_at is not None


class UnavailableALXReader(ALXReaderProtocol):
    """Legacy safe placeholder retained for isolated tests."""

    def latest_snapshot(self) -> Optional[ALXSnapshot]:
        return None

    def is_ok(self) -> bool:
        return True

    def wristband_status(self):
        from robot_follow.lidar_alx_assisted_follow.sensors.wristband import (
            WristbandStatus,
            WristbandTopicState,
        )

        return WristbandStatus(WristbandTopicState.UNKNOWN, received_at=None)

    def is_wristband_status_fresh(self, now: Optional[float] = None) -> bool:
        del now
        return False


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
    components = {"config": config}

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
    cloud_source = SequencedFastLIOPointCloudSource(
        topic=config.sensors.topic,
        expected_frame_id=config.sensors.expected_frame_id,
        cloud_y_sign=config.sensors.cloud_y_sign,
    )
    components["cloud_source"] = cloud_source

    # Filtered ALX topic input.  This process never opens the serial port; the
    # standalone ALX ROS publisher is its sole owner.
    alx_buffer = ALXPositionBuffer(
        alx_config=config.alx,
        buffer_duration_s=0.5,
        max_samples=16,
    )
    components["alx_buffer"] = alx_buffer

    # CSV logger
    csv_logger = PrototypeCSVLogger(
        csv_path=config.logging.csv_path,
        enabled=config.logging.enabled,
        flush_interval_s=config.logging.flush_interval_s,
    )
    components["csv_logger"] = csv_logger

    # Motion bridge
    if output_mode == "mock":
        bridge = MockMotionBridge()
    else:
        bridge_config = build_g1_bridge_config(config)
        bridge = G1MotionBridge(config.g1.network_interface, config=bridge_config)
    components["bridge"] = bridge

    alx_reader = ALXPositionTopicSource(
        topic=config.alx_ros.topic,
        wristband_topic=config.alx_ros.wristband_topic,
        expected_frame_id=config.alx_ros.frame_id,
        stale_timeout_s=config.alx.stale_timeout_s,
        wristband_timeout_s=config.alx_ros.wristband_timeout_s,
        buffer=alx_buffer,
    )
    components["alx_reader"] = alx_reader

    # Lifecycle guardian
    lifecycle = WristbandLifecycleGuardian(
        ready_timeout_s=config.sensors.ready_timeout_s,
    )
    components["lifecycle"] = lifecycle
    radar_services = RadarServiceManager(build_radar_service_config(config))
    components["radar_services"] = radar_services

    target_manager = TargetLockManager(config.target)
    limiter = CommandSlewRateLimiter(config.output_motion)
    components["target_manager"] = target_manager
    components["limiter"] = limiter

    # Runtime
    runtime = LidarALXAssistedFollowRuntime(
        config=config,
        cloud_source=cloud_source,
        alx_reader=alx_reader,
        alx_buffer=alx_buffer,
        lifecycle=lifecycle,
        radar_services=radar_services,
        target_manager=target_manager,
        stop_latch=stop_latch,
        event_inbox=event_inbox,
        latest_status=latest_status,
        limiter=limiter,
        bridge=bridge,
        csv_logger=csv_logger,
    )
    components["runtime"] = runtime

    # Web hub
    web_hub = PrototypeWebHub(
        stop_latch=stop_latch,
        event_inbox=event_inbox,
        latest_status=latest_status,
        max_clients=config.web.max_clients,
        max_events_per_second=config.web.max_events_per_second,
        status_interval_s=1.0 / config.web.status_hz,
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

  # The ALX ROS publisher starts automatically.  Use --external-alx-ros only
  # when a separately managed publisher is intentionally already running.

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

    parser.add_argument(
        "--external-alx-ros",
        action="store_true",
        help=(
            "Do not start the ALX ROS publisher; subscribe to an intentionally "
            "separately managed publisher instead"
        ),
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
    managed_alx_ros_node: Optional[ManagedALXRosNode] = None
    managed_alx_ros_failed = False

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
        radar_services = components["radar_services"]
        bridge = components["bridge"]
        lifecycle = components["lifecycle"]
        runtime = components["runtime"]
        web_hub = components["web_hub"]
        csv_logger = components.get("csv_logger")

        # The ALX input is already composed as a ROS2 topic subscriber.  Do
        # not create ALXLiveSource here: the publisher node is the only serial
        # owner.
        alx_topic_source = components["alx_reader"]

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

        # 1. Start the sole serial owner unless an external supervisor owns it.
        if getattr(args, "external_alx_ros", False):
            logger.info("Using externally managed ALX ROS publisher")
        else:
            managed_alx_ros_node = ManagedALXRosNode(args.config)
            logger.info(
                "Starting managed ALX ROS publisher: %s",
                " ".join(managed_alx_ros_node.command),
            )
            managed_alx_ros_node.start()

        # 2. Start filtered ALX topic subscriber.
        logger.info(
            "Subscribing to ALX topics: position=%s wristband=%s...",
            config.alx_ros.topic,
            config.alx_ros.wristband_topic,
        )
        alx_topic_source.start()

        # 3. Start the cloud source.  It imports ROS only in its reader thread.
        logger.info("Starting FAST-LIO cloud reader...")
        cloud_source.start()

        # 4. Start control thread
        logger.info("Starting control thread...")
        control_thread = threading.Thread(
            target=runtime.run_control_loop,
            name="control",
            daemon=False,
        )
        control_thread.start()

        # 5. Build and start web server
        import asyncio
        from aiohttp import web

        static_dir = Path(__file__).parent / "frontend" / "dist"
        app = create_app(web_hub, static_dir=str(static_dir))
        host = args.host if args.host else config.web.host
        port = args.port if args.port else config.web.port

        logger.info(f"Starting web server at {host}:{port}...")

        async def run_web():
            """Run web server with shutdown handling."""
            runner = web.AppRunner(app, shutdown_timeout=WEB_SHUTDOWN_TIMEOUT_S)
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
            if (
                managed_alx_ros_node is not None
                and not managed_alx_ros_node.is_running
            ):
                logger.error(
                    "Managed ALX ROS publisher exited unexpectedly (code=%s); "
                    "requesting safe shutdown",
                    managed_alx_ros_node.return_code,
                )
                managed_alx_ros_failed = True
                shutdown_requested.set()
                continue
            shutdown_requested.wait(timeout=0.5)

    except SystemExit:
        # Re-raise SystemExit from safety gates
        raise
    except Exception as e:
        logger.exception(f"Startup failed: {e}")
        if managed_alx_ros_node is not None:
            managed_alx_ros_node.stop()
        return 1

    # Shutdown sequence
    logger.info("Initiating graceful shutdown...")

    try:
        # 1. Request STOP
        logger.info("Requesting STOP...")
        stop_latch.request("shutdown_requested", StopKind.HARD)

        # 2. Join control thread
        logger.info("Waiting for control thread to finish...")
        runtime.stop()
        control_thread.join(timeout=5.0)
        if control_thread.is_alive():
            logger.warning("Control thread did not shut down cleanly")

        # 3. Close bridge
        logger.info("Closing motion bridge...")
        bridge.close()

        # 4. Stop services (sensors)
        logger.info("Stopping sensor services...")
        radar_services.stop()

        # 5. Stop ALX topic subscriber
        logger.info("Stopping ALX topic subscriber...")
        alx_topic_source.stop()

        # 6. Stop the sole serial owner after subscribers have stopped.
        if managed_alx_ros_node is not None:
            logger.info("Stopping managed ALX ROS publisher...")
            managed_alx_ros_node.stop()

        # 7. Close CSV logger
        if csv_logger:
            logger.info("Closing CSV logger...")
            csv_logger.close()

        # 8. Join web thread
        logger.info("Waiting for web server to finish...")
        web_thread.join(timeout=WEB_SHUTDOWN_TIMEOUT_S + 1.0)
        if web_thread.is_alive():
            logger.warning("Web thread did not shut down cleanly")

        # 9. Close cloud source
        logger.info("Closing cloud source...")
        cloud_source.stop()

        logger.info("Shutdown complete")
        return 1 if managed_alx_ros_failed else 0

    except Exception as e:
        logger.exception(f"Error during shutdown: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
