#!/usr/bin/env python3
"""Diagnostic script to identify blocking points in G1 startup.

Run this BEFORE the main CLI to see which component is blocking.
"""

import sys
import time
from pathlib import Path

def main():
    config_path = Path("configs/jetson_g1_ao_radar.yaml")

    print("=" * 70, flush=True)
    print("DIAGNOSTIC: G1 AO Radar Startup Check", flush=True)
    print("=" * 70, flush=True)

    # Layer 1: Config parsing
    print("\n[Layer 1] Config parsing...", flush=True)
    try:
        from robot_follow.ao.config import load_ao_radar_config
        config = load_ao_radar_config(config_path)
        print(f"  ✓ Config loaded: {config_path}", flush=True)
        print(f"    - ALX port: {config.alx.port}", flush=True)
        print(f"    - Output mode: {config.output.mode}", flush=True)
        print(f"    - Network interface: {config.output.network_interface}", flush=True)
        print(f"    - confirm_real_g1: {config.output.confirm_real_g1}", flush=True)
    except Exception as e:
        print(f"  ✗ Config failed: {e}", flush=True)
        return 1

    # Layer 2: Import checks
    print("\n[Layer 2] Import checks...", flush=True)
    imports = [
        ("robot_follow.ao.live.cli", "CLI entry point"),
        ("robot_follow.ao.live.runtime", "Runtime"),
        ("robot_follow.ao.live.fast_lio_source", "FastLIO source"),
        ("robot_follow.ao.live.radar_service_manager", "Radar service manager"),
        ("robot_follow.real.alx_source", "ALX source"),
        ("robot_follow.real.g1.bridge", "G1 bridge"),
    ]
    for module_name, description in imports:
        try:
            __import__(module_name)
            print(f"  ✓ {description}: {module_name}", flush=True)
        except Exception as e:
            print(f"  ✗ {description} failed: {module_name} - {e}", flush=True)
            return 1

    # Layer 3: ROS2 environment
    print("\n[Layer 3] ROS2 environment...", flush=True)
    import os
    ros_domain_id = os.environ.get("ROS_DOMAIN_ID", "not set")
    ros_localhost_only = os.environ.get("ROS_LOCALHOST_ONLY", "not set")
    print(f"  - ROS_DOMAIN_ID: {ros_domain_id}", flush=True)
    print(f"  - ROS_LOCALHOST_ONLY: {ros_localhost_only}", flush=True)

    try:
        import rclpy
        print(f"  ✓ rclpy available", flush=True)
    except Exception as e:
        print(f"  ✗ rclpy import failed: {e}", flush=True)
        return 1

    # Layer 4: G1 SDK availability
    print("\n[Layer 4] G1 SDK availability...", flush=True)
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        print(f"  ✓ unitree_sdk2py available", flush=True)
    except Exception as e:
        print(f"  ✗ unitree_sdk2py import failed: {e}", flush=True)
        print(f"    Note: This is expected in mock mode, but required for real G1", flush=True)
        if config.output.mode == "g1":
            return 1

    # Layer 5: Network interface check
    print("\n[Layer 5] Network interface...", flush=True)
    import subprocess
    try:
        result = subprocess.run(
            ["ip", "link", "show", config.output.network_interface],
            capture_output=True,
            text=True,
            timeout=2,
        )
        if result.returncode == 0:
            print(f"  ✓ Interface {config.output.network_interface} exists", flush=True)
        else:
            print(f"  ✗ Interface {config.output.network_interface} not found", flush=True)
            print(f"    Error: {result.stderr.strip()}", flush=True)
            if config.output.mode == "g1":
                return 1
    except Exception as e:
        print(f"  ✗ Interface check failed: {e}", flush=True)

    # Layer 6: Serial port check
    print("\n[Layer 6] Serial port (ALX)...", flush=True)
    import os.path
    if os.path.exists(config.alx.port):
        print(f"  ✓ {config.alx.port} exists", flush=True)
    else:
        print(f"  ✗ {config.alx.port} not found", flush=True)
        print(f"    Note: ALX device may not be connected yet", flush=True)

    # Layer 7: Quick runtime init test
    print("\n[Layer 7] Runtime initialization test...", flush=True)
    print("  Testing ALXLiveSource init...", flush=True)
    try:
        from robot_follow.real.alx_source import ALXLiveSource
        alx = ALXLiveSource(port=config.alx.port, baudrate=config.alx.baudrate)
        alx.start()
        time.sleep(0.5)  # Brief wait for thread to start
        snapshot = alx.snapshot()
        alx.stop()
        if snapshot is None:
            print(f"  ⚠ ALX started but no data (expected if device not connected)", flush=True)
        else:
            print(f"  ✓ ALX snapshot: distance={snapshot.distance_m:.2f}m, azimuth={snapshot.azimuth_deg:.1f}°", flush=True)
            # Check wristband state
            wristband = alx.wristband_state()
            print(f"    - Wristband state: {wristband.enabled}", flush=True)
    except Exception as e:
        print(f"  ✗ ALX init failed: {e}", flush=True)

    print("\n" + "=" * 70, flush=True)
    print("DIAGNOSTIC COMPLETE", flush=True)
    print("=" * 70, flush=True)

    return 0

if __name__ == "__main__":
    sys.exit(main())