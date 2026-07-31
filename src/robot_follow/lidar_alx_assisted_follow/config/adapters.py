"""Configuration adapters to existing types."""

from __future__ import annotations

from robot_follow.ao.config import AORadarServiceConfig
from robot_follow.real.g1.types import G1BridgeConfig

from .schema import PrototypeConfig


def build_radar_service_config(config: PrototypeConfig) -> AORadarServiceConfig:
    """Build AORadarServiceConfig from prototype configuration.

    Parameters
    ----------
    config : PrototypeConfig
        Prototype configuration.

    Returns
    -------
    AORadarServiceConfig
        Radar service configuration for existing infrastructure.
    """
    return AORadarServiceConfig(
        idle_stop_s=5.0,  # Not used by prototype
        static_tf_start_command=config.sensors.static_tf_start_command,
        livox_start_command=config.sensors.livox_start_command,
        fast_lio_start_command=config.sensors.fast_lio_start_command,
        stop_timeout_s=config.sensors.stop_timeout_s,
        ready_timeout_s=config.sensors.ready_timeout_s,
    )


def build_g1_bridge_config(config: PrototypeConfig) -> G1BridgeConfig:
    """Build G1BridgeConfig from prototype configuration.

    Parameters
    ----------
    config : PrototypeConfig
        Prototype configuration.

    Returns
    -------
    G1BridgeConfig
        G1 bridge configuration for existing infrastructure.
    """
    return G1BridgeConfig(
        max_abs_vx=config.g1.max_vx,
        max_abs_vy=config.g1.max_abs_vy,
        max_abs_omega=config.g1.max_abs_omega,
        yaw_sign=config.g1.yaw_sign,
        sdk_timeout_s=config.g1.sdk_timeout_s,
        sdk_timeout_code=config.g1.sdk_timeout_code,
        sdk_timeout_retry_interval_s=config.g1.sdk_timeout_retry_interval_s,
        max_consecutive_sdk_timeouts=config.g1.max_consecutive_sdk_timeouts,
        command_timeout_s=config.g1.command_timeout_s,
    )
