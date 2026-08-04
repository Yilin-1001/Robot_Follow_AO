#!/usr/bin/env bash
# Start the production LiDAR + ALX follow application in the qualified Humble
# Conda runtime.  The CLI options below intentionally match the approved
# operator command for the G1 + MID-360 backpack.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"

ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
fast_lio_ws="${FAST_LIO_WS:-/home/fk/g1_mid360_ros2_ws}"
fast_lio_config="${FAST_LIO_CONFIG:-$fast_lio_ws/src/FAST_LIO/config/mid360.yaml}"
conda_env="${ROBOT_FOLLOW_CONDA_ENV:-robot_followao_radar_humble}"
config_path="${ROBOT_FOLLOW_CONFIG:-$repo_root/configs/jetson_g1_lidar_alx_assisted_follow_production.yaml}"

[[ -r "$ros_setup" ]] || { echo "ROS setup not found: $ros_setup" >&2; exit 2; }
[[ -r "$fast_lio_ws/install/setup.bash" ]] || { echo "FAST-LIO setup not found: $fast_lio_ws/install/setup.bash" >&2; exit 2; }
command -v conda >/dev/null || { echo "conda is required to run $conda_env" >&2; exit 2; }
[[ -r "$fast_lio_config" ]] || { echo "FAST-LIO config not found: $fast_lio_config" >&2; exit 2; }

source "$ros_setup"
source "$fast_lio_ws/install/setup.bash"

# Keep the ROS graph distinct from Unitree's SDK, which initialises DDS domain
# 0 internally.  Fast DDS avoids loading ROS's CycloneDDS alongside Unitree's
# required Python cyclonedds==0.10.2 in the same process.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FAST_LIO_CONFIG="$fast_lio_config"
# The installed Humble FAST-LIO launch file accepts ``config_path`` plus
# ``config_file`` rather than a single full configuration path.
export FAST_LIO_CONFIG_DIR="$(dirname "$fast_lio_config")"
export FAST_LIO_CONFIG_FILE="$(basename "$fast_lio_config")"
export CUPY_CACHE_DIR="${CUPY_CACHE_DIR:-/tmp/robot-follow-cupy-cache}"
export PYTHONPATH="$repo_root/src${PYTHONPATH:+:$PYTHONPATH}"

cd "$repo_root"
exec conda run --no-capture-output -n "$conda_env" \
  python -m robot_follow.lidar_alx_assisted_follow.cli \
  "$config_path" \
  --confirm-real-g1 \
  --allow-provisional-calibration \
  --host 0.0.0.0 \
  --port 8080 \
  --allow-unauthenticated-lan \
  --log-level DEBUG \
  "$@"
