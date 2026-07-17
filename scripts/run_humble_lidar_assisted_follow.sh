#!/usr/bin/env bash
# Start the LiDAR + ALX follow application in the qualified Humble runtime.
# Real G1 motion still needs the CLI confirmations enforced by the application.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"

ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
fast_lio_ws="${FAST_LIO_WS:?FAST_LIO_WS must point to the Humble FAST-LIO workspace}"
fast_lio_config="${FAST_LIO_CONFIG:?FAST_LIO_CONFIG must point to the Humble MID-360 config}"
venv_dir="${ROBOT_FOLLOW_VENV:-$repo_root/.venv-humble}"
config_path="${ROBOT_FOLLOW_CONFIG:-$repo_root/configs/jetson_g1_lidar_alx_assisted_follow.yaml}"

[[ -r "$ros_setup" ]] || { echo "ROS setup not found: $ros_setup" >&2; exit 2; }
[[ -r "$fast_lio_ws/install/setup.bash" ]] || { echo "FAST-LIO setup not found: $fast_lio_ws/install/setup.bash" >&2; exit 2; }
[[ -x "$venv_dir/bin/python" ]] || { echo "Humble venv Python not found: $venv_dir/bin/python" >&2; exit 2; }
[[ -r "$fast_lio_config" ]] || { echo "FAST-LIO config not found: $fast_lio_config" >&2; exit 2; }

source "$ros_setup"
source "$fast_lio_ws/install/setup.bash"

# Keep the ROS graph distinct from Unitree's SDK, which initialises DDS domain
# 0 internally.  Fast DDS avoids loading ROS's CycloneDDS alongside Unitree's
# required Python cyclonedds==0.10.2 in the same process.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FAST_LIO_CONFIG="$fast_lio_config"
# The installed Humble FAST-LIO launch file accepts ``config_path`` plus
# ``config_file`` rather than a single full configuration path.
export FAST_LIO_CONFIG_DIR="$(dirname "$fast_lio_config")"
export FAST_LIO_CONFIG_FILE="$(basename "$fast_lio_config")"

cd "$repo_root"
exec "$venv_dir/bin/python" -m robot_follow.lidar_alx_assisted_follow.cli \
  "$config_path" "$@"
