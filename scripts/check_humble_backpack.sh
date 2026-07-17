#!/usr/bin/env bash
# Read-only preflight for a ROS 2 Humble backpack.  It deliberately does not
# start ROS nodes, open the ALX serial port, or initialise the Unitree SDK.
set -euo pipefail

failed=0

check() {
  local description="$1"
  shift
  if "$@"; then
    printf 'PASS  %s\n' "$description"
  else
    printf 'FAIL  %s\n' "$description" >&2
    failed=1
  fi
}

printf 'Robot Follow ROS 2 Humble preflight\n'
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-<unset>}"
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-<default>}"
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-0}"
printf 'python3=%s\n' "$(command -v python3 || true)"
python3 --version || failed=1

check 'Ubuntu 22.04 is installed' bash -c '. /etc/os-release && [ "$VERSION_ID" = "22.04" ]'
check 'ROS 2 Humble is sourced' test "${ROS_DISTRO:-}" = humble
check 'Humble setup file exists' test -r /opt/ros/humble/setup.bash
check 'ROS 2 CLI is available' command -v ros2
check 'ROS Python client is importable by python3' python3 -c 'import rclpy'
check 'PointCloud2 helpers are importable by python3' python3 -c 'from sensor_msgs.msg import PointCloud2; from sensor_msgs_py import point_cloud2'

if [[ -n "${FAST_LIO_WS:-}" ]]; then
  check 'FAST_LIO workspace install setup exists' test -r "$FAST_LIO_WS/install/setup.bash"
else
  printf 'WARN  FAST_LIO_WS is unset; set it before running the follow service.\n' >&2
fi

if [[ -n "${FAST_LIO_CONFIG:-}" ]]; then
  check 'FAST_LIO configuration exists' test -r "$FAST_LIO_CONFIG"
else
  printf 'WARN  FAST_LIO_CONFIG is unset; set it before running the follow service.\n' >&2
fi

if [[ -e /dev/ttyUSB0 ]]; then
  printf 'PASS  configured ALX device exists: /dev/ttyUSB0\n'
else
  printf 'WARN  /dev/ttyUSB0 is absent; connect ALX before hardware tests.\n' >&2
fi

if [[ "${RMW_IMPLEMENTATION:-}" == "rmw_cyclonedds_cpp" ]]; then
  printf 'WARN  ROS uses CycloneDDS. Qualify coexistence with Unitree cyclonedds==0.10.2 before real G1 motion.\n' >&2
fi

exit "$failed"
