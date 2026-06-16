#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

CONDA_ENV="${CONDA_ENV:-robot-follow}"
USE_CONDA="${USE_CONDA:-1}"

TEST_ENV="${TEST_ENV:-jetson}"
TEST_TYPE="${TEST_TYPE:-closed_loop_low_speed}"
LOGS_ROOT="${LOGS_ROOT:-logs}"
CONFIG="${CONFIG:-configs/jetson_g1_safe.yaml}"
DURATION="${DURATION:-}"

if [[ "${USE_CONDA}" == "1" ]] && command -v conda >/dev/null 2>&1; then
  PYTHON_CMD=(conda run -n "${CONDA_ENV}" python)
else
  PYTHON_CMD=("${PYTHON_BIN:-python}")
fi

echo "Running logged Robot_Follow test"
echo "repo=${REPO_ROOT}"
echo "test_env=${TEST_ENV}"
echo "test_type=${TEST_TYPE}"
echo "logs_root=${LOGS_ROOT}"
echo "config=${CONFIG}"
if [[ -n "${DURATION}" ]]; then
  echo "duration=${DURATION}"
else
  echo "duration=until Ctrl+C"
fi
echo
echo "WARNING: this command enables real robot motion."
echo

ROBOT_ARGS=(
  --config "${CONFIG}"
  --enable-motion
  --i-understand-this-moves-robot
)

if [[ -n "${DURATION}" ]]; then
  ROBOT_ARGS+=(--duration "${DURATION}")
fi

exec "${PYTHON_CMD[@]}" -m robot_follow.run_logger \
  --test-env "${TEST_ENV}" \
  --test-type "${TEST_TYPE}" \
  --logs-root "${LOGS_ROOT}" \
  -- \
  "${ROBOT_ARGS[@]}" \
  "$@"
