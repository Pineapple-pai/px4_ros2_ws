#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
VALIDATION_MODE="${VALIDATION_MODE:-chain}"

cd "${WS_DIR}"

echo "[INFO] Step 1/4: offline logic self-test"
"${PYTHON_BIN}" "${WS_DIR}/scripts/validate_fastlio_ego_avoidance.py" --self-test

restore_nounset=0
if [[ $- == *u* ]]; then
  restore_nounset=1
  set +u
fi
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
if [[ "${restore_nounset}" == "1" ]]; then
  set -u
fi

echo
echo "[INFO] Step 2/4: runtime topic/node check"
"${WS_DIR}/scripts/check_planning_runtime.sh"

echo
echo "[INFO] Step 3/4: planning-chain validation"
"${PYTHON_BIN}" "${WS_DIR}/scripts/validate_fastlio_ego_avoidance.py" --require-chain --no-arm

if [[ "${VALIDATION_MODE}" == "chain" ]]; then
  echo
  echo "[INFO] Chain validation finished. Export VALIDATION_MODE=flight to continue with obstacle-crossing flight test."
  exit 0
fi

echo
echo "[INFO] Step 4/4: obstacle-crossing flight validation"
"${PYTHON_BIN}" "${WS_DIR}/scripts/validate_fastlio_ego_avoidance.py" --require-chain
