#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"

cd "${WS_DIR}"

chmod +x \
  scripts/check_planning_runtime.sh \
  scripts/collect_planning_diagnostics.sh

echo "[INFO] Step 1/2: Runtime smoke check"
./scripts/check_planning_runtime.sh

echo
echo "[INFO] Step 2/2: Collect diagnostics"
./scripts/collect_planning_diagnostics.sh

echo
echo "[INFO] Post-start checks complete."
echo "[INFO] If the system still does not behave as expected, share the latest folder under:"
echo "  ${WS_DIR}/log/planning_diagnostics"
