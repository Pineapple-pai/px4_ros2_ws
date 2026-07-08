#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
REPOSITION_WAIT_S="${REPOSITION_WAIT_S:-6}"
RTL_SETTLE_WAIT_S="${RTL_SETTLE_WAIT_S:-12}"
NORTH_OFFSET_M="${NORTH_OFFSET_M:-2.5}"
EAST_OFFSET_M="${EAST_OFFSET_M:-0.0}"
GOAL_ALT_M="${GOAL_ALT_M:-1.5}"
YAW_DEG="${YAW_DEG:-0.0}"

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

TMP_BEFORE="$(mktemp)"
TMP_AFTER="$(mktemp)"
cleanup() {
  rm -f "${TMP_BEFORE}" "${TMP_AFTER}"
}
trap cleanup EXIT

echo "[INFO] Step 1/5: capture initial VehicleStatus"
timeout 15s ros2 topic echo /fmu/out/vehicle_status_v4 --once > "${TMP_BEFORE}"
sed -n '1,80p' "${TMP_BEFORE}"

echo
echo "[INFO] Step 2/5: simulate a QGC reposition target"
NORTH_OFFSET_M="${NORTH_OFFSET_M}" \
EAST_OFFSET_M="${EAST_OFFSET_M}" \
GOAL_ALT_M="${GOAL_ALT_M}" \
YAW_DEG="${YAW_DEG}" \
  "${WS_DIR}/scripts/simulate_qgc_goto.sh"

echo
echo "[INFO] Step 3/5: wait ${REPOSITION_WAIT_S}s for goal protection / planner activation"
sleep "${REPOSITION_WAIT_S}"

echo
echo "[INFO] Step 4/5: send RTL command"
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  '{timestamp: 0, command: 20, param1: 0.0, param2: 0.0, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}'

echo
echo "[INFO] Step 5/5: wait ${RTL_SETTLE_WAIT_S}s, then capture VehicleStatus again"
sleep "${RTL_SETTLE_WAIT_S}"
timeout 15s ros2 topic echo /fmu/out/vehicle_status_v4 --once > "${TMP_AFTER}"
sed -n '1,80p' "${TMP_AFTER}"

echo
echo "[INFO] Evaluating RTL takeover result"
"${PYTHON_BIN}" - "${TMP_BEFORE}" "${TMP_AFTER}" <<'PY'
import re
import sys

before_path, after_path = sys.argv[1:]

def parse(path: str):
    text = open(path, "r", encoding="utf-8").read()
    def get_int(name: str):
        m = re.search(rf"^{name}:\s*([-+]?\d+)", text, re.MULTILINE)
        return int(m.group(1)) if m else None
    def get_bool(name: str):
        m = re.search(rf"^{name}:\s*(true|false)", text, re.MULTILINE)
        return m.group(1) == "true" if m else None
    return {
        "nav_state": get_int("nav_state"),
        "nav_state_user_intention": get_int("nav_state_user_intention"),
        "failsafe": get_bool("failsafe"),
        "accepts_offboard_setpoints": get_bool("accepts_offboard_setpoints"),
        "gcs_connection_lost": get_bool("gcs_connection_lost"),
    }

before = parse(before_path)
after = parse(after_path)

left_ros2_external = after["nav_state"] not in (23,)
offboard_not_accepted = after["accepts_offboard_setpoints"] is False
not_stuck_in_offboard = after["nav_state"] != 14

print("RTL_TAKEOVER_SUMMARY")
print(f"before_nav_state {before['nav_state']}")
print(f"after_nav_state {after['nav_state']}")
print(f"after_nav_state_user_intention {after['nav_state_user_intention']}")
print(f"after_failsafe {after['failsafe']}")
print(f"after_accepts_offboard_setpoints {after['accepts_offboard_setpoints']}")
print(f"after_gcs_connection_lost {after['gcs_connection_lost']}")
print(f"left_ros2_external {left_ros2_external}")
print(f"not_stuck_in_offboard {not_stuck_in_offboard}")
print(f"offboard_not_accepted {offboard_not_accepted}")
print(f"PASS_RTL_TAKEOVER {left_ros2_external and not_stuck_in_offboard and offboard_not_accepted}")
PY