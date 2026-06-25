#!/usr/bin/env bash
set -eo pipefail

WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"
NORTH_OFFSET_M="${NORTH_OFFSET_M:-2.0}"
EAST_OFFSET_M="${EAST_OFFSET_M:-0.0}"
GOAL_ALT_M="${GOAL_ALT_M:-1.5}"
YAW_DEG="${YAW_DEG:-0.0}"

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

TMP_YAML="$(mktemp)"
trap 'rm -f "${TMP_YAML}"' EXIT

timeout 5 ros2 topic echo /fmu/out/vehicle_local_position_v1 --once > "${TMP_YAML}"

python3 - "$TMP_YAML" "$NORTH_OFFSET_M" "$EAST_OFFSET_M" "$GOAL_ALT_M" "$YAW_DEG" <<'PY'
import math
import re
import sys

path, north_m, east_m, goal_alt_m, yaw_deg = sys.argv[1:]
north_m = float(north_m)
east_m = float(east_m)
goal_alt_m = float(goal_alt_m)
yaw_deg = float(yaw_deg)

text = open(path, "r", encoding="utf-8").read()

def extract(name: str) -> float:
    m = re.search(rf"^{name}:\s*([-+0-9.eE]+)", text, re.MULTILINE)
    if not m:
        raise RuntimeError(f"Missing field: {name}")
    return float(m.group(1))

xy_global_match = re.search(r"^xy_global:\s*(true|false)", text, re.MULTILINE)
if not xy_global_match or xy_global_match.group(1) != "true":
    raise RuntimeError("VehicleLocalPosition has no valid global XY reference yet.")

ref_lat = math.radians(extract("ref_lat"))
ref_lon = math.radians(extract("ref_lon"))
earth_radius_m = 6378137.0

target_lat = ref_lat + (north_m / earth_radius_m)
target_lon = ref_lon + (east_m / (earth_radius_m * math.cos(ref_lat)))

print(f"{math.degrees(target_lat):.8f} {math.degrees(target_lon):.8f} {goal_alt_m:.3f} {yaw_deg:.3f}")
PY

read -r TARGET_LAT TARGET_LON TARGET_ALT TARGET_YAW <<<"$(python3 - "$TMP_YAML" "$NORTH_OFFSET_M" "$EAST_OFFSET_M" "$GOAL_ALT_M" "$YAW_DEG" <<'PY'
import math
import re
import sys

path, north_m, east_m, goal_alt_m, yaw_deg = sys.argv[1:]
north_m = float(north_m)
east_m = float(east_m)
goal_alt_m = float(goal_alt_m)
yaw_deg = float(yaw_deg)

text = open(path, "r", encoding="utf-8").read()

def extract(name: str) -> float:
    m = re.search(rf"^{name}:\s*([-+0-9.eE]+)", text, re.MULTILINE)
    if not m:
        raise RuntimeError(f"Missing field: {name}")
    return float(m.group(1))

xy_global_match = re.search(r"^xy_global:\s*(true|false)", text, re.MULTILINE)
if not xy_global_match or xy_global_match.group(1) != "true":
    raise RuntimeError("VehicleLocalPosition has no valid global XY reference yet.")

ref_lat = math.radians(extract("ref_lat"))
ref_lon = math.radians(extract("ref_lon"))
earth_radius_m = 6378137.0

target_lat = ref_lat + (north_m / earth_radius_m)
target_lon = ref_lon + (east_m / (earth_radius_m * math.cos(ref_lat)))

print(f"{math.degrees(target_lat):.8f} {math.degrees(target_lon):.8f} {goal_alt_m:.3f} {yaw_deg:.3f}")
PY
)"

echo "[INFO] Simulating QGC DO_REPOSITION"
echo "  north_offset_m=${NORTH_OFFSET_M}"
echo "  east_offset_m=${EAST_OFFSET_M}"
echo "  target_lat=${TARGET_LAT}"
echo "  target_lon=${TARGET_LON}"
echo "  goal_alt_m=${TARGET_ALT}"
echo "  yaw_deg=${TARGET_YAW}"

ros2 topic pub --once /fmu/out/vehicle_command px4_msgs/msg/VehicleCommand \
  "{timestamp: 0, command: 192, param1: 1.0, param2: 0.0, param3: 0.0, param4: ${TARGET_YAW}, param5: ${TARGET_LAT}, param6: ${TARGET_LON}, param7: ${TARGET_ALT}, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}"
