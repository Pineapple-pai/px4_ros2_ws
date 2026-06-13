#!/usr/bin/env bash
set -euo pipefail
echo "Waiting a few seconds for PX4 MAVLink to come up before starting QGC..."
sleep 8
exec "/home/p/px4_ros2_ws/.runtime/start_qgc.sh"
