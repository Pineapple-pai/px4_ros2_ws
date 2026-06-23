#!/usr/bin/env bash
# Wrapper script: reads a Gazebo Classic topic using gz topic -e
#
# Usage: gz_topic_reader.sh <topic_name>
#
# CRITICAL: Must run in a clean environment stripped of ROS2 libs,
# because ROS2 protobuf conflicts with Gazebo Classic protobuf,
# causing "gz topic -e" to silently produce zero output.

set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 <topic_name>" >&2
  exit 1
fi

TOPIC="$1"
shift

# Step 1: Subscribe in a clean env to avoid ROS2 protobuf interference.
# PX4's gazebo_lidar_plugin publishes Range messages via transport::Node;
# these topics do NOT appear as publishers under gz topic -i, so we skip
# the publisher check and just attempt to read.
# Gazebo Classic uses ``gz topic -e <topic>``.  ``-t`` is not a topic option
# here and makes echo exit immediately without data on Gazebo 11.
exec /usr/bin/env -i \
  PATH="/usr/local/bin:/usr/bin:/bin" \
  HOME="$HOME" \
  USER="$USER" \
  GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://localhost:11345}" \
  GAZEBO_MODEL_DATABASE_URI="http://models.gazebosim.org" \
  GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11" \
  GAZEBO_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins" \
  GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:${HOME}/.gazebo/models" \
  LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu" \
  /usr/bin/gz topic -e "${TOPIC}"
