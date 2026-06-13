set +u
source /opt/ros/humble/setup.bash
source "/home/p/px4_ros2_ws/install/setup.bash"
set -u
export LD_LIBRARY_PATH="/home/p/px4_ros2_ws/local/livox_sdk2/lib:${LD_LIBRARY_PATH:-}"
