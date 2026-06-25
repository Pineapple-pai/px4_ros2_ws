import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


EARTH_RADIUS_M = 6378137.0


def _yaw_to_quaternion(yaw_rad: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))


class QgcRepositionGoalBridge(Node):
    def __init__(self) -> None:
        super().__init__("qgc_reposition_goal_bridge")

        self.declare_parameter("vehicle_command_topic", "/fmu/out/vehicle_command")
        self.declare_parameter("vehicle_command_in_topic", "/fmu/in/vehicle_command")
        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status_v4")
        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter("goal_pose_topic", "/autonomy/qgc_goal_pose")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("fixed_goal_altitude_m", 0.0)
        self.declare_parameter("use_qgc_altitude", False)
        self.declare_parameter("require_external_command", True)
        self.declare_parameter("auto_request_ros2_mode", True)
        self.declare_parameter("ros2_nav_state", int(VehicleStatus.NAVIGATION_STATE_EXTERNAL1))
        self.declare_parameter("require_armed_for_nav2_goal", True)
        self.declare_parameter("mode_request_period_s", 0.25)
        self.declare_parameter("mode_request_hold_s", 30.0)
        self.declare_parameter("goal_dedup_distance_m", 0.25)
        self.declare_parameter("nav2_server_timeout_s", 2.0)
        self.declare_parameter("enable_nav2_action", True)

        command_topic = self.get_parameter("vehicle_command_topic").value
        command_in_topic = self.get_parameter("vehicle_command_in_topic").value
        local_position_topic = self.get_parameter("vehicle_local_position_topic").value
        status_topic = self.get_parameter("vehicle_status_topic").value
        action_name = self.get_parameter("nav2_action_name").value
        self._goal_pose_topic = self.get_parameter("goal_pose_topic").value
        self._map_frame_id = self.get_parameter("map_frame_id").value
        self._fixed_goal_altitude_m = float(self.get_parameter("fixed_goal_altitude_m").value)
        self._use_qgc_altitude = bool(self.get_parameter("use_qgc_altitude").value)
        self._require_external_command = bool(self.get_parameter("require_external_command").value)
        self._auto_request_ros2_mode = bool(self.get_parameter("auto_request_ros2_mode").value)
        self._ros2_nav_state = int(self.get_parameter("ros2_nav_state").value)
        self._require_armed_for_nav2_goal = bool(self.get_parameter("require_armed_for_nav2_goal").value)
        self._mode_request_period_s = float(self.get_parameter("mode_request_period_s").value)
        self._mode_request_hold_s = float(self.get_parameter("mode_request_hold_s").value)
        self._goal_dedup_distance_m = float(self.get_parameter("goal_dedup_distance_m").value)
        self._nav2_server_timeout_s = float(self.get_parameter("nav2_server_timeout_s").value)
        self._enable_nav2_action = bool(self.get_parameter("enable_nav2_action").value)

        self._latest_local_position: Optional[VehicleLocalPosition] = None
        self._latest_vehicle_status: Optional[VehicleStatus] = None
        self._last_goal_xy: Optional[Tuple[float, float]] = None
        self._nav2_goal_active = False
        self._last_goal_request_time = self.get_clock().now()

        self._goal_pub = self.create_publisher(PoseStamped, self._goal_pose_topic, 10)
        self._vehicle_command_pub = self.create_publisher(VehicleCommand, command_in_topic, 10)
        self._nav2_client = ActionClient(self, NavigateToPose, action_name) if self._enable_nav2_action else None

        self.create_subscription(
            VehicleLocalPosition,
            local_position_topic,
            self._handle_local_position,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleStatus,
            status_topic,
            self._handle_vehicle_status,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleCommand,
            command_topic,
            self._handle_vehicle_command,
            qos_profile_sensor_data,
        )
        timer_period_s = max(0.1, self._mode_request_period_s)
        self.create_timer(timer_period_s, self._enforce_ros2_mode_while_goal_active)

        self.get_logger().info(
            f"Listening for QGC reposition commands on {command_topic}; "
            f"requesting nav_state={self._ros2_nav_state} through {command_in_topic}; "
            f"publishing planner goals on {self._goal_pose_topic} "
            f"with Nav2 action {'enabled' if self._enable_nav2_action else 'disabled'}."
        )

    def _handle_local_position(self, msg: VehicleLocalPosition) -> None:
        self._latest_local_position = msg

    def _handle_vehicle_status(self, msg: VehicleStatus) -> None:
        self._latest_vehicle_status = msg

    def _handle_vehicle_command(self, msg: VehicleCommand) -> None:
        if msg.command != VehicleCommand.VEHICLE_CMD_DO_REPOSITION:
            return
        if self._require_external_command and not msg.from_external:
            return
        if not math.isfinite(msg.param5) or not math.isfinite(msg.param6):
            self.get_logger().warn("Ignoring QGC reposition command with invalid latitude/longitude.")
            return

        pose = self._make_goal_pose(msg)
        if pose is None:
            return

        if not self._vehicle_ready_for_nav2_goal():
            self._goal_pub.publish(pose)
            return

        goal_xy = (pose.pose.position.x, pose.pose.position.y)
        if self._last_goal_xy is not None:
            dx = goal_xy[0] - self._last_goal_xy[0]
            dy = goal_xy[1] - self._last_goal_xy[1]
            if math.hypot(dx, dy) < self._goal_dedup_distance_m:
                return

        self._last_goal_xy = goal_xy
        self._goal_pub.publish(pose)
        self._last_goal_request_time = self.get_clock().now()
        self._nav2_goal_active = True
        self._request_ros2_mode_if_needed(force=True)
        if self._enable_nav2_action:
            self._send_nav2_goal(pose)
        else:
            self.get_logger().info(
                "Forwarded QGC reposition target to planner topic only: "
                f"map x={pose.pose.position.x:.2f} "
                f"y={pose.pose.position.y:.2f} "
                f"z={pose.pose.position.z:.2f}"
            )

    def _vehicle_ready_for_nav2_goal(self) -> bool:
        status = self._latest_vehicle_status
        if status is None:
            self.get_logger().warn(
                "QGC reposition target converted, but no VehicleStatus received yet. "
                "Debug goal published; Nav2 goal withheld to avoid an unguarded native GoTo."
            )
            return False

        if status.failsafe:
            self.get_logger().warn("Ignoring QGC reposition while PX4 reports failsafe=true.")
            return False

        if self._require_armed_for_nav2_goal and status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().warn("Ignoring QGC reposition while vehicle is not armed.")
            return False

        return True

    def _request_ros2_mode_if_needed(self, force: bool = False) -> None:
        status = self._latest_vehicle_status
        if not self._auto_request_ros2_mode:
            return
        if not force and status is not None and status.nav_state == self._ros2_nav_state:
            return

        command = VehicleCommand()
        command.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        command.command = VehicleCommand.VEHICLE_CMD_SET_NAV_STATE
        command.param1 = float(self._ros2_nav_state)
        command.target_system = 1
        command.target_component = 1
        command.source_system = 1
        command.source_component = 1
        command.from_external = True
        self._vehicle_command_pub.publish(command)

        current_nav_state = "unknown" if status is None else str(status.nav_state)
        self.get_logger().warn(
            "QGC Go-to is being routed through Nav2; requested ROS2 external mode "
            f"nav_state={self._ros2_nav_state} from current nav_state={current_nav_state}."
        )

    def _enforce_ros2_mode_while_goal_active(self) -> None:
        if not self._auto_request_ros2_mode:
            return

        status = self._latest_vehicle_status
        if status is None:
            return

        now = self.get_clock().now()
        goal_age_s = (now - self._last_goal_request_time).nanoseconds / 1e9
        protect_active = self._nav2_goal_active or goal_age_s <= self._mode_request_hold_s
        if not protect_active:
            return

        if status.failsafe:
            return
        if self._require_armed_for_nav2_goal and status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            return

        if status.nav_state != self._ros2_nav_state:
            self._request_ros2_mode_if_needed(force=True)

    def _make_goal_pose(self, msg: VehicleCommand) -> Optional[PoseStamped]:
        local_position = self._latest_local_position
        if local_position is None:
            self.get_logger().warn("Cannot convert QGC target yet: no VehicleLocalPosition reference received.")
            return None
        if not local_position.xy_global:
            self.get_logger().warn("Cannot convert QGC target: PX4 local position has no global XY reference.")
            return None

        ref_lat = math.radians(local_position.ref_lat)
        target_lat = math.radians(msg.param5)
        target_lon = math.radians(msg.param6)
        ref_lon = math.radians(local_position.ref_lon)

        north_m = (target_lat - ref_lat) * EARTH_RADIUS_M
        east_m = (target_lon - ref_lon) * EARTH_RADIUS_M * math.cos(ref_lat)
        altitude_m = float(msg.param7) if self._use_qgc_altitude and math.isfinite(msg.param7) else self._fixed_goal_altitude_m

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._map_frame_id
        pose.pose.position.x = north_m
        pose.pose.position.y = east_m
        pose.pose.position.z = altitude_m

        yaw_rad = math.radians(float(msg.param4)) if math.isfinite(msg.param4) else 0.0
        qx, qy, qz, qw = _yaw_to_quaternion(yaw_rad)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _send_nav2_goal(self, pose: PoseStamped) -> None:
        if self._nav2_client is None:
            return
        if not self._nav2_client.wait_for_server(timeout_sec=self._nav2_server_timeout_s):
            self.get_logger().warn(
                f"Nav2 action server is not available; debug goal still published on {self._goal_pose_topic}."
            )
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        future = self._nav2_client.send_goal_async(goal_msg)
        future.add_done_callback(self._handle_goal_response)

        self.get_logger().info(
            "Sent QGC reposition target to Nav2: "
            f"map x={pose.pose.position.x:.2f} "
            f"y={pose.pose.position.y:.2f} "
            f"z={pose.pose.position.z:.2f}"
        )

    def _handle_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._nav2_goal_active = False
            self.get_logger().warn("Nav2 rejected the QGC reposition goal.")
            return
        self.get_logger().info("Nav2 accepted the QGC reposition goal.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_goal_result)

    def _handle_goal_result(self, future) -> None:
        self._nav2_goal_active = False
        try:
            result = future.result()
            status = getattr(result, "status", "unknown")
            self.get_logger().info(f"Nav2 QGC goal finished with action status={status}.")
        except Exception as exc:  # noqa: BLE001 - keep the bridge alive after action shutdowns
            self.get_logger().warn(f"Nav2 goal result unavailable: {exc}")


def main() -> None:
    rclpy.init()
    node = QgcRepositionGoalBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
