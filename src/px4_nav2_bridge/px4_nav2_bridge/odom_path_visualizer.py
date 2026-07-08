import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class OdomPathVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("odom_path_visualizer")

        self.declare_parameter("input_odom_topic", "/autonomy/lio_odometry")
        self.declare_parameter("path_topic", "/drone_0_vis/path")
        self.declare_parameter("path_frame_id", "world")
        self.declare_parameter("max_path_length", 2000)
        self.declare_parameter("min_pose_separation_m", 0.05)
        self.declare_parameter("min_pose_separation_yaw_rad", 0.10)

        input_odom_topic = self.get_parameter("input_odom_topic").get_parameter_value().string_value
        path_topic = self.get_parameter("path_topic").get_parameter_value().string_value
        self.path_frame_id = self.get_parameter("path_frame_id").get_parameter_value().string_value
        self.max_path_length = max(
            10,
            self.get_parameter("max_path_length").get_parameter_value().integer_value,
        )
        self.min_pose_separation_m = self.get_parameter(
            "min_pose_separation_m"
        ).get_parameter_value().double_value
        self.min_pose_separation_yaw_rad = self.get_parameter(
            "min_pose_separation_yaw_rad"
        ).get_parameter_value().double_value

        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, input_odom_topic, self.odom_callback, 10)
        self.poses: deque[PoseStamped] = deque(maxlen=self.max_path_length)

        self.get_logger().info(
            f"Accumulating path from {input_odom_topic} -> {path_topic} in frame {self.path_frame_id}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self.path_frame_id or msg.header.frame_id
        pose.pose = msg.pose.pose

        if self.poses and not self._should_append(pose):
            self._publish(msg.header.stamp)
            return

        self.poses.append(pose)
        self._publish(msg.header.stamp)

    def _should_append(self, pose: PoseStamped) -> bool:
        last_pose = self.poses[-1].pose
        dx = pose.pose.position.x - last_pose.position.x
        dy = pose.pose.position.y - last_pose.position.y
        dz = pose.pose.position.z - last_pose.position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance >= self.min_pose_separation_m:
            return True

        last_yaw = self._yaw_from_quaternion(last_pose.orientation)
        yaw = self._yaw_from_quaternion(pose.pose.orientation)
        yaw_delta = math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
        return abs(yaw_delta) >= self.min_pose_separation_yaw_rad

    @staticmethod
    def _yaw_from_quaternion(orientation) -> float:
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _publish(self, stamp) -> None:
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.path_frame_id
        path.poses = list(self.poses)
        self.path_pub.publish(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomPathVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()