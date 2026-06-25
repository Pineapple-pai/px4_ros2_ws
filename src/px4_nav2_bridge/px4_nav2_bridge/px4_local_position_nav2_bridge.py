import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import tf2_ros


def _yaw_to_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))


class Px4LocalPositionNav2Bridge(Node):
    def __init__(self) -> None:
        super().__init__("px4_local_position_nav2_bridge")

        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("project_to_2d", True)

        local_position_topic = self.get_parameter("vehicle_local_position_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self._map_frame_id = self.get_parameter("map_frame_id").value
        self._odom_frame_id = self.get_parameter("odom_frame_id").value
        self._base_frame_id = self.get_parameter("base_frame_id").value
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._project_to_2d = bool(self.get_parameter("project_to_2d").value)

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_map_to_odom()

        self.create_subscription(
            VehicleLocalPosition,
            local_position_topic,
            self._handle_position,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"PX4 local position -> Nav2 odom: {local_position_topic} -> {odom_topic}"
        )

    def _publish_static_map_to_odom(self) -> None:
        if not self._publish_tf:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._map_frame_id
        transform.child_frame_id = self._odom_frame_id
        transform.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform(transform)

    def _handle_position(self, msg: VehicleLocalPosition) -> None:
        if not msg.xy_valid or not msg.z_valid:
            return

        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self._odom_frame_id
        odom.child_frame_id = self._base_frame_id
        odom.pose.pose.position.x = float(msg.x)
        odom.pose.pose.position.y = float(msg.y)
        odom.pose.pose.position.z = 0.0 if self._project_to_2d else float(msg.z)

        qx, qy, qz, qw = _yaw_to_quaternion(float(msg.heading))
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        if msg.v_xy_valid:
            odom.twist.twist.linear.x = float(msg.vx)
            odom.twist.twist.linear.y = float(msg.vy)
        if msg.v_z_valid:
            odom.twist.twist.linear.z = 0.0 if self._project_to_2d else float(msg.vz)

        self._odom_pub.publish(odom)
        if self._publish_tf:
            self._publish_odom_tf(odom)

    def _publish_odom_tf(self, odom: Odometry) -> None:
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = Px4LocalPositionNav2Bridge()
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
