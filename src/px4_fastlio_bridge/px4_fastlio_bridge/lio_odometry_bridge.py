import math
from typing import Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import tf2_ros


Quaternion = Tuple[float, float, float, float]


def _quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_conjugate(q: Quaternion) -> Quaternion:
    x, y, z, w = q
    return (-x, -y, -z, w)


def _quat_normalize(q: Quaternion) -> Quaternion:
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def _flu_to_frd_vector(x: float, y: float, z: float) -> Tuple[float, float, float]:
    return (x, -y, -z)


def _flu_to_frd_quaternion(q: Quaternion) -> Quaternion:
    # Same 180 degree X-axis basis conversion on parent and child frames.
    basis = (1.0, 0.0, 0.0, 0.0)
    return _quat_normalize(_quat_multiply(_quat_multiply(basis, q), _quat_conjugate(basis)))


def _enu_to_ned_vector(x: float, y: float, z: float) -> Tuple[float, float, float]:
    return (y, x, -z)


def _enu_flu_to_ned_frd_quaternion(q: Quaternion) -> Quaternion:
    half_sqrt2 = math.sqrt(0.5)
    q_enu_to_ned = (half_sqrt2, half_sqrt2, 0.0, 0.0)
    q_frd_to_flu = (1.0, 0.0, 0.0, 0.0)
    return _quat_normalize(_quat_multiply(_quat_multiply(q_enu_to_ned, q), q_frd_to_flu))


class LioOdometryBridge(Node):
    def __init__(self) -> None:
        super().__init__("lio_odometry_bridge")

        self.declare_parameter("input_odom_topic", "/Odometry")
        self.declare_parameter("output_odom_topic", "/autonomy/lio_odometry")
        self.declare_parameter("output_nav2_odom_topic", "/odom")
        self.declare_parameter("output_position_topic", "/autonomy/lio_position_ned")
        self.declare_parameter("input_map_topic", "/Laser_map")
        self.declare_parameter("output_map_topic", "/autonomy/local_map")
        self.declare_parameter("input_registered_cloud_topic", "/cloud_registered")
        self.declare_parameter("output_registered_cloud_topic", "/autonomy/cloud_registered")
        self.declare_parameter("input_frame_id", "camera_init")
        self.declare_parameter("output_frame_id", "ned")
        self.declare_parameter("output_child_frame_id", "base_link_frd")
        self.declare_parameter("nav2_map_frame_id", "map")
        self.declare_parameter("nav2_odom_frame_id", "odom")
        self.declare_parameter("nav2_base_frame_id", "base_link")
        self.declare_parameter("publish_nav2_tf", True)
        self.declare_parameter("enable_sanity_check", True)
        self.declare_parameter("max_position_norm_m", 100.0)
        self.declare_parameter("max_position_delta_m", 2.0)
        self.declare_parameter("max_velocity_m_s", 5.0)
        self.declare_parameter("reset_after_rejects", 50)

        input_odom_topic = self.get_parameter("input_odom_topic").value
        output_odom_topic = self.get_parameter("output_odom_topic").value
        output_nav2_odom_topic = self.get_parameter("output_nav2_odom_topic").value
        output_position_topic = self.get_parameter("output_position_topic").value
        input_map_topic = self.get_parameter("input_map_topic").value
        output_map_topic = self.get_parameter("output_map_topic").value
        input_registered_cloud_topic = self.get_parameter("input_registered_cloud_topic").value
        output_registered_cloud_topic = self.get_parameter("output_registered_cloud_topic").value

        self._output_frame_id = self.get_parameter("output_frame_id").value
        self._output_child_frame_id = self.get_parameter("output_child_frame_id").value
        self._nav2_map_frame_id = self.get_parameter("nav2_map_frame_id").value
        self._nav2_odom_frame_id = self.get_parameter("nav2_odom_frame_id").value
        self._nav2_base_frame_id = self.get_parameter("nav2_base_frame_id").value
        self._publish_nav2_tf = bool(self.get_parameter("publish_nav2_tf").value)
        self._enable_sanity_check = bool(self.get_parameter("enable_sanity_check").value)
        self._max_position_norm_m = float(self.get_parameter("max_position_norm_m").value)
        self._max_position_delta_m = float(self.get_parameter("max_position_delta_m").value)
        self._max_velocity_m_s = float(self.get_parameter("max_velocity_m_s").value)
        self._reset_after_rejects = max(1, int(self.get_parameter("reset_after_rejects").value))
        self._last_raw_position: Tuple[float, float, float] | None = None
        self._last_raw_stamp_s: float | None = None
        self._consecutive_rejects = 0

        self._odom_pub = self.create_publisher(Odometry, output_odom_topic, 10)
        self._nav2_odom_pub = self.create_publisher(Odometry, output_nav2_odom_topic, 10)
        self._position_pub = self.create_publisher(PointStamped, output_position_topic, 10)
        self._map_pub = self.create_publisher(PointCloud2, output_map_topic, 10)
        self._registered_cloud_pub = self.create_publisher(PointCloud2, output_registered_cloud_topic, 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_map_to_odom()

        self.create_subscription(Odometry, input_odom_topic, self._handle_odom, 10)
        self.create_subscription(PointCloud2, input_map_topic, self._handle_map, 10)
        self.create_subscription(PointCloud2, input_registered_cloud_topic, self._handle_registered_cloud, 10)

        self.get_logger().info(
            f"Bridging FAST-LIO2 odometry {input_odom_topic} -> {output_odom_topic}, "
            f"Nav2 odom -> {output_nav2_odom_topic}, map {input_map_topic} -> {output_map_topic}"
        )

    def _handle_odom(self, msg: Odometry) -> None:
        raw_position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        )
        stamp_s = float(msg.header.stamp.sec) + (float(msg.header.stamp.nanosec) * 1e-9)
        if not self._lio_sample_is_sane(raw_position, stamp_s):
            return

        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = self._output_frame_id
        odom.child_frame_id = self._output_child_frame_id

        px, py, pz = _enu_to_ned_vector(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz

        q = msg.pose.pose.orientation
        qx, qy, qz, qw = _enu_flu_to_ned_frd_quaternion((q.x, q.y, q.z, q.w))
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = msg.pose.covariance

        vx, vy, vz = _flu_to_frd_vector(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )
        wx, wy, wz = _flu_to_frd_vector(
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        )
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.x = wx
        odom.twist.twist.angular.y = wy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = msg.twist.covariance

        position = PointStamped()
        position.header = odom.header
        position.point.x = px
        position.point.y = py
        position.point.z = pz

        self._odom_pub.publish(odom)
        self._position_pub.publish(position)

        nav2_odom = Odometry()
        nav2_odom.header = msg.header
        nav2_odom.header.frame_id = self._nav2_odom_frame_id
        nav2_odom.child_frame_id = self._nav2_base_frame_id
        nav2_odom.pose = msg.pose
        nav2_odom.twist = msg.twist
        self._nav2_odom_pub.publish(nav2_odom)
        if self._publish_nav2_tf:
            self._publish_nav2_odom_tf(nav2_odom)

    def _lio_sample_is_sane(self, position: Tuple[float, float, float], stamp_s: float) -> bool:
        if not self._enable_sanity_check:
            return True

        if not all(math.isfinite(value) for value in position):
            self._reject_lio_sample("non-finite position", reset_baseline=False)
            return False

        position_norm = math.sqrt(sum(value * value for value in position))
        if self._max_position_norm_m > 0.0 and position_norm > self._max_position_norm_m:
            self._reject_lio_sample(
                f"position norm {position_norm:.2f} m exceeds {self._max_position_norm_m:.2f} m",
                reset_baseline=False,
            )
            return False

        if self._last_raw_position is None or self._last_raw_stamp_s is None:
            self._accept_lio_sample(position, stamp_s)
            return True

        delta = math.sqrt(
            sum((position[index] - self._last_raw_position[index]) ** 2 for index in range(3))
        )
        dt = max(1e-3, stamp_s - self._last_raw_stamp_s)
        speed = delta / dt
        if self._max_position_delta_m > 0.0 and delta > self._max_position_delta_m:
            self._reject_lio_sample(
                f"position jump {delta:.2f} m exceeds {self._max_position_delta_m:.2f} m",
                reset_baseline=True,
            )
            return False
        if self._max_velocity_m_s > 0.0 and speed > self._max_velocity_m_s:
            self._reject_lio_sample(
                f"implied speed {speed:.2f} m/s exceeds {self._max_velocity_m_s:.2f} m/s",
                reset_baseline=True,
            )
            return False

        self._accept_lio_sample(position, stamp_s)
        return True

    def _accept_lio_sample(self, position: Tuple[float, float, float], stamp_s: float) -> None:
        self._last_raw_position = position
        self._last_raw_stamp_s = stamp_s
        self._consecutive_rejects = 0

    def _reject_lio_sample(self, reason: str, *, reset_baseline: bool) -> None:
        self._consecutive_rejects += 1
        self.get_logger().warn(
            f"Dropping FAST-LIO odometry sample: {reason}",
            throttle_duration_sec=1.0,
        )
        if reset_baseline and self._consecutive_rejects >= self._reset_after_rejects:
            self._last_raw_position = None
            self._last_raw_stamp_s = None
            self._consecutive_rejects = 0
            self.get_logger().warn("FAST-LIO sanity gate baseline reset after repeated rejects.")

    def _handle_map(self, msg: PointCloud2) -> None:
        relayed = self._copy_cloud(msg)
        relayed.header.frame_id = self._nav2_map_frame_id
        self._map_pub.publish(relayed)

    def _handle_registered_cloud(self, msg: PointCloud2) -> None:
        relayed = self._copy_cloud(msg)
        relayed.header.frame_id = self._nav2_map_frame_id
        self._registered_cloud_pub.publish(relayed)

    @staticmethod
    def _copy_cloud(msg: PointCloud2) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header = Header()
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = msg.header.frame_id
        cloud.height = msg.height
        cloud.width = msg.width
        cloud.fields = msg.fields
        cloud.is_bigendian = msg.is_bigendian
        cloud.point_step = msg.point_step
        cloud.row_step = msg.row_step
        cloud.data = msg.data
        cloud.is_dense = msg.is_dense
        return cloud

    def _publish_static_map_to_odom(self) -> None:
        if not self._publish_nav2_tf:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._nav2_map_frame_id
        transform.child_frame_id = self._nav2_odom_frame_id
        transform.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform(transform)

    def _publish_nav2_odom_tf(self, odom: Odometry) -> None:
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
    node = LioOdometryBridge()
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
