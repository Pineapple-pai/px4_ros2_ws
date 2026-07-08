import rclpy
from px4_msgs.msg import SensorCombined, VehicleImu
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class Px4ImuBridge(Node):
    def __init__(self) -> None:
        super().__init__("px4_imu_bridge")

        self.declare_parameter("input_topic", "/fmu/out/sensor_combined")
        self.declare_parameter("input_type", "sensor_combined")
        self.declare_parameter("output_topic", "/sim/imu")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("enforce_monotonic_stamp", True)
        self.declare_parameter("min_stamp_step_ns", 1000)

        input_topic = self.get_parameter("input_topic").value
        input_type = self.get_parameter("input_type").value
        output_topic = self.get_parameter("output_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._enforce_monotonic_stamp = bool(
            self.get_parameter("enforce_monotonic_stamp").value
        )
        self._min_stamp_step_ns = max(1, int(self.get_parameter("min_stamp_step_ns").value))
        self._last_stamp_ns: int | None = None

        self._pub = self.create_publisher(Imu, output_topic, 10)
        if input_type == "vehicle_imu":
            self.create_subscription(
                VehicleImu,
                input_topic,
                self._handle_vehicle_imu,
                qos_profile_sensor_data,
            )
        elif input_type == "sensor_combined":
            self.create_subscription(
                SensorCombined,
                input_topic,
                self._handle_sensor_combined,
                qos_profile_sensor_data,
            )
        else:
            raise ValueError(
                f"Unsupported input_type '{input_type}'. Use 'sensor_combined' or 'vehicle_imu'."
            )

        self.get_logger().info(
            f"PX4 {input_type} -> sensor_msgs/Imu: {input_topic} -> {output_topic}"
        )

    def _handle_vehicle_imu(self, msg: VehicleImu) -> None:
        imu = Imu()
        imu.header.stamp = self._next_stamp()
        imu.header.frame_id = self._frame_id

        dt_angle = max(msg.delta_angle_dt, 1) * 1e-6
        dt_vel = max(msg.delta_velocity_dt, 1) * 1e-6

        # PX4 publishes FRD deltas. Convert to FLU for ROS.
        imu.angular_velocity.x = float(msg.delta_angle[0] / dt_angle)
        imu.angular_velocity.y = float(-msg.delta_angle[1] / dt_angle)
        imu.angular_velocity.z = float(-msg.delta_angle[2] / dt_angle)

        imu.linear_acceleration.x = float(msg.delta_velocity[0] / dt_vel)
        imu.linear_acceleration.y = float(-msg.delta_velocity[1] / dt_vel)
        imu.linear_acceleration.z = float(-msg.delta_velocity[2] / dt_vel)

        imu.orientation_covariance[0] = -1.0
        self._pub.publish(imu)

    def _handle_sensor_combined(self, msg: SensorCombined) -> None:
        imu = Imu()
        imu.header.stamp = self._next_stamp()
        imu.header.frame_id = self._frame_id

        # PX4 publishes FRD measurements. Convert to ROS FLU for FAST-LIO.
        imu.angular_velocity.x = float(msg.gyro_rad[0])
        imu.angular_velocity.y = float(-msg.gyro_rad[1])
        imu.angular_velocity.z = float(-msg.gyro_rad[2])

        imu.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        imu.linear_acceleration.y = float(-msg.accelerometer_m_s2[1])
        imu.linear_acceleration.z = float(-msg.accelerometer_m_s2[2])

        imu.orientation_covariance[0] = -1.0
        self._pub.publish(imu)

    def _next_stamp(self):
        now = self.get_clock().now()
        stamp_ns = now.nanoseconds
        if (
            self._enforce_monotonic_stamp
            and self._last_stamp_ns is not None
            and stamp_ns <= self._last_stamp_ns
        ):
            stamp_ns = self._last_stamp_ns + self._min_stamp_step_ns
        self._last_stamp_ns = stamp_ns
        return rclpy.time.Time(nanoseconds=stamp_ns).to_msg()


def main() -> None:
    rclpy.init()
    node = Px4ImuBridge()
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
