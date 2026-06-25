import math

import rclpy
from px4_msgs.msg import VehicleImu
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class Px4ImuBridge(Node):
    def __init__(self) -> None:
        super().__init__("px4_imu_bridge")

        self.declare_parameter("input_topic", "/fmu/out/vehicle_imu")
        self.declare_parameter("output_topic", "/sim/imu")
        self.declare_parameter("frame_id", "base_link")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._frame_id = self.get_parameter("frame_id").value

        self._pub = self.create_publisher(Imu, output_topic, 10)
        self.create_subscription(
            VehicleImu,
            input_topic,
            self._handle_vehicle_imu,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"PX4 VehicleImu -> sensor_msgs/Imu: {input_topic} -> {output_topic}"
        )

    def _handle_vehicle_imu(self, msg: VehicleImu) -> None:
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
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
