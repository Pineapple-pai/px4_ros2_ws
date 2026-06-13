import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ObstacleDistanceSim(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_distance_sim")

        self.declare_parameter("topic", "/perception/min_obstacle_distance")
        self.declare_parameter("mode", "safe")
        self.declare_parameter("safe_distance_m", 5.0)
        self.declare_parameter("hold_distance_m", 1.5)
        self.declare_parameter("abort_distance_m", 0.5)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("wave_period_s", 12.0)

        topic = self.get_parameter("topic").get_parameter_value().string_value
        self._mode = self.get_parameter("mode").get_parameter_value().string_value
        self._safe_distance_m = self.get_parameter("safe_distance_m").get_parameter_value().double_value
        self._hold_distance_m = self.get_parameter("hold_distance_m").get_parameter_value().double_value
        self._abort_distance_m = self.get_parameter("abort_distance_m").get_parameter_value().double_value
        self._publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self._wave_period_s = self.get_parameter("wave_period_s").get_parameter_value().double_value

        self._publisher = self.create_publisher(Float32, topic, 10)
        self._start_time = self.get_clock().now()

        period = 1.0 / max(self._publish_rate_hz, 1.0)
        self._timer = self.create_timer(period, self._publish_distance)

        self.get_logger().info(
            f"Obstacle distance simulator publishing to {topic}, mode={self._mode}"
        )

    def _publish_distance(self) -> None:
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9

        if self._mode == "safe":
            distance = self._safe_distance_m
        elif self._mode == "hold":
            distance = self._hold_distance_m
        elif self._mode == "abort":
            distance = self._abort_distance_m
        elif self._mode == "wave":
            wave = 0.5 * (1.0 + math.sin((2.0 * math.pi * elapsed) / max(self._wave_period_s, 0.1)))
            distance = self._abort_distance_m + wave * (self._safe_distance_m - self._abort_distance_m)
        else:
            distance = self._safe_distance_m

        msg = Float32()
        msg.data = float(distance)
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = ObstacleDistanceSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
