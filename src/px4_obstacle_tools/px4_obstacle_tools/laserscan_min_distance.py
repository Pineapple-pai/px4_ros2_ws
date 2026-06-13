import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class LaserScanMinDistance(Node):
    def __init__(self) -> None:
        super().__init__("laserscan_min_distance")

        self.declare_parameter("scan_topic", "/lidar/scan")
        self.declare_parameter("distance_topic", "/perception/min_obstacle_distance")
        self.declare_parameter("no_obstacle_distance_m", 20.0)

        scan_topic = self.get_parameter("scan_topic").value
        distance_topic = self.get_parameter("distance_topic").value
        self._no_obstacle_distance_m = float(self.get_parameter("no_obstacle_distance_m").value)

        self._publisher = self.create_publisher(Float32, distance_topic, 10)
        self._received_first_scan = False
        self._subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self._on_scan,
            10,
        )

        self.get_logger().info(f"Converting {scan_topic} to {distance_topic}")

    def _on_scan(self, msg: LaserScan) -> None:
        if not self._received_first_scan:
            self._received_first_scan = True
            self.get_logger().info(
                f"Received first laser scan: frame={msg.header.frame_id}, "
                f"ranges={len(msg.ranges)}, range=[{msg.range_min:.2f}, {msg.range_max:.2f}]"
            )

        closest = self._no_obstacle_distance_m
        for distance in msg.ranges:
            if not math.isfinite(distance):
                continue
            if msg.range_min <= distance <= msg.range_max:
                closest = min(closest, distance)

        out = Float32()
        out.data = float(closest)
        self._publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = LaserScanMinDistance()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
