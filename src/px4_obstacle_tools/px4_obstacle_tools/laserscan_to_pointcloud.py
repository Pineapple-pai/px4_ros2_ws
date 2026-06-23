import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection


class LaserScanToPointCloud(Node):
    """Convert /lidar/scan (LaserScan) to /lidar/points (PointCloud2)."""

    def __init__(self) -> None:
        super().__init__("laserscan_to_pointcloud")

        self.declare_parameter("scan_topic", "/lidar/scan")
        self.declare_parameter("cloud_topic", "/lidar/points")

        scan_topic = self.get_parameter("scan_topic").value
        cloud_topic = self.get_parameter("cloud_topic").value

        self._projector = LaserProjection()
        self._publisher = self.create_publisher(PointCloud2, cloud_topic, 10)
        self._subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self._on_scan,
            10,
        )

        self.get_logger().info(
            f"LaserScan → PointCloud2: {scan_topic} -> {cloud_topic}"
        )

    def _on_scan(self, scan_msg: LaserScan) -> None:
        try:
            cloud_msg = self._projector.projectLaser(scan_msg)
            cloud_msg.header = scan_msg.header
            self._publisher.publish(cloud_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to project laser scan: {e}")


def main() -> None:
    rclpy.init()
    node = LaserScanToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
