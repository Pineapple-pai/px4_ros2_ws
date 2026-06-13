import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32


class PointCloudMinDistance(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_min_distance")

        self.declare_parameter("pointcloud_topic", "/lidar/points")
        self.declare_parameter("distance_topic", "/perception/min_obstacle_distance")
        self.declare_parameter("max_distance_m", 20.0)
        self.declare_parameter("min_distance_m", 0.1)
        self.declare_parameter("min_x_m", 0.0)
        self.declare_parameter("max_x_m", 20.0)
        self.declare_parameter("min_y_m", -2.0)
        self.declare_parameter("max_y_m", 2.0)
        self.declare_parameter("min_z_m", -1.5)
        self.declare_parameter("max_z_m", 1.5)
        self.declare_parameter("no_obstacle_distance_m", 20.0)

        pointcloud_topic = self.get_parameter("pointcloud_topic").value
        distance_topic = self.get_parameter("distance_topic").value
        self._max_distance_m = float(self.get_parameter("max_distance_m").value)
        self._min_distance_m = float(self.get_parameter("min_distance_m").value)
        self._min_x_m = float(self.get_parameter("min_x_m").value)
        self._max_x_m = float(self.get_parameter("max_x_m").value)
        self._min_y_m = float(self.get_parameter("min_y_m").value)
        self._max_y_m = float(self.get_parameter("max_y_m").value)
        self._min_z_m = float(self.get_parameter("min_z_m").value)
        self._max_z_m = float(self.get_parameter("max_z_m").value)
        self._no_obstacle_distance_m = float(self.get_parameter("no_obstacle_distance_m").value)

        self._publisher = self.create_publisher(Float32, distance_topic, 10)
        self._received_first_cloud = False
        self._subscription = self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self._on_pointcloud,
            10,
        )

        self.get_logger().info(
            f"Converting {pointcloud_topic} to {distance_topic} "
            f"with ROI x=[{self._min_x_m:.1f}, {self._max_x_m:.1f}], "
            f"y=[{self._min_y_m:.1f}, {self._max_y_m:.1f}], "
            f"z=[{self._min_z_m:.1f}, {self._max_z_m:.1f}]"
        )

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        if not self._received_first_cloud:
            self._received_first_cloud = True
            self.get_logger().info(
                f"Received first point cloud: frame={msg.header.frame_id}, "
                f"width={msg.width}, height={msg.height}, fields={[field.name for field in msg.fields]}"
            )

        closest = self._no_obstacle_distance_m
        valid_points = 0

        for x, y, z in point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ):
            if not (
                self._min_x_m <= x <= self._max_x_m
                and self._min_y_m <= y <= self._max_y_m
                and self._min_z_m <= z <= self._max_z_m
            ):
                continue

            distance = math.sqrt((x * x) + (y * y) + (z * z))
            if self._min_distance_m <= distance <= self._max_distance_m:
                valid_points += 1
                closest = min(closest, distance)

        out = Float32()
        out.data = float(closest)
        self._publisher.publish(out)

        if valid_points == 0:
            self.get_logger().debug("No obstacle points inside ROI")


def main() -> None:
    rclpy.init()
    node = PointCloudMinDistance()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
