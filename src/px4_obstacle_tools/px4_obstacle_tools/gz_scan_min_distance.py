import re
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class GazeboScanMinDistance(Node):
    def __init__(self) -> None:
        super().__init__("gz_scan_min_distance")

        self.declare_parameter(
            "gz_scan_topic",
            "/world/room_obstacles/model/x500_lidar_front_0/link/lidar_sensor_link/sensor/lidar/scan",
        )
        self.declare_parameter("distance_topic", "/perception/min_obstacle_distance")
        self.declare_parameter("no_obstacle_distance_m", 20.0)

        self._gz_scan_topic = self.get_parameter("gz_scan_topic").value
        distance_topic = self.get_parameter("distance_topic").value
        self._no_obstacle_distance_m = float(self.get_parameter("no_obstacle_distance_m").value)

        self._publisher = self.create_publisher(Float32, distance_topic, 10)
        self._range_pattern = re.compile(r"^\s*ranges:\s*([-+0-9.eE]+)\s*$")
        self._process = subprocess.Popen(
            ["gz", "topic", "-e", "-t", self._gz_scan_topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self._reader = threading.Thread(target=self._read_gz_topic, daemon=True)
        self._reader.start()

        self.get_logger().info(f"Reading Gazebo scan {self._gz_scan_topic} -> {distance_topic}")

    def _read_gz_topic(self) -> None:
        if self._process.stdout is None:
            return

        received_first_range = False
        for line in self._process.stdout:
            match = self._range_pattern.match(line)
            if match is None:
                continue

            try:
                distance = float(match.group(1))
            except ValueError:
                distance = self._no_obstacle_distance_m

            msg = Float32()
            msg.data = distance
            self._publisher.publish(msg)

            if not received_first_range:
                received_first_range = True
                self.get_logger().info(f"Received first Gazebo lidar range: {distance:.2f} m")

    def destroy_node(self) -> bool:
        if self._process.poll() is None:
            self._process.terminate()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = GazeboScanMinDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
