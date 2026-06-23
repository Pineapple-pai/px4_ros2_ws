import math
import os
import re
import subprocess
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from px4_obstacle_tools.gz_scan_min_distance import _GZ_TOPIC_READER_SH


class GazeboScanToPointCloud(Node):
    """Convert Gazebo Classic transport LaserScan text output to PointCloud2."""

    def __init__(self) -> None:
        super().__init__("gz_scan_to_pointcloud")

        self.declare_parameter(
            "gz_scan_topic",
            "/gazebo/default/iris_rplidar/rplidar/link/laser/scan",
        )
        self.declare_parameter("cloud_topic", "/livox/lidar")
        self.declare_parameter("frame_id", "livox_frame")
        self.declare_parameter("range_min_m", 0.1)
        self.declare_parameter("range_max_m", 20.0)
        self.declare_parameter("angle_min_rad", -3.141592653589793)
        self.declare_parameter("angle_max_rad", 3.141592653589793)

        self._gz_scan_topic = self.get_parameter("gz_scan_topic").value
        cloud_topic = self.get_parameter("cloud_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._range_min_m = float(self.get_parameter("range_min_m").value)
        self._range_max_m = float(self.get_parameter("range_max_m").value)
        self._default_angle_min_rad = float(self.get_parameter("angle_min_rad").value)
        self._default_angle_max_rad = float(self.get_parameter("angle_max_rad").value)

        self._publisher = self.create_publisher(PointCloud2, cloud_topic, 10)
        self._float_pattern = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")
        self._single_value_patterns = {
            "angle_min": re.compile(r"angle_min:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"),
            "angle_max": re.compile(r"angle_max:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"),
            "angle_step": re.compile(r"angle_step:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"),
            "range_min": re.compile(r"range_min:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"),
            "range_max": re.compile(r"range_max:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"),
        }

        self._process: Optional[subprocess.Popen[str]] = None
        self._shutdown_event = threading.Event()
        self._process_lock = threading.Lock()

        self._start_process()
        self._reader = threading.Thread(target=self._read_gz_topic, daemon=True)
        self._reader.start()
        self._health_timer = self.create_timer(1.0, self._check_process_health)

        self.get_logger().info(
            f"Gazebo scan -> PointCloud2: {self._gz_scan_topic} -> {cloud_topic}"
        )

    def _clean_env(self) -> dict[str, str]:
        env = {}
        for key in (
            "PATH",
            "HOME",
            "USER",
            "DISPLAY",
            "XAUTHORITY",
            "GAZEBO_MASTER_URI",
            "GAZEBO_MODEL_PATH",
            "GAZEBO_PLUGIN_PATH",
            "GAZEBO_RESOURCE_PATH",
            "GAZEBO_MODEL_DATABASE_URI",
        ):
            if key in os.environ:
                env[key] = os.environ[key]
        env["PATH"] = "/usr/local/bin:/usr/bin:/bin"
        env["LD_LIBRARY_PATH"] = (
            "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:"
            "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu"
        )
        return env

    def _start_process(self) -> None:
        with self._process_lock:
            if self._process is not None and self._process.poll() is None:
                self._process.terminate()
                try:
                    self._process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self._process.kill()

            self._process = subprocess.Popen(
                [_GZ_TOPIC_READER_SH, self._gz_scan_topic],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
                env=self._clean_env(),
            )

    def _check_process_health(self) -> None:
        if self._shutdown_event.is_set():
            return
        with self._process_lock:
            if self._process is None:
                return
            rc = self._process.poll()
        if rc is not None:
            self.get_logger().debug(f"gz topic reader exited ({rc}), restarting")
            self._start_process()
            self._reader = threading.Thread(target=self._read_gz_topic, daemon=True)
            self._reader.start()

    def _read_gz_topic(self) -> None:
        if self._process is None or self._process.stdout is None:
            return

        ranges: list[float] = []
        scan_meta = {
            "angle_min": self._default_angle_min_rad,
            "angle_max": self._default_angle_max_rad,
            "angle_step": None,
            "range_min": self._range_min_m,
            "range_max": self._range_max_m,
        }
        collecting_ranges = False
        received_first = False

        def publish_cloud() -> None:
            nonlocal ranges, scan_meta, received_first
            if not ranges:
                return

            angle_min = float(scan_meta["angle_min"])
            angle_max = float(scan_meta["angle_max"])
            angle_step = scan_meta["angle_step"]
            if angle_step is None:
                denom = max(len(ranges) - 1, 1)
                angle_step = (angle_max - angle_min) / denom
            range_min = float(scan_meta["range_min"])
            range_max = float(scan_meta["range_max"])

            points = []
            for index, distance in enumerate(ranges):
                if not math.isfinite(distance) or not (range_min <= distance <= range_max):
                    continue
                angle = angle_min + (float(angle_step) * index)
                points.append((distance * math.cos(angle), distance * math.sin(angle), 0.0))

            ranges = []
            if not points:
                return

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self._frame_id
            self._publisher.publish(point_cloud2.create_cloud_xyz32(header, points))

            if not received_first:
                received_first = True
                self.get_logger().info(
                    f"Published first point cloud with {len(points)} points"
                )

        try:
            for line in self._process.stdout:
                stripped = line.strip()
                if stripped.startswith("time {"):
                    publish_cloud()
                    scan_meta = {
                        "angle_min": self._default_angle_min_rad,
                        "angle_max": self._default_angle_max_rad,
                        "angle_step": None,
                        "range_min": self._range_min_m,
                        "range_max": self._range_max_m,
                    }
                    collecting_ranges = False
                    continue

                for name, pattern in self._single_value_patterns.items():
                    match = pattern.search(stripped)
                    if match is None:
                        continue
                    try:
                        scan_meta[name] = float(match.group(1))
                    except ValueError:
                        pass

                if "ranges:" in stripped:
                    collecting_ranges = True
                    values_text = stripped.split("ranges:", 1)[1]
                elif collecting_ranges:
                    values_text = stripped
                else:
                    continue

                values = self._float_pattern.findall(values_text)
                if not values and not stripped.startswith(("[", "]")):
                    collecting_ranges = False
                    continue

                for value in values:
                    try:
                        ranges.append(float(value))
                    except ValueError:
                        continue

            publish_cloud()
        except Exception:
            pass

    def destroy_node(self) -> bool:
        self._shutdown_event.set()
        with self._process_lock:
            if self._process is not None and self._process.poll() is None:
                self._process.terminate()
                try:
                    self._process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self._process.kill()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = GazeboScanToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
