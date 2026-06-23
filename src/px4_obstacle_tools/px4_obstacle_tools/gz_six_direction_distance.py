import math
import os
import re
import subprocess
import threading
from typing import Optional

import rclpy
from px4_msgs.msg import ObstacleDistance, VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from px4_obstacle_tools.gz_scan_min_distance import _GZ_TOPIC_READER_SH


class GazeboSixDirectionDistance(Node):
    def __init__(self) -> None:
        super().__init__("gz_six_direction_distance")

        self.declare_parameter(
            "gz_scan_topic",
            "/gazebo/default/iris_rplidar/rplidar/link/laser/scan",
        )
        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("room_height_m", 5.0)
        self.declare_parameter("no_obstacle_distance_m", 20.0)
        self.declare_parameter("sector_width_deg", 35.0)
        self.declare_parameter("print_rate_hz", 1.0)
        self.declare_parameter("px4_publish_rate_hz", 10.0)
        self.declare_parameter("publish_px4_obstacle_distance", True)

        self._gz_scan_topic = self.get_parameter("gz_scan_topic").value
        vehicle_local_position_topic = self.get_parameter("vehicle_local_position_topic").value
        self._room_height_m = float(self.get_parameter("room_height_m").value)
        self._no_obstacle_distance_m = float(self.get_parameter("no_obstacle_distance_m").value)
        self._sector_half_width_rad = math.radians(
            float(self.get_parameter("sector_width_deg").value) / 2.0
        )

        self._distances = {
            "front": self._no_obstacle_distance_m,
            "back": self._no_obstacle_distance_m,
            "left": self._no_obstacle_distance_m,
            "right": self._no_obstacle_distance_m,
            "up": self._no_obstacle_distance_m,
            "down": self._no_obstacle_distance_m,
        }
        self._last_px4_timestamp_us = 0

        self._distance_publishers = {
            "min": self.create_publisher(Float32, "/perception/min_obstacle_distance", 10),
            "front": self.create_publisher(Float32, "/perception/front_obstacle_distance", 10),
            "back": self.create_publisher(Float32, "/perception/back_obstacle_distance", 10),
            "left": self.create_publisher(Float32, "/perception/left_obstacle_distance", 10),
            "right": self.create_publisher(Float32, "/perception/right_obstacle_distance", 10),
            "up": self.create_publisher(Float32, "/perception/up_obstacle_distance", 10),
            "down": self.create_publisher(Float32, "/perception/down_obstacle_distance", 10),
        }
        self._summary_pub = self.create_publisher(String, "/perception/six_direction_distance", 10)
        self._publish_px4_obstacle_distance_enabled = bool(
            self.get_parameter("publish_px4_obstacle_distance").value
        )
        self._px4_obstacle_pub = self.create_publisher(
            ObstacleDistance,
            "/fmu/in/obstacle_distance",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self.create_subscription(
            VehicleLocalPosition,
            vehicle_local_position_topic,
            self._on_local_position,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

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
        self.create_timer(1.0, self._check_process_health)

        px4_publish_period_s = 1.0 / max(float(self.get_parameter("px4_publish_rate_hz").value), 1.0)
        self.create_timer(px4_publish_period_s, self._publish_distances)

        print_period_s = 1.0 / max(float(self.get_parameter("print_rate_hz").value), 0.1)
        self.create_timer(print_period_s, self._print_summary)

        self.get_logger().info(
            f"Six-direction distance from {self._gz_scan_topic}; "
            f"room_height={self._room_height_m:.2f} m"
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

    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        if msg.timestamp > 0:
            self._last_px4_timestamp_us = int(msg.timestamp)

        if msg.dist_bottom_valid and math.isfinite(msg.dist_bottom):
            down = max(float(msg.dist_bottom), 0.0)
        elif msg.z_valid and math.isfinite(msg.z):
            down = max(-float(msg.z), 0.0)
        else:
            return

        self._distances["down"] = min(down, self._no_obstacle_distance_m)
        self._distances["up"] = min(
            max(self._room_height_m - down, 0.0),
            self._no_obstacle_distance_m,
        )

    def _read_gz_topic(self) -> None:
        if self._process is None or self._process.stdout is None:
            return

        ranges: list[float] = []
        scan_meta = {
            "angle_min": -math.pi,
            "angle_max": math.pi,
            "angle_step": None,
            "range_min": 0.1,
            "range_max": self._no_obstacle_distance_m,
        }
        collecting_ranges = False

        def consume_scan() -> None:
            nonlocal ranges, scan_meta
            if not ranges:
                return

            angle_min = float(scan_meta["angle_min"])
            angle_max = float(scan_meta["angle_max"])
            angle_step = scan_meta["angle_step"]
            if angle_step is None:
                angle_step = (angle_max - angle_min) / max(len(ranges) - 1, 1)
            range_min = float(scan_meta["range_min"])
            range_max = float(scan_meta["range_max"])

            buckets = {
                "front": [],
                "back": [],
                "left": [],
                "right": [],
            }
            for index, distance in enumerate(ranges):
                if not math.isfinite(distance) or not (range_min <= distance <= range_max):
                    continue
                angle = self._normalize_angle(angle_min + (float(angle_step) * index))
                if self._angle_near(angle, 0.0):
                    buckets["front"].append(distance)
                if self._angle_near(angle, math.pi):
                    buckets["back"].append(distance)
                if self._angle_near(angle, math.pi / 2.0):
                    buckets["left"].append(distance)
                if self._angle_near(angle, -math.pi / 2.0):
                    buckets["right"].append(distance)

            for key, values in buckets.items():
                self._distances[key] = min(values) if values else self._no_obstacle_distance_m

            ranges = []

        try:
            for line in self._process.stdout:
                stripped = line.strip()
                if stripped.startswith("time {"):
                    consume_scan()
                    scan_meta = {
                        "angle_min": -math.pi,
                        "angle_max": math.pi,
                        "angle_step": None,
                        "range_min": 0.1,
                        "range_max": self._no_obstacle_distance_m,
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

            consume_scan()
        except Exception:
            pass

    def _publish_distances(self) -> None:
        horizontal_min_distance = min(
            self._distances[key] for key in ("front", "back", "left", "right")
        )
        for key, publisher in self._distance_publishers.items():
            msg = Float32()
            msg.data = float(horizontal_min_distance if key == "min" else self._distances[key])
            publisher.publish(msg)

        if self._publish_px4_obstacle_distance_enabled:
            self._publish_px4_obstacle_distance()

    def _print_summary(self) -> None:
        summary = (
            f"前: {self._distances['front']:.2f} m\n"
            f"后: {self._distances['back']:.2f} m\n"
            f"左: {self._distances['left']:.2f} m\n"
            f"右: {self._distances['right']:.2f} m\n"
            f"上: {self._distances['up']:.2f} m\n"
            f"下: {self._distances['down']:.2f} m"
        )
        msg = String()
        msg.data = summary
        self._summary_pub.publish(msg)
        self.get_logger().info("\n" + summary)

    def _publish_px4_obstacle_distance(self) -> None:
        msg = ObstacleDistance()
        msg.timestamp = self._last_px4_timestamp_us or int(self.get_clock().now().nanoseconds / 1000)
        msg.frame = ObstacleDistance.MAV_FRAME_BODY_FRD
        msg.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        msg.increment = 5.0
        msg.min_distance = 20
        msg.max_distance = int(self._no_obstacle_distance_m * 100.0)
        msg.angle_offset = 0.0

        no_obstacle = msg.max_distance + 1
        msg.distances = [no_obstacle] * 72

        for direction, center_deg in (
            ("front", 0.0),
            ("right", 90.0),
            ("back", 180.0),
            ("left", 270.0),
        ):
            self._fill_obstacle_bins(
                msg.distances,
                center_deg,
                self._distances[direction],
                msg.min_distance,
                msg.max_distance,
            )

        self._px4_obstacle_pub.publish(msg)

    def _fill_obstacle_bins(
        self,
        bins: list[int],
        center_deg: float,
        distance_m: float,
        min_cm: int,
        max_cm: int,
    ) -> None:
        if not math.isfinite(distance_m):
            return

        if distance_m >= self._no_obstacle_distance_m:
            distance_cm = max_cm + 1
        else:
            distance_cm = int(max(distance_m, 0.0) * 100.0)
            if distance_cm < min_cm:
                distance_cm = 0
            elif distance_cm > max_cm:
                distance_cm = max_cm + 1

        half_bins = max(1, int(round(math.degrees(self._sector_half_width_rad) / 5.0)))
        center_bin = int(round(center_deg / 5.0)) % 72
        for offset in range(-half_bins, half_bins + 1):
            bins[(center_bin + offset) % 72] = distance_cm

    def _angle_near(self, angle: float, target: float) -> bool:
        return abs(self._normalize_angle(angle - target)) <= self._sector_half_width_rad

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        return angle

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
    node = GazeboSixDirectionDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
