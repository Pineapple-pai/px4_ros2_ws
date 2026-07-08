import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class Mid360SimBridge(Node):
    def __init__(self) -> None:
        super().__init__("mid360_sim_bridge")

        self.declare_parameter("input_topic", "/sim/mid360/points_raw")
        self.declare_parameter("output_topic", "/livox/lidar")
        self.declare_parameter("frame_id", "livox_frame")
        self.declare_parameter("num_lines", 32)
        self.declare_parameter("min_range_m", 0.2)
        self.declare_parameter("max_range_m", 40.0)
        self.declare_parameter("min_vertical_angle_rad", -0.2617993877991494)
        self.declare_parameter("max_vertical_angle_rad", 0.2617993877991494)
        self.declare_parameter("default_reflectivity", 80.0)
        self.declare_parameter("tag", 16)
        self.declare_parameter("scan_rate_hz", 10.0)
        self.declare_parameter("use_input_stamp", False)
        self.declare_parameter("point_time_unit", "nanoseconds")
        self.declare_parameter("line_time_offsets", [0.0, 0.000025, 0.000050, 0.000075])
        self.declare_parameter("vertical_angle_table_rad", [-0.2617993877991494, -0.08726646259971647, 0.08726646259971647, 0.2617993877991494])
        self.declare_parameter("pattern_mode", 0)
        self.declare_parameter("snapshot_time", True)
        self.declare_parameter("line_phase_jitter_ratio", 0.15)
        self.declare_parameter("azimuth_jitter_ratio", 0.05)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._num_lines = max(1, int(self.get_parameter("num_lines").value))
        self._min_range = float(self.get_parameter("min_range_m").value)
        self._max_range = float(self.get_parameter("max_range_m").value)
        self._min_vertical = float(self.get_parameter("min_vertical_angle_rad").value)
        self._max_vertical = float(self.get_parameter("max_vertical_angle_rad").value)
        self._default_reflectivity = float(self.get_parameter("default_reflectivity").value)
        self._tag = int(self.get_parameter("tag").value) & 0xFF
        self._scan_rate_hz = max(1e-3, float(self.get_parameter("scan_rate_hz").value))
        self._use_input_stamp = bool(self.get_parameter("use_input_stamp").value)
        self._point_time_scale = self._time_unit_scale(
            str(self.get_parameter("point_time_unit").value)
        )
        self._line_time_offsets = [float(v) for v in self.get_parameter("line_time_offsets").value]
        self._vertical_angle_table = [float(v) for v in self.get_parameter("vertical_angle_table_rad").value]
        self._pattern_mode = int(self.get_parameter("pattern_mode").value)
        self._snapshot_time = bool(self.get_parameter("snapshot_time").value)
        self._line_phase_jitter_ratio = max(0.0, float(self.get_parameter("line_phase_jitter_ratio").value))
        self._azimuth_jitter_ratio = max(0.0, float(self.get_parameter("azimuth_jitter_ratio").value))
        if len(self._vertical_angle_table) != self._num_lines:
            self._vertical_angle_table = self._build_default_vertical_table()
        if len(self._line_time_offsets) != self._num_lines:
            self._line_time_offsets = [i * 2.5e-5 for i in range(self._num_lines)]

        self._publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self.create_subscription(PointCloud2, input_topic, self._handle_cloud, qos_profile_sensor_data)

    def _handle_cloud(self, msg: PointCloud2) -> None:
        field_names = {field.name for field in msg.fields}
        has_intensity = "intensity" in field_names
        iterator = point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z", "intensity") if has_intensity else ("x", "y", "z"),
            skip_nans=True,
        )

        points = []
        scan_period = 1.0 / self._scan_rate_hz
        for entry in iterator:
            if has_intensity:
                x, y, z, intensity = entry
            else:
                x, y, z = entry
                intensity = self._default_reflectivity

            distance = math.sqrt((x * x) + (y * y) + (z * z))
            if not (self._min_range <= distance <= self._max_range):
                continue

            elevation = math.atan2(z, math.hypot(x, y))
            line = self._nearest_line_index(elevation)
            yaw = math.atan2(y, x)
            yaw_normalized = (yaw + math.pi) / (2.0 * math.pi)
            yaw_normalized = min(1.0, max(0.0, yaw_normalized))
            point_time_s = 0.0
            if not self._snapshot_time:
                point_time_s = self._line_time_offsets[line] + self._pattern_time_offset(
                    line,
                    yaw_normalized,
                    x,
                    y,
                    z,
                    scan_period,
                )
            point_time = point_time_s * self._point_time_scale
            points.append((
                float(x),
                float(y),
                float(z),
                float(intensity),
                float(intensity),
                self._tag,
                line,
                float(point_time),
            ))

        if not points:
            return

        header = Header()
        header.stamp = msg.header.stamp if self._use_input_stamp else self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        cloud = point_cloud2.create_cloud(header, self._cloud_fields(), points)
        self._publisher.publish(cloud)

    @staticmethod
    def _time_unit_scale(unit: str) -> float:
        normalized = unit.strip().lower()
        if normalized in {"s", "sec", "second", "seconds"}:
            return 1.0
        if normalized in {"ms", "millisecond", "milliseconds"}:
            return 1.0e3
        if normalized in {"us", "microsecond", "microseconds"}:
            return 1.0e6
        if normalized in {"ns", "nanosecond", "nanoseconds"}:
            return 1.0e9
        raise ValueError(
            "Unsupported point_time_unit "
            f"'{unit}'. Use seconds, milliseconds, microseconds, or nanoseconds."
        )

    @staticmethod
    def _cloud_fields() -> list[PointField]:
        return [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="reflectivity", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="tag", offset=20, datatype=PointField.UINT8, count=1),
            PointField(name="line", offset=21, datatype=PointField.UINT8, count=1),
            PointField(name="time", offset=24, datatype=PointField.FLOAT64, count=1),
        ]

    def _nearest_line_index(self, elevation_rad: float) -> int:
        best_index = 0
        best_error = float("inf")
        for index, angle in enumerate(self._vertical_angle_table):
            error = abs(elevation_rad - angle)
            if error < best_error:
                best_error = error
                best_index = index
        return best_index

    def _build_default_vertical_table(self) -> list[float]:
        if self._num_lines == 1:
            return [0.0]
        step = (self._max_vertical - self._min_vertical) / float(self._num_lines - 1)
        return [self._min_vertical + (step * i) for i in range(self._num_lines)]

    def _pattern_time_offset(
        self,
        line: int,
        yaw_normalized: float,
        x: float,
        y: float,
        z: float,
        scan_period: float,
    ) -> float:
        if self._pattern_mode == 0:
            return yaw_normalized * scan_period

        point_hash = math.sin((x * 12.9898) + (y * 78.233) + (z * 37.719) + (line * 19.19))
        point_noise = point_hash - math.floor(point_hash)
        centered_noise = (point_noise * 2.0) - 1.0

        line_phase = (line / max(1, self._num_lines - 1)) * self._line_phase_jitter_ratio
        azimuth_term = yaw_normalized + (centered_noise * self._azimuth_jitter_ratio)
        azimuth_term = azimuth_term % 1.0
        return min(scan_period, (line_phase * scan_period) + (azimuth_term * scan_period))


def main() -> None:
    rclpy.init()
    node = Mid360SimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
