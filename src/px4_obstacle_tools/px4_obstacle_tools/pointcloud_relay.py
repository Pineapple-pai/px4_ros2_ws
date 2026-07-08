import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class PointCloudRelay(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_relay")

        self.declare_parameter("input_topic", "/sim/mid360/points")
        self.declare_parameter("output_topic", "/autonomy/local_map")
        self.declare_parameter("output_frame_id", "")
        self.declare_parameter("max_rate_hz", 0.0)
        self.declare_parameter("point_step_stride", 1)
        self.declare_parameter("max_points", 0)
        self.declare_parameter("drop_non_finite_points", False)
        self.declare_parameter("min_world_z_m", float("-inf"))
        self.declare_parameter("max_world_z_m", float("inf"))
        self.declare_parameter("self_filter_enabled", False)
        self.declare_parameter("self_filter_radius_xy_m", 0.75)
        self.declare_parameter("self_filter_radius_z_m", 0.60)
        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("vehicle_odometry_topic", "")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._output_frame_id = self.get_parameter("output_frame_id").value
        self._stride = max(1, int(self.get_parameter("point_step_stride").value))
        self._max_points = max(0, int(self.get_parameter("max_points").value))
        self._drop_non_finite_points = bool(self.get_parameter("drop_non_finite_points").value)
        self._min_world_z_m = float(self.get_parameter("min_world_z_m").value)
        self._max_world_z_m = float(self.get_parameter("max_world_z_m").value)
        self._self_filter_enabled = bool(self.get_parameter("self_filter_enabled").value)
        self._self_filter_radius_xy_m = max(
            0.0, float(self.get_parameter("self_filter_radius_xy_m").value)
        )
        self._self_filter_radius_z_m = max(
            0.0, float(self.get_parameter("self_filter_radius_z_m").value)
        )
        max_rate_hz = max(0.0, float(self.get_parameter("max_rate_hz").value))
        self._min_period_ns = int(1.0e9 / max_rate_hz) if max_rate_hz > 0.0 else 0
        self._last_publish_ns: int | None = None
        self._vehicle_position_enu: tuple[float, float, float] | None = None
        self._vehicle_position_source = ""
        self._last_pose_warn_ns = 0

        self._pub = self.create_publisher(PointCloud2, output_topic, 10)
        if self._self_filter_enabled:
            vehicle_odometry_topic = self.get_parameter("vehicle_odometry_topic").value
            if vehicle_odometry_topic:
                self.create_subscription(
                    Odometry,
                    vehicle_odometry_topic,
                    self._handle_vehicle_odometry,
                    10,
                )
            vehicle_local_position_topic = self.get_parameter("vehicle_local_position_topic").value
            self.create_subscription(
                VehicleLocalPosition,
                vehicle_local_position_topic,
                self._handle_vehicle_local_position,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            PointCloud2,
            input_topic,
            self._handle_cloud,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Relaying PointCloud2 "
            f"{input_topic} -> {output_topic}, "
            f"stride={self._stride}, max_points={self._max_points}, "
            f"max_rate_hz={max_rate_hz}, output_frame_id='{self._output_frame_id}', "
            f"drop_non_finite_points={self._drop_non_finite_points}, "
            f"min_world_z_m={self._min_world_z_m}, max_world_z_m={self._max_world_z_m}, "
            f"self_filter_enabled={self._self_filter_enabled}"
        )

    def _handle_vehicle_local_position(self, msg: VehicleLocalPosition) -> None:
        if self._vehicle_position_source == "odom":
            return
        if not msg.xy_valid or not msg.z_valid:
            return
        # PX4 local position is NED. This is only a fallback when planner odom is unavailable.
        self._vehicle_position_enu = (float(msg.y), float(msg.x), float(-msg.z))
        self._vehicle_position_source = "px4_local"

    def _handle_vehicle_odometry(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if not all(math.isfinite(value) for value in (p.x, p.y, p.z)):
            return
        self._vehicle_position_enu = (float(p.x), float(p.y), float(p.z))
        self._vehicle_position_source = "odom"

    def _handle_cloud(self, msg: PointCloud2) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if (
            self._min_period_ns > 0
            and self._last_publish_ns is not None
            and now_ns - self._last_publish_ns < self._min_period_ns
        ):
            return

        if self._self_filter_enabled and self._vehicle_position_enu is None:
            if now_ns - self._last_pose_warn_ns > int(2.0e9):
                self.get_logger().warning(
                    "Skipping point cloud relay until a valid vehicle pose is available for self-filtering."
                )
                self._last_pose_warn_ns = now_ns
            return

        relayed = self._build_output_cloud(msg)
        self._pub.publish(relayed)
        self._last_publish_ns = now_ns

    def _build_output_cloud(self, msg: PointCloud2) -> PointCloud2:
        if not self._needs_repacking():
            relayed = self._copy_metadata(msg)
            if self._output_frame_id:
                relayed.header.frame_id = self._output_frame_id
            self._fill_cloud_data(msg, relayed)
            return relayed

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self._output_frame_id or msg.header.frame_id
        points = self._filter_points(msg)
        return point_cloud2.create_cloud_xyz32(header, points)

    def _needs_repacking(self) -> bool:
        return any(
            [
                self._stride != 1,
                self._max_points > 0,
                self._drop_non_finite_points,
                math.isfinite(self._min_world_z_m),
                math.isfinite(self._max_world_z_m),
                self._self_filter_enabled,
            ]
        )

    @staticmethod
    def _copy_metadata(msg: PointCloud2) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header = Header()
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = msg.header.frame_id
        cloud.fields = msg.fields
        cloud.is_bigendian = msg.is_bigendian
        cloud.point_step = msg.point_step
        cloud.is_dense = msg.is_dense
        return cloud

    def _fill_cloud_data(self, msg: PointCloud2, relayed: PointCloud2) -> None:
        total_points = int(msg.width) * int(msg.height)
        no_point_limit = self._max_points <= 0 or total_points <= self._max_points
        if self._stride == 1 and no_point_limit:
            relayed.height = msg.height
            relayed.width = msg.width
            relayed.row_step = msg.row_step
            relayed.data = msg.data
            return

        selected = bytearray()
        selected_count = 0
        source_index = 0
        stop = False

        for row in range(int(msg.height)):
            row_base = row * int(msg.row_step)
            for col in range(int(msg.width)):
                if source_index % self._stride == 0:
                    point_base = row_base + (col * int(msg.point_step))
                    selected.extend(msg.data[point_base:point_base + int(msg.point_step)])
                    selected_count += 1
                    if self._max_points > 0 and selected_count >= self._max_points:
                        stop = True
                        break
                source_index += 1
            if stop:
                break

        relayed.height = 1
        relayed.width = selected_count
        relayed.row_step = selected_count * int(msg.point_step)
        relayed.data = bytes(selected)

    def _filter_points(self, msg: PointCloud2) -> list[tuple[float, float, float]]:
        selected: list[tuple[float, float, float]] = []
        vehicle_position = self._vehicle_position_enu
        source_index = 0

        for point in point_cloud2.read_points(
            msg,
            field_names=["x", "y", "z"],
            skip_nans=False,
        ):
            x = float(point["x"])
            y = float(point["y"])
            z = float(point["z"])

            if self._drop_non_finite_points and (
                not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z)
            ):
                source_index += 1
                continue

            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                source_index += 1
                continue

            if z < self._min_world_z_m or z > self._max_world_z_m:
                source_index += 1
                continue

            if vehicle_position is not None:
                delta_x = x - vehicle_position[0]
                delta_y = y - vehicle_position[1]
                delta_z = z - vehicle_position[2]
                if (
                    math.hypot(delta_x, delta_y) <= self._self_filter_radius_xy_m
                    and abs(delta_z) <= self._self_filter_radius_z_m
                ):
                    source_index += 1
                    continue

            if source_index % self._stride == 0:
                selected.append((x, y, z))
                if self._max_points > 0 and len(selected) >= self._max_points:
                    break

            source_index += 1

        return selected


def main() -> None:
    rclpy.init()
    node = PointCloudRelay()
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
