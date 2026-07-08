#!/usr/bin/env python3
import argparse
import math
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check real-bench readiness before enabling Ego-Planner Offboard flight."
    )
    parser.add_argument("--duration-s", type=float, default=8.0)
    parser.add_argument("--min-px4-status-hz", type=float, default=2.0)
    parser.add_argument("--min-px4-local-hz", type=float, default=2.0)
    parser.add_argument("--min-imu-hz", type=float, default=20.0)
    parser.add_argument("--min-lidar-hz", type=float, default=5.0)
    parser.add_argument("--min-fastlio-odom-hz", type=float, default=5.0)
    parser.add_argument("--max-frame-horizontal-error-m", type=float, default=0.50)
    parser.add_argument("--max-frame-vertical-error-m", type=float, default=0.30)
    parser.add_argument("--px4-status-topic", default="/fmu/out/vehicle_status_v4")
    parser.add_argument("--px4-local-position-topic", default="/fmu/out/vehicle_local_position_v1")
    parser.add_argument("--px4-imu-topic", default="/fmu/out/sensor_combined")
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument("--fastlio-odom-topic", default="/Odometry")
    parser.add_argument("--planner-odom-topic", default="/odom")
    parser.add_argument("--ego-local-map-topic", default="/autonomy/ego_local_map")
    args = parser.parse_args()

    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from px4_msgs.msg import SensorCombined
        from px4_msgs.msg import VehicleLocalPosition
        from px4_msgs.msg import VehicleStatus
        from rclpy.qos import HistoryPolicy
        from rclpy.qos import QoSProfile
        from rclpy.qos import ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
    except ModuleNotFoundError as exc:
        print(f"Missing runtime dependency: {exc}", file=sys.stderr)
        return 1

    rclpy.init()
    node = rclpy.create_node("real_bench_readiness_check")
    qos = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
    )

    counts = {
        "px4_status": 0,
        "px4_local": 0,
        "px4_imu": 0,
        "lidar": 0,
        "fastlio_odom": 0,
        "planner_odom": 0,
        "ego_local_map": 0,
    }
    state = {
        "status": None,
        "px4_enu": None,
        "planner_enu": None,
        "fastlio_enu": None,
        "last_lidar_stamp": None,
        "lidar_stamp_regressions": 0,
        "last_imu_timestamp": None,
        "imu_timestamp_regressions": 0,
    }

    def px4_status_cb(msg):
        counts["px4_status"] += 1
        state["status"] = msg

    def px4_local_cb(msg):
        counts["px4_local"] += 1
        state["px4_enu"] = (float(msg.y), float(msg.x), float(-msg.z))

    def px4_imu_cb(msg):
        counts["px4_imu"] += 1
        timestamp = int(msg.timestamp)
        last = state["last_imu_timestamp"]
        if last is not None and timestamp < last:
            state["imu_timestamp_regressions"] += 1
        state["last_imu_timestamp"] = timestamp

    def lidar_cb(msg):
        counts["lidar"] += 1
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        last = state["last_lidar_stamp"]
        if last is not None and stamp_ns < last:
            state["lidar_stamp_regressions"] += 1
        state["last_lidar_stamp"] = stamp_ns

    def fastlio_odom_cb(msg):
        counts["fastlio_odom"] += 1
        p = msg.pose.pose.position
        state["fastlio_enu"] = (float(p.x), float(p.y), float(p.z))

    def planner_odom_cb(msg):
        counts["planner_odom"] += 1
        p = msg.pose.pose.position
        state["planner_enu"] = (float(p.x), float(p.y), float(p.z))

    def ego_local_map_cb(_msg):
        counts["ego_local_map"] += 1

    node.create_subscription(VehicleStatus, args.px4_status_topic, px4_status_cb, qos)
    node.create_subscription(VehicleLocalPosition, args.px4_local_position_topic, px4_local_cb, qos)
    node.create_subscription(SensorCombined, args.px4_imu_topic, px4_imu_cb, qos)
    node.create_subscription(PointCloud2, args.lidar_topic, lidar_cb, qos)
    node.create_subscription(Odometry, args.fastlio_odom_topic, fastlio_odom_cb, 10)
    node.create_subscription(Odometry, args.planner_odom_topic, planner_odom_cb, 10)
    node.create_subscription(PointCloud2, args.ego_local_map_topic, ego_local_map_cb, qos)

    start = time.time()
    deadline = start + max(2.0, args.duration_s)
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    elapsed = max(0.001, time.time() - start)
    rates = {key: value / elapsed for key, value in counts.items()}

    failures = []

    def require_rate(key, minimum, label):
        rate = rates[key]
        print(f"{label}: count={counts[key]} rate={rate:.2f} Hz required>={minimum:.2f}")
        if rate < minimum:
            failures.append(f"{label} rate {rate:.2f} Hz < {minimum:.2f} Hz")

    require_rate("px4_status", args.min_px4_status_hz, args.px4_status_topic)
    require_rate("px4_local", args.min_px4_local_hz, args.px4_local_position_topic)
    require_rate("px4_imu", args.min_imu_hz, args.px4_imu_topic)
    require_rate("lidar", args.min_lidar_hz, args.lidar_topic)
    require_rate("fastlio_odom", args.min_fastlio_odom_hz, args.fastlio_odom_topic)
    require_rate("planner_odom", args.min_fastlio_odom_hz, args.planner_odom_topic)
    print(f"{args.ego_local_map_topic}: count={counts['ego_local_map']} rate={rates['ego_local_map']:.2f} Hz")

    status = state["status"]
    if status is None:
        failures.append("PX4 vehicle status unavailable")
    else:
        print(
            "PX4_STATUS "
            f"arming_state={int(status.arming_state)} "
            f"nav_state={int(status.nav_state)} "
            f"preflight={bool(status.pre_flight_checks_pass)} "
            f"failsafe={bool(status.failsafe)}"
        )
        if status.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
            failures.append("PX4 is not disarmed; bench readiness must be checked with props removed and disarmed")
        if status.failsafe:
            failures.append("PX4 reports failsafe")

    if state["imu_timestamp_regressions"]:
        failures.append(f"PX4 IMU timestamp regressed {state['imu_timestamp_regressions']} times")
    if state["lidar_stamp_regressions"]:
        failures.append(f"LiDAR timestamp regressed {state['lidar_stamp_regressions']} times")
    print(
        "TIMESTAMP_CHECK "
        f"imu_regressions={state['imu_timestamp_regressions']} "
        f"lidar_regressions={state['lidar_stamp_regressions']}"
    )

    px4_enu = state["px4_enu"]
    planner_enu = state["planner_enu"]
    fastlio_enu = state["fastlio_enu"]
    if px4_enu is None or planner_enu is None:
        failures.append("Cannot compare planner /odom with PX4 local ENU")
    else:
        horizontal_error = math.hypot(planner_enu[0] - px4_enu[0], planner_enu[1] - px4_enu[1])
        vertical_error = abs(planner_enu[2] - px4_enu[2])
        print(
            "FRAME_ALIGNMENT "
            f"planner_enu={tuple(round(value, 3) for value in planner_enu)} "
            f"px4_enu={tuple(round(value, 3) for value in px4_enu)} "
            f"horizontal_error={horizontal_error:.3f} "
            f"vertical_error={vertical_error:.3f}"
        )
        if horizontal_error > args.max_frame_horizontal_error_m:
            failures.append(
                f"planner/PX4 horizontal error {horizontal_error:.3f} m > {args.max_frame_horizontal_error_m:.3f} m"
            )
        if vertical_error > args.max_frame_vertical_error_m:
            failures.append(
                f"planner/PX4 vertical error {vertical_error:.3f} m > {args.max_frame_vertical_error_m:.3f} m"
            )

    if fastlio_enu is not None:
        print(f"FASTLIO_ODOM last={tuple(round(value, 3) for value in fastlio_enu)}")

    print("MANUAL_SAFETY_REQUIRED props_removed=true frame_secured=true rc_mode_switch_tested=true kill_switch_tested=true")

    node.destroy_node()
    rclpy.shutdown()

    if failures:
        print("BENCH_READINESS_RESULT passed=False")
        for failure in failures:
            print(f"FAIL: {failure}")
        return 2

    print("BENCH_READINESS_RESULT passed=True")
    return 0


if __name__ == "__main__":
    sys.exit(main())
