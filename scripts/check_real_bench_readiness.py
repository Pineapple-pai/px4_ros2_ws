#!/usr/bin/env python3
import argparse
import math
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check PX4/FAST-LIO/planner readiness before enabling Ego-Planner Offboard flight."
    )
    parser.add_argument("--duration-s", type=float, default=8.0)
    parser.add_argument("--min-px4-status-hz", type=float, default=2.0)
    parser.add_argument("--min-px4-local-hz", type=float, default=2.0)
    parser.add_argument("--min-imu-hz", type=float, default=20.0)
    parser.add_argument("--min-lidar-hz", type=float, default=5.0)
    parser.add_argument("--min-fastlio-odom-hz", type=float, default=5.0)
    parser.add_argument("--min-ego-local-map-hz", type=float, default=2.0)
    parser.add_argument("--max-frame-horizontal-error-m", type=float, default=0.50)
    parser.add_argument("--max-frame-vertical-error-m", type=float, default=0.30)
    parser.add_argument("--max-fastlio-drift-m", type=float, default=0.20)
    parser.add_argument("--max-fastlio-step-m", type=float, default=0.10)
    parser.add_argument("--max-timestamp-stall-ratio", type=float, default=0.05)
    parser.add_argument("--expected-local-map-frame", default="world")
    parser.add_argument("--px4-status-topic", default="/fmu/out/vehicle_status_v4")
    parser.add_argument("--px4-local-position-topic", default="/fmu/out/vehicle_local_position_v1")
    parser.add_argument("--px4-imu-topic", default="/fmu/out/sensor_combined")
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument("--fastlio-odom-topic", default="/Odometry")
    parser.add_argument("--planner-odom-topic", default="/odom")
    parser.add_argument("--ego-local-map-topic", default="/autonomy/local_map")
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
        "px4_local_valid": False,
        "px4_enu": None,
        "planner_enu": None,
        "fastlio_enu": None,
        "last_lidar_stamp": None,
        "lidar_stamp_regressions": 0,
        "last_imu_timestamp": None,
        "imu_timestamp_regressions": 0,
        "imu_timestamp_stalls": 0,
        "lidar_timestamp_stalls": 0,
        "non_finite_samples": 0,
        "fastlio_positions": [],
        "frame_errors": [],
        "ego_local_map_frame_ids": set(),
        "ego_local_map_empty_count": 0,
    }

    def px4_status_cb(msg):
        counts["px4_status"] += 1
        state["status"] = msg

    def px4_local_cb(msg):
        counts["px4_local"] += 1
        state["px4_local_valid"] = bool(msg.xy_valid and msg.z_valid)
        position = (float(msg.y), float(msg.x), float(-msg.z))
        if all(math.isfinite(value) for value in position):
            state["px4_enu"] = position
        else:
            state["non_finite_samples"] += 1

    def px4_imu_cb(msg):
        counts["px4_imu"] += 1
        timestamp = int(msg.timestamp)
        last = state["last_imu_timestamp"]
        if last is not None:
            if timestamp < last:
                state["imu_timestamp_regressions"] += 1
            elif timestamp == last:
                state["imu_timestamp_stalls"] += 1
        state["last_imu_timestamp"] = timestamp

    def lidar_cb(msg):
        counts["lidar"] += 1
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        last = state["last_lidar_stamp"]
        if last is not None:
            if stamp_ns < last:
                state["lidar_stamp_regressions"] += 1
            elif stamp_ns == last:
                state["lidar_timestamp_stalls"] += 1
        state["last_lidar_stamp"] = stamp_ns

    def fastlio_odom_cb(msg):
        counts["fastlio_odom"] += 1
        p = msg.pose.pose.position
        position = (float(p.x), float(p.y), float(p.z))
        if all(math.isfinite(value) for value in position):
            state["fastlio_enu"] = position
            state["fastlio_positions"].append(position)
        else:
            state["non_finite_samples"] += 1

    def planner_odom_cb(msg):
        counts["planner_odom"] += 1
        p = msg.pose.pose.position
        position = (float(p.x), float(p.y), float(p.z))
        if not all(math.isfinite(value) for value in position):
            state["non_finite_samples"] += 1
            return
        state["planner_enu"] = position
        if state["px4_enu"] is not None:
            px4 = state["px4_enu"]
            planner = state["planner_enu"]
            state["frame_errors"].append(
                (math.hypot(planner[0] - px4[0], planner[1] - px4[1]), abs(planner[2] - px4[2]))
            )

    def ego_local_map_cb(msg):
        counts["ego_local_map"] += 1
        state["ego_local_map_frame_ids"].add(msg.header.frame_id)
        if int(msg.width) * int(msg.height) == 0:
            state["ego_local_map_empty_count"] += 1

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
    require_rate("ego_local_map", args.min_ego_local_map_hz, args.ego_local_map_topic)

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
        if not status.pre_flight_checks_pass:
            failures.append("PX4 pre-flight checks are not passing")
        if status.failsafe:
            failures.append("PX4 reports failsafe")

    if not state["px4_local_valid"]:
        failures.append("PX4 local horizontal/vertical position is not valid")
    if state["non_finite_samples"]:
        failures.append(f"Observed {state['non_finite_samples']} non-finite position samples")

    if state["imu_timestamp_regressions"]:
        failures.append(f"PX4 IMU timestamp regressed {state['imu_timestamp_regressions']} times")
    if state["lidar_stamp_regressions"]:
        failures.append(f"LiDAR timestamp regressed {state['lidar_stamp_regressions']} times")
    imu_stall_ratio = state["imu_timestamp_stalls"] / max(1, counts["px4_imu"] - 1)
    lidar_stall_ratio = state["lidar_timestamp_stalls"] / max(1, counts["lidar"] - 1)
    if imu_stall_ratio > args.max_timestamp_stall_ratio:
        failures.append(f"PX4 IMU timestamp stall ratio {imu_stall_ratio:.3f} exceeds limit")
    if lidar_stall_ratio > args.max_timestamp_stall_ratio:
        failures.append(f"LiDAR timestamp stall ratio {lidar_stall_ratio:.3f} exceeds limit")
    print(
        "TIMESTAMP_CHECK "
        f"imu_regressions={state['imu_timestamp_regressions']} "
        f"lidar_regressions={state['lidar_stamp_regressions']} "
        f"imu_stall_ratio={imu_stall_ratio:.3f} "
        f"lidar_stall_ratio={lidar_stall_ratio:.3f}"
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

    if state["frame_errors"]:
        horizontal_errors = [sample[0] for sample in state["frame_errors"]]
        vertical_errors = [sample[1] for sample in state["frame_errors"]]
        print(
            "FRAME_ALIGNMENT_WINDOW "
            f"samples={len(state['frame_errors'])} "
            f"horizontal_mean={sum(horizontal_errors) / len(horizontal_errors):.3f} "
            f"horizontal_max={max(horizontal_errors):.3f} "
            f"vertical_mean={sum(vertical_errors) / len(vertical_errors):.3f} "
            f"vertical_max={max(vertical_errors):.3f}"
        )
        if max(horizontal_errors) > args.max_frame_horizontal_error_m:
            failures.append("planner/PX4 horizontal window error exceeded limit")
        if max(vertical_errors) > args.max_frame_vertical_error_m:
            failures.append("planner/PX4 vertical window error exceeded limit")

    if fastlio_enu is not None:
        print(f"FASTLIO_ODOM last={tuple(round(value, 3) for value in fastlio_enu)}")

    fastlio_positions = state["fastlio_positions"]
    if len(fastlio_positions) >= 2:
        origin = fastlio_positions[0]
        drift = max(math.dist(origin, point) for point in fastlio_positions)
        max_step = max(math.dist(previous, current) for previous, current in zip(fastlio_positions, fastlio_positions[1:]))
        print(f"FASTLIO_STABILITY samples={len(fastlio_positions)} max_drift={drift:.3f} max_step={max_step:.3f}")
        if drift > args.max_fastlio_drift_m:
            failures.append(f"FAST-LIO drift {drift:.3f} m > {args.max_fastlio_drift_m:.3f} m")
        if max_step > args.max_fastlio_step_m:
            failures.append(f"FAST-LIO max step {max_step:.3f} m > {args.max_fastlio_step_m:.3f} m")

    print(
        "EGO_LOCAL_MAP "
        f"frames={sorted(state['ego_local_map_frame_ids'])} "
        f"empty={state['ego_local_map_empty_count']}/{counts['ego_local_map']}"
    )
    if state["ego_local_map_empty_count"]:
        failures.append(f"Ego local map contained {state['ego_local_map_empty_count']} empty messages")
    if state["ego_local_map_frame_ids"] != {args.expected_local_map_frame}:
        failures.append(
            "Ego local map frame mismatch: "
            f"observed={sorted(state['ego_local_map_frame_ids'])} expected={args.expected_local_map_frame}"
        )

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
