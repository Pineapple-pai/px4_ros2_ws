#!/usr/bin/env python3
import argparse
import math
import sys
import time


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Take off with a direct PX4 Offboard hover helper, hand back to Position mode, then validate Ego-Planner small-goal takeover."
    )
    parser.add_argument("--takeoff-altitude-m", type=float, default=1.5)
    parser.add_argument("--goal-forward-m", type=float, default=0.35)
    parser.add_argument("--target-threshold-m", type=float, default=0.20)
    parser.add_argument("--takeoff-timeout-s", type=float, default=35.0)
    parser.add_argument("--takeoff-hover-tolerance-m", type=float, default=0.15)
    parser.add_argument("--post-takeoff-settle-s", type=float, default=2.0)
    parser.add_argument("--handover-timeout-s", type=float, default=25.0)
    parser.add_argument("--land-timeout-s", type=float, default=45.0)
    parser.add_argument("--initial-ground-altitude-max-m", type=float, default=0.30)
    parser.add_argument("--land-complete-altitude-max-m", type=float, default=0.80)
    parser.add_argument("--max-move-pass-m", type=float, default=0.90)
    parser.add_argument("--max-alt-pass-m", type=float, default=2.20)
    parser.add_argument("--offboard-warmup-cycles", type=int, default=30)
    parser.add_argument("--setpoint-quiet-window-s", type=float, default=0.75)
    parser.add_argument("--max-frame-horizontal-error-m", type=float, default=1.0)
    parser.add_argument("--max-frame-vertical-error-m", type=float, default=0.6)
    parser.add_argument(
        "--inject-helper-timeout-after-arm",
        action="store_true",
        help="Simulation-only safety test: fail immediately after ARM and verify LAND/disarm cleanup.",
    )
    args = parser.parse_args()

    try:
        from pymavlink import mavutil
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from px4_msgs.msg import OffboardControlMode
        from px4_msgs.msg import BatteryStatus
        from px4_msgs.msg import FailsafeFlags
        from px4_msgs.msg import TrajectorySetpoint
        from px4_msgs.msg import VehicleCommand
        from px4_msgs.msg import VehicleLocalPosition
        from px4_msgs.msg import VehicleStatus
        from quadrotor_msgs.msg import PositionCommand
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy
        from rclpy.qos import QoSProfile
        from rclpy.qos import ReliabilityPolicy
        from traj_utils.msg import Bspline
    except ModuleNotFoundError as exc:
        print(f"Missing runtime dependency: {exc}", file=sys.stderr)
        return 1

    rclpy.init()
    node = Node("ego_small_goal_handover_validation")
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
    )
    state = {
        "status": None,
        "lpos": None,
        "odom": None,
        "bspline": 0,
        "pcmd": 0,
        "setpoint": 0,
        "failsafe_flags": None,
        "battery": None,
    }

    node.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4", lambda msg: state.__setitem__("status", msg), qos)
    node.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", lambda msg: state.__setitem__("lpos", msg), qos)
    node.create_subscription(FailsafeFlags, "/fmu/out/failsafe_flags", lambda msg: state.__setitem__("failsafe_flags", msg), qos)
    node.create_subscription(BatteryStatus, "/fmu/out/battery_status_v1", lambda msg: state.__setitem__("battery", msg), qos)
    node.create_subscription(Odometry, "/odom", lambda msg: state.__setitem__("odom", msg), 10)
    node.create_subscription(Bspline, "/planning/bspline", lambda msg: state.__setitem__("bspline", state["bspline"] + 1), 10)
    node.create_subscription(PositionCommand, "/planning/position_cmd", lambda msg: state.__setitem__("pcmd", state["pcmd"] + 1), 10)
    node.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", lambda msg: state.__setitem__("setpoint", state["setpoint"] + 1), 10)

    offboard_pub = node.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
    setpoint_pub = node.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
    command_pub = node.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
    goal_pub = node.create_publisher(PoseStamped, "/move_base_simple/goal", 10)

    def spin_for(duration_s: float):
        end = time.time() + duration_s
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def planner_enu():
        odom = state["odom"]
        if odom is None:
            return None
        p = odom.pose.pose.position
        return (float(p.x), float(p.y), float(p.z))

    def px4_enu():
        lpos = state["lpos"]
        if lpos is None:
            return None
        return (float(lpos.y), float(lpos.x), float(-lpos.z))

    def publish_hover_setpoint(x_ned: float, y_ned: float, z_ned: float):
        timestamp_us = int(node.get_clock().now().nanoseconds / 1000)

        mode = OffboardControlMode()
        mode.timestamp = timestamp_us
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        offboard_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = timestamp_us
        sp.position = [x_ned, y_ned, z_ned]
        sp.velocity = [0.0, 0.0, 0.0]
        sp.acceleration = [0.0, 0.0, 0.0]
        sp.yaw = float("nan")
        setpoint_pub.publish(sp)

    def publish_vehicle_command(command_id: int, param1: float = 0.0, param2: float = 0.0, param7: float = 0.0):
        cmd = VehicleCommand()
        cmd.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        cmd.command = command_id
        cmd.param1 = param1
        cmd.param2 = param2
        cmd.param7 = param7
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        command_pub.publish(cmd)

    def print_failsafe_context():
        flags = state["failsafe_flags"]
        battery = state["battery"]
        if flags is not None:
            print(
                "FAILSAFE_FLAGS battery_warning=%s battery_low_remaining_time=%s battery_unhealthy=%s "
                "offboard_signal_lost=%s local_position_invalid=%s local_altitude_invalid=%s "
                "local_velocity_invalid=%s gcs_connection_lost=%s manual_control_signal_lost=%s fd_alt_loss=%s"
                % (
                    int(flags.battery_warning),
                    bool(flags.battery_low_remaining_time),
                    bool(flags.battery_unhealthy),
                    bool(flags.offboard_control_signal_lost),
                    bool(flags.local_position_invalid),
                    bool(flags.local_altitude_invalid),
                    bool(flags.local_velocity_invalid),
                    bool(flags.gcs_connection_lost),
                    bool(flags.manual_control_signal_lost),
                    bool(flags.fd_alt_loss),
                )
            )
        else:
            print("FAILSAFE_FLAGS unavailable")

        if battery is not None:
            print(
                "BATTERY_STATUS remaining=%.3f warning=%s connected=%s voltage=%.2f current=%.2f current_avg=%.2f"
                % (
                    float(battery.remaining),
                    int(battery.warning),
                    bool(battery.connected),
                    float(battery.voltage_v),
                    float(battery.current_a),
                    float(battery.current_average_a),
                )
            )
        else:
            print("BATTERY_STATUS unavailable")

    def land_and_wait(reason: str) -> bool:
        """Common cleanup for every failure after an ARM command was sent."""
        print(f"SAFETY_CLEANUP reason={reason}")
        deadline = time.time() + max(10.0, args.land_timeout_s)
        last_land = 0.0
        last_print = 0.0
        while time.time() < deadline:
            spin_for(0.05)
            st = state["status"]
            pos = px4_enu()
            now = time.time()
            if st and st.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                altitude = pos[2] if pos else float("nan")
                if pos is None or altitude < args.land_complete_altitude_max_m:
                    print("LAND_COMPLETE alt=%.2f nav=%s armed=%s" % (altitude, st.nav_state, st.arming_state))
                    return True
            if now - last_land > 0.5:
                publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                last_land = now
            if now - last_print > 3.0:
                print(
                    "landing_wait alt=%.2f nav=%s armed=%s failsafe=%s"
                    % (
                        pos[2] if pos else -999.0,
                        st.nav_state if st else -1,
                        st.arming_state if st else -1,
                        bool(st.failsafe) if st else True,
                    )
                )
                last_print = now
            time.sleep(0.05)
        st = state["status"]
        pos = px4_enu()
        print(
            "LAND_TIMEOUT alt=%.2f nav=%s armed=%s"
            % (pos[2] if pos else -999.0, st.nav_state if st else -1, st.arming_state if st else -1),
            file=sys.stderr,
        )
        return False

    wait_deadline = time.time() + 20.0
    while time.time() < wait_deadline:
        spin_for(0.1)
        st = state["status"]
        if st and state["lpos"] and state["odom"] and st.pre_flight_checks_pass:
            break

    if state["status"] is None or state["lpos"] is None or state["odom"] is None:
        missing_topics = []
        if state["status"] is None:
            missing_topics.append("/fmu/out/vehicle_status_v4")
        if state["lpos"] is None:
            missing_topics.append("/fmu/out/vehicle_local_position_v1")
        if state["odom"] is None:
            missing_topics.append("/odom")
        print(
            "Missing initial runtime topics: " + ", ".join(missing_topics),
            file=sys.stderr,
        )
        print(
            "Start PX4 SITL, MicroXRCEAgent, FAST-LIO/odom bridge, Ego-Planner, "
            "and trajectory_interface before running this validator.",
            file=sys.stderr,
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    st = state["status"]
    print(
        "initial status armed=%s nav=%s preflight=%s failsafe=%s"
        % (st.arming_state, st.nav_state, bool(st.pre_flight_checks_pass), bool(st.failsafe))
    )
    print(
        "initial frame planner=%s px4_enu=%s"
        % (tuple(round(v, 3) for v in planner_enu()), tuple(round(v, 3) for v in px4_enu()))
    )

    initial_px4 = px4_enu()
    allowed_initial_nav_states = (
        VehicleStatus.NAVIGATION_STATE_POSCTL,
        VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
    )
    if (
        st.arming_state != VehicleStatus.ARMING_STATE_DISARMED
        or st.nav_state not in allowed_initial_nav_states
        or st.failsafe
        or not st.pre_flight_checks_pass
        or initial_px4[2] > args.initial_ground_altitude_max_m
    ):
        print(
            "Refusing to start from a dirty initial state: "
            "expected disarmed ground POSCTL/AUTO_LOITER without failsafe, got "
            f"armed={st.arming_state} nav={st.nav_state} "
            f"failsafe={bool(st.failsafe)} preflight={bool(st.pre_flight_checks_pass)} "
            f"alt={initial_px4[2]:.2f}",
            file=sys.stderr,
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    initial_planner = planner_enu()
    horizontal_error = math.hypot(initial_planner[0] - initial_px4[0], initial_planner[1] - initial_px4[1])
    vertical_error = abs(initial_planner[2] - initial_px4[2])
    if (
        horizontal_error > args.max_frame_horizontal_error_m
        or vertical_error > args.max_frame_vertical_error_m
    ):
        print(
            "Refusing to start with planner/PX4 frame mismatch: "
            f"horizontal={horizontal_error:.3f} vertical={vertical_error:.3f} "
            f"limits={args.max_frame_horizontal_error_m:.3f}/{args.max_frame_vertical_error_m:.3f}",
            file=sys.stderr,
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    setpoints_before_quiet_check = state["setpoint"]
    spin_for(max(0.25, args.setpoint_quiet_window_s))
    stale_setpoints = state["setpoint"] - setpoints_before_quiet_check
    if stale_setpoints > 0:
        print(
            "Refusing helper takeoff because /fmu/in/trajectory_setpoint is already active "
            f"({stale_setpoints} messages during quiet window). Restart/reset the planning execution chain.",
            file=sys.stderr,
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    lpos = state["lpos"]
    target_hover_x_ned = float(lpos.x)
    target_hover_y_ned = float(lpos.y)
    target_hover_z_ned = -abs(args.takeoff_altitude_m)

    for _ in range(max(10, args.offboard_warmup_cycles)):
        publish_hover_setpoint(target_hover_x_ned, target_hover_y_ned, target_hover_z_ned)
        spin_for(0.05)

    publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
    print("helper sent ARM")
    time.sleep(0.5)
    publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
    print("helper requested OFFBOARD hover takeoff")
    if args.inject_helper_timeout_after_arm:
        print("INJECTED takeoff_helper_timeout", file=sys.stderr)
        cleaned = land_and_wait("injected_helper_timeout")
        node.destroy_node()
        rclpy.shutdown()
        return 2 if cleaned else 3

    takeoff_deadline = time.time() + max(10.0, args.takeoff_timeout_s)
    last_mode_request = 0.0
    while time.time() < takeoff_deadline:
        publish_hover_setpoint(target_hover_x_ned, target_hover_y_ned, target_hover_z_ned)
        spin_for(0.05)
        st = state["status"]
        pos = px4_enu()
        now = time.time()
        if now - last_mode_request > 1.0:
            if st and st.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            if st and st.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            last_mode_request = now
        if (
            st
            and pos
            and st.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and st.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and pos[2] >= args.takeoff_altitude_m - args.takeoff_hover_tolerance_m
        ):
            print("takeoff_stable alt=%.2f nav=%s armed=%s" % (pos[2], st.nav_state, st.arming_state))
            break
        time.sleep(0.05)
    else:
        st = state["status"]
        pos = px4_enu()
        print(
            "takeoff_helper_timeout alt=%.2f nav=%s armed=%s"
            % (pos[2] if pos else -999.0, st.nav_state if st else -1, st.arming_state if st else -1),
            file=sys.stderr,
        )
        cleaned = land_and_wait("takeoff_helper_timeout")
        node.destroy_node()
        rclpy.shutdown()
        return 1 if cleaned else 3

    settle_deadline = time.time() + max(0.5, args.post_takeoff_settle_s)
    while time.time() < settle_deadline:
        publish_hover_setpoint(target_hover_x_ned, target_hover_y_ned, target_hover_z_ned)
        spin_for(0.05)
        time.sleep(0.05)

    try:
        mav = mavutil.mavlink_connection("udp:127.0.0.1:14540", source_system=253)
        mav.wait_heartbeat(timeout=10)
        mav.set_mode("LOITER")
        print("helper requested LOITER handover hold")
    except Exception as exc:
        print(f"failed to request LOITER via MAVLink: {exc}", file=sys.stderr)
        cleaned = land_and_wait("loiter_handover_request_failed")
        node.destroy_node()
        rclpy.shutdown()
        return 1 if cleaned else 3

    handover_deadline = time.time() + 8.0
    while time.time() < handover_deadline:
        spin_for(0.1)
        st = state["status"]
        if (
            st
            and st.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and st.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            break
    else:
        st = state["status"]
        print(
            "failed to leave Offboard before Ego handover: nav=%s armed=%s accepts_offboard=%s"
            % (
                st.nav_state if st else -1,
                st.arming_state if st else -1,
                bool(st.accepts_offboard_setpoints) if st else False,
            ),
            file=sys.stderr,
        )
        cleaned = land_and_wait("failed_to_leave_offboard")
        node.destroy_node()
        rclpy.shutdown()
        return 1 if cleaned else 3

    st = state["status"]
    print(
        "handover_hold nav=%s armed=%s accepts_offboard=%s"
        % (st.nav_state, st.arming_state, bool(st.accepts_offboard_setpoints))
    )

    hover_planner = planner_enu()
    hover_px4 = px4_enu()
    print(
        "hover frame planner=%s px4_enu=%s herr=%.3f verr=%.3f"
        % (
            tuple(round(v, 3) for v in hover_planner),
            tuple(round(v, 3) for v in hover_px4),
            math.hypot(hover_planner[0] - hover_px4[0], hover_planner[1] - hover_px4[1]),
            abs(hover_planner[2] - hover_px4[2]),
        )
    )

    target = (
        hover_planner[0] + args.goal_forward_m,
        hover_planner[1],
        hover_planner[2],
    )
    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.pose.orientation.w = 1.0
    goal.pose.position.x = target[0]
    goal.pose.position.y = target[1]
    goal.pose.position.z = target[2]
    print(
        "published_small_goal x=%.3f y=%.3f z=%.3f start=%s"
        % (target[0], target[1], target[2], tuple(round(v, 3) for v in hover_planner))
    )

    initial_counts = (state["bspline"], state["pcmd"], state["setpoint"])
    offboard_seen = False
    min_goal_dist = float("inf")
    max_move = 0.0
    max_alt = hover_planner[2]
    last_print = 0.0
    handover_start = time.time()
    while time.time() - handover_start < max(5.0, args.handover_timeout_s):
        goal.header.stamp = node.get_clock().now().to_msg()
        goal_pub.publish(goal)
        spin_for(0.05)
        st = state["status"]
        planner = planner_enu()
        if st and planner:
            offboard_seen = offboard_seen or (st.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
            goal_dist = _dist(planner, target)
            move = _dist(planner, hover_planner)
            min_goal_dist = min(min_goal_dist, goal_dist)
            max_move = max(max_move, move)
            max_alt = max(max_alt, planner[2])
            elapsed = time.time() - handover_start
            if elapsed - last_print > 2.0:
                print(
                    "t=%.1f nav=%s armed=%s offboard=%s pos=%s move=%.2f goal_dist=%.2f bs+%d pc+%d sp+%d"
                    % (
                        elapsed,
                        st.nav_state,
                        st.arming_state,
                        offboard_seen,
                        tuple(round(v, 2) for v in planner),
                        move,
                        goal_dist,
                        state["bspline"] - initial_counts[0],
                        state["pcmd"] - initial_counts[1],
                        state["setpoint"] - initial_counts[2],
                    )
                )
                last_print = elapsed
            if offboard_seen and goal_dist < args.target_threshold_m and elapsed > 4.0:
                break
            if st.failsafe:
                print("failsafe detected")
                print_failsafe_context()
                break
        time.sleep(0.05)

    st = state["status"]
    planner = planner_enu()
    reasons = []
    if not offboard_seen:
        reasons.append("no_offboard")
    if min_goal_dist >= args.target_threshold_m:
        reasons.append("target_not_reached")
    if max_move > args.max_move_pass_m:
        reasons.append("overshoot")
    if max_alt > args.max_alt_pass_m:
        reasons.append("altitude_excursion")
    if st and st.failsafe:
        reasons.append("failsafe")

    passed = not reasons
    print(
        "SMALL_GOAL_RESULT passed=%s bad=%s offboard_seen=%s min_goal_dist=%.3f max_move=%.3f max_alt=%.3f final=%s nav=%s armed=%s failsafe=%s bs+%d pc+%d sp+%d"
        % (
            passed,
            ",".join(reasons) if reasons else "none",
            offboard_seen,
            min_goal_dist,
            max_move,
            max_alt,
            tuple(round(v, 3) for v in planner),
            st.nav_state if st else -1,
            st.arming_state if st else -1,
            bool(st.failsafe) if st else True,
            state["bspline"] - initial_counts[0],
            state["pcmd"] - initial_counts[1],
            state["setpoint"] - initial_counts[2],
        )
    )

    print("sent_land")
    cleaned = land_and_wait("normal_test_completion")
    node.destroy_node()
    rclpy.shutdown()
    if not cleaned:
        return 3
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
