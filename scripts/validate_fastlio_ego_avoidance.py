#!/usr/bin/env python3
import argparse
import math
import sys
import time

MID_OBSTACLE_MIN = (-2.75, -2.10, 0.0)
MID_OBSTACLE_MAX = (-1.65, -0.90, 2.0)


def dist_to_aabb(point, aabb_min, aabb_max):
    dx = max(aabb_min[0] - point[0], 0.0, point[0] - aabb_max[0])
    dy = max(aabb_min[1] - point[1], 0.0, point[1] - aabb_max[1])
    dz = max(aabb_min[2] - point[2], 0.0, point[2] - aabb_max[2])
    outside = math.sqrt(dx * dx + dy * dy + dz * dz)
    inside = (
        aabb_min[0] <= point[0] <= aabb_max[0]
        and aabb_min[1] <= point[1] <= aabb_max[1]
        and aabb_min[2] <= point[2] <= aabb_max[2]
    )
    return -outside if inside else outside


def dist_to_target(point, target):
    return math.sqrt(
        (point[0] - target[0]) ** 2
        + (point[1] - target[1]) ** 2
        + (point[2] - target[2]) ** 2
    )


def line_intersects_aabb_xy(start, goal, aabb_min, aabb_max, samples=200):
    for i in range(samples + 1):
        t = float(i) / float(samples)
        x = start[0] + ((goal[0] - start[0]) * t)
        y = start[1] + ((goal[1] - start[1]) * t)
        if aabb_min[0] <= x <= aabb_max[0] and aabb_min[1] <= y <= aabb_max[1]:
            return True
    return False


def line_aabb_xy_overlap_score(start, goal, aabb_min, aabb_max, samples=200):
    inside_count = 0
    for i in range(samples + 1):
        t = float(i) / float(samples)
        x = start[0] + ((goal[0] - start[0]) * t)
        y = start[1] + ((goal[1] - start[1]) * t)
        if aabb_min[0] <= x <= aabb_max[0] and aabb_min[1] <= y <= aabb_max[1]:
            inside_count += 1
    return inside_count


def path_intersects_aabb_xy(samples, aabb_min, aabb_max):
    for _, point, _, _ in samples:
        if aabb_min[0] <= point[0] <= aabb_max[0] and aabb_min[1] <= point[1] <= aabb_max[1]:
            return True
    return False


def xy_clearance_to_aabb(point, aabb_min, aabb_max):
    dx = max(aabb_min[0] - point[0], 0.0, point[0] - aabb_max[0])
    dy = max(aabb_min[1] - point[1], 0.0, point[1] - aabb_max[1])
    inside = aabb_min[0] <= point[0] <= aabb_max[0] and aabb_min[1] <= point[1] <= aabb_max[1]
    clearance = math.sqrt(dx * dx + dy * dy)
    return -clearance if inside else clearance


def points_on_opposite_x_sides(start, goal, aabb_min, aabb_max):
    return (start[0] > aabb_max[0] and goal[0] < aabb_min[0]) or (
        start[0] < aabb_min[0] and goal[0] > aabb_max[0]
    )


def choose_cross_obstacle_target(current_point, target_z):
    obstacle_mid_y = (MID_OBSTACLE_MIN[1] + MID_OBSTACLE_MAX[1]) / 2.0
    current_x = current_point[0]
    obstacle_center_x = (MID_OBSTACLE_MIN[0] + MID_OBSTACLE_MAX[0]) / 2.0

    if current_x <= obstacle_center_x:
        candidate_x = MID_OBSTACLE_MAX[0] + 2.8
        far_x = MID_OBSTACLE_MAX[0] + 4.0
    else:
        candidate_x = MID_OBSTACLE_MIN[0] - 2.8
        far_x = MID_OBSTACLE_MIN[0] - 4.0

    candidates = [
        (candidate_x, obstacle_mid_y, target_z),
        (candidate_x, obstacle_mid_y + 0.7, target_z),
        (candidate_x, obstacle_mid_y - 0.7, target_z),
        (candidate_x, obstacle_mid_y + 1.2, target_z),
        (candidate_x, obstacle_mid_y - 1.2, target_z),
        (candidate_x - 0.4, obstacle_mid_y, target_z),
        (candidate_x + 0.4, obstacle_mid_y, target_z),
        (far_x, obstacle_mid_y, target_z),
        (far_x, obstacle_mid_y + 0.7, target_z),
        (far_x, obstacle_mid_y - 0.7, target_z),
    ]

    best_candidate = None
    best_score = (-1, -1.0, -1.0)
    for candidate in candidates:
        if dist_to_target(current_point, candidate) <= 3.0:
            continue
        if not points_on_opposite_x_sides(current_point, candidate, MID_OBSTACLE_MIN, MID_OBSTACLE_MAX):
            continue
        overlap_score = line_aabb_xy_overlap_score(
            current_point,
            candidate,
            MID_OBSTACLE_MIN,
            MID_OBSTACLE_MAX,
        )
        target_clearance = xy_clearance_to_aabb(candidate, MID_OBSTACLE_MIN, MID_OBSTACLE_MAX)
        # Prefer a route whose straight-line reference truly cuts through the obstacle,
        # then prefer a clear landing zone behind it.
        score = (overlap_score, target_clearance, dist_to_target(current_point, candidate))
        if score > best_score:
            best_score = score
            best_candidate = candidate

    if best_candidate is not None and best_score[0] > 0:
        return best_candidate

    return candidates[0]


def choose_short_chain_target(current_point, distance, z_offset, min_z):
    return (
        current_point[0] + distance,
        current_point[1],
        max(min_z, current_point[2] + z_offset),
    )


def main():
    parser = argparse.ArgumentParser(
        description="Run a PX4 SITL FAST-LIO + Ego-Planner obstacle avoidance validation."
    )
    parser.add_argument("--self-test", action="store_true", help="Run offline logic checks and exit.")
    parser.add_argument("--target-x", type=float, default=None)
    parser.add_argument("--target-y", type=float, default=None)
    parser.add_argument("--target-z", type=float, default=1.5)
    parser.add_argument(
        "--target-frame",
        choices=("px4_enu", "planner"),
        default="px4_enu",
        help="Frame for --target-x/--target-y/--target-z. px4_enu is the physical Gazebo/PX4 ENU frame used by safety checks.",
    )
    parser.add_argument("--takeoff-altitude-m", type=float, default=1.5)
    parser.add_argument("--takeoff-timeout-s", type=float, default=35.0)
    parser.add_argument("--takeoff-hover-tolerance-m", type=float, default=0.15)
    parser.add_argument("--post-takeoff-settle-s", type=float, default=2.0)
    parser.add_argument("--initial-ground-altitude-max-m", type=float, default=0.30)
    parser.add_argument("--land-complete-altitude-max-m", type=float, default=0.80)
    parser.add_argument("--offboard-warmup-cycles", type=int, default=30)
    parser.add_argument("--chain-goal-distance", type=float, default=0.6)
    parser.add_argument("--chain-goal-z-offset", type=float, default=0.3)
    parser.add_argument("--chain-goal-min-z", type=float, default=0.5)
    parser.add_argument("--required-clearance", type=float, default=0.45)
    parser.add_argument(
        "--landing-zone-clearance",
        type=float,
        default=0.80,
        help="Required XY clearance between the final/landing target and the obstacle AABB.",
    )
    parser.add_argument(
        "--require-obstacle-between-start-and-goal",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Require the start-to-goal straight line to intersect the obstacle and the goal to be behind it.",
    )
    parser.add_argument("--target-threshold", type=float, default=0.45)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--initial-pose-timeout", type=float, default=15.0)
    parser.add_argument("--arm-wait-timeout", type=float, default=8.0)
    parser.add_argument("--preflight-wait-timeout", type=float, default=12.0)
    parser.add_argument(
        "--land-on-exit",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="After flight validation, command PX4 LAND and wait for disarm. Ignored with --no-arm.",
    )
    parser.add_argument("--land-wait-timeout", type=float, default=45.0)
    parser.add_argument(
        "--takeover-after-s",
        type=float,
        default=0.0,
        help="During an active Ego trajectory, send an external takeover command after this many seconds.",
    )
    parser.add_argument(
        "--takeover-command",
        choices=("land", "rtl", "disarm"),
        default="land",
        help="External takeover command used with --takeover-after-s.",
    )
    parser.add_argument("--no-arm", action="store_true")
    parser.add_argument("--require-chain", action="store_true")
    parser.add_argument("--goal-topic", default="/move_base_simple/goal")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--bspline-topic", default="/planning/bspline")
    parser.add_argument("--position-cmd-topic", default="/planning/position_cmd")
    parser.add_argument(
        "--require-frame-alignment",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Require planner odom and PX4 local ENU to be aligned before publishing planner goals.",
    )
    parser.add_argument("--max-frame-horizontal-error", type=float, default=1.0)
    parser.add_argument("--max-frame-vertical-error", type=float, default=0.6)
    parser.add_argument(
        "--min-goal-distance",
        type=float,
        default=2.0,
        help="Reject or replace validation targets that are already too close to the current vehicle position",
    )
    parser.add_argument(
        "--allow-preexisting-setpoints",
        action="store_true",
        help="Allow flight validation to start even if /fmu/in/trajectory_setpoint is already active before helper takeoff.",
    )
    args = parser.parse_args()

    if args.self_test:
        near_start_left = choose_cross_obstacle_target((-4.9, -1.5, 1.5), 1.5)
        near_start_right = choose_cross_obstacle_target((-1.0, -1.5, 1.5), 1.5)
        assert line_intersects_aabb_xy((-4.9, -1.5), near_start_left, MID_OBSTACLE_MIN, MID_OBSTACLE_MAX)
        assert line_intersects_aabb_xy((-1.0, -1.5), near_start_right, MID_OBSTACLE_MIN, MID_OBSTACLE_MAX)
        assert dist_to_target((-4.9, -1.5, 1.5), near_start_left) > 3.0
        assert dist_to_target((-1.0, -1.5, 1.5), near_start_right) > 3.0
        assert line_intersects_aabb_xy((-4.9, -1.5), (1.0, -1.5), MID_OBSTACLE_MIN, MID_OBSTACLE_MAX)
        print("SELF_TEST_OK")
        return

    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from px4_msgs.msg import OffboardControlMode
        from px4_msgs.msg import TrajectorySetpoint
        from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from traj_utils.msg import Bspline
        from quadrotor_msgs.msg import PositionCommand
    except ModuleNotFoundError as exc:
        if exc.name == "quadrotor_msgs":
            print(
                "Missing ROS 2 message package 'quadrotor_msgs'. "
                "Run 'source /opt/ros/humble/setup.bash && source /home/p/px4_ros2_ws/install/setup.bash' "
                "before executing this script.",
                file=sys.stderr,
            )
            sys.exit(1)
        raise

    if PositionCommand is None:
        print(
            "Missing ROS 2 message package 'quadrotor_msgs'. "
            "Run 'source /opt/ros/humble/setup.bash && source /home/p/px4_ros2_ws/install/setup.bash' "
            "before executing this script.",
            file=sys.stderr,
        )
        sys.exit(1)

    target = None
    validation_target = None
    samples = []
    status = {"arming_state": None, "nav_state": None, "failsafe": None, "preflight": None}
    frame_state = {"px4_enu": None, "planner_enu": None}
    chain_state = {
        "odom_received": False,
        "goal_publish_count": 0,
        "bspline_received": False,
        "position_cmd_received": False,
        "bspline_has_publisher": False,
        "position_cmd_has_publisher": False,
        "preexisting_setpoints": 0,
    }

    rclpy.init()
    node = rclpy.create_node("fastlio_ego_avoidance_validation")
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    goal_pub = node.create_publisher(PoseStamped, args.goal_topic, 10)
    cmd_pub = node.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
    offboard_pub = node.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
    setpoint_pub = node.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)

    def local_cb(msg):
        # PX4 local position is NED. Planner world here is ENU.
        point = (float(msg.y), float(msg.x), float(-msg.z))
        frame_state["px4_enu"] = point
        nonlocal target, validation_target
        if target is None and validation_target is None:
            if args.target_x is not None and args.target_y is not None:
                if args.target_frame == "planner":
                    target = (args.target_x, args.target_y, args.target_z)
                else:
                    validation_target = (args.target_x, args.target_y, args.target_z)
            elif args.no_arm:
                target = choose_short_chain_target(
                    point,
                    args.chain_goal_distance,
                    args.chain_goal_z_offset,
                    args.chain_goal_min_z,
                )
                validation_target = target
        target_for_sample = validation_target if validation_target is not None else target if target is not None else point
        samples.append(
            (
                time.time(),
                point,
                dist_to_aabb(point, MID_OBSTACLE_MIN, MID_OBSTACLE_MAX),
                dist_to_target(point, target_for_sample),
            )
        )

    def status_cb(msg):
        status["arming_state"] = int(msg.arming_state)
        status["nav_state"] = int(msg.nav_state)
        status["failsafe"] = bool(msg.failsafe)
        status["preflight"] = bool(msg.pre_flight_checks_pass)

    def odom_cb(msg):
        point = msg.pose.pose.position
        frame_state["planner_enu"] = (float(point.x), float(point.y), float(point.z))
        chain_state["odom_received"] = True

    def bspline_cb(_msg):
        chain_state["bspline_received"] = True

    def position_cmd_cb(_msg):
        chain_state["position_cmd_received"] = True

    node.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", local_cb, qos)
    node.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4", status_cb, qos)
    node.create_subscription(Odometry, args.odom_topic, odom_cb, 10)
    node.create_subscription(Bspline, args.bspline_topic, bspline_cb, 10)
    node.create_subscription(PositionCommand, args.position_cmd_topic, position_cmd_cb, 10)
    node.create_subscription(
        TrajectorySetpoint,
        "/fmu/in/trajectory_setpoint",
        lambda _msg: chain_state.__setitem__("preexisting_setpoints", chain_state["preexisting_setpoints"] + 1),
        10,
    )

    def refresh_topic_presence():
        try:
            bspline_names = node.get_publishers_info_by_topic(args.bspline_topic)
            pos_cmd_names = node.get_publishers_info_by_topic(args.position_cmd_topic)
            chain_state["bspline_has_publisher"] = len(bspline_names) > 0
            chain_state["position_cmd_has_publisher"] = len(pos_cmd_names) > 0
        except Exception:
            pass

    def publish_goal():
        if target is None:
            return
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target
        msg.pose.orientation.w = 1.0
        goal_pub.publish(msg)
        chain_state["goal_publish_count"] += 1

    def publish_hover_setpoint(x_ned, y_ned, z_ned):
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

    def publish_vehicle_command(command_id, param1=0.0, param2=0.0, param7=0.0):
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
        cmd_pub.publish(cmd)

    def publish_land():
        cmd = VehicleCommand()
        cmd.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        cmd.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd_pub.publish(cmd)

    def publish_takeover_command():
        if args.takeover_command == "land":
            publish_land()
            return
        if args.takeover_command == "rtl":
            publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
            return
        if args.takeover_command == "disarm":
            publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
            return

    def px4_enu():
        return frame_state["px4_enu"]

    def planner_enu():
        return frame_state["planner_enu"]

    def spin_for(duration_s):
        end = time.time() + duration_s
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def run_helper_takeoff_and_loiter_handover():
        try:
            from pymavlink import mavutil
        except ModuleNotFoundError as exc:
            print(f"Missing runtime dependency: {exc}", file=sys.stderr)
            return False

        setpoints_before_check = chain_state["preexisting_setpoints"]
        spin_for(0.5)
        preexisting_setpoints = chain_state["preexisting_setpoints"] - setpoints_before_check
        if preexisting_setpoints > 0 and not args.allow_preexisting_setpoints:
            print(
                "Refusing helper takeoff because /fmu/in/trajectory_setpoint is already active. "
                "This usually means Ego-Planner/trajectory_interface still has a previous goal. "
                "Restart the Ego-Planner launch, wait for PX4 to be disarmed in Loiter/Position, "
                "then rerun flight validation.",
                file=sys.stderr,
            )
            return False

        allowed_initial_nav_states = (
            VehicleStatus.NAVIGATION_STATE_POSCTL,
            VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
        )
        current_status = status.copy()
        current_px4 = px4_enu()
        if (
            current_status["arming_state"] != VehicleStatus.ARMING_STATE_DISARMED
            or current_status["nav_state"] not in allowed_initial_nav_states
            or current_status["failsafe"]
            or current_px4 is None
            or current_px4[2] > args.initial_ground_altitude_max_m
        ):
            print(
                "Refusing to start from a dirty initial state: "
                "expected disarmed ground POSCTL/AUTO_LOITER without failsafe, got "
                f"armed={current_status['arming_state']} nav={current_status['nav_state']} "
                f"failsafe={current_status['failsafe']} alt={current_px4[2] if current_px4 else float('nan'):.2f}",
                file=sys.stderr,
            )
            return False

        # Use PX4 NED local position directly for the helper hover setpoint.
        last_lpos = {"x": None, "y": None}

        def capture_lpos(msg):
            last_lpos["x"] = float(msg.x)
            last_lpos["y"] = float(msg.y)

        lpos_sub = node.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", capture_lpos, qos)
        spin_for(0.5)
        if last_lpos["x"] is None or last_lpos["y"] is None:
            node.destroy_subscription(lpos_sub)
            print("Missing PX4 local position for helper takeoff.", file=sys.stderr)
            return False

        target_hover_x_ned = last_lpos["x"]
        target_hover_y_ned = last_lpos["y"]
        target_hover_z_ned = -abs(args.takeoff_altitude_m)

        for _ in range(max(10, args.offboard_warmup_cycles)):
            publish_hover_setpoint(target_hover_x_ned, target_hover_y_ned, target_hover_z_ned)
            spin_for(0.05)

        publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        print("helper sent ARM")
        time.sleep(0.5)
        publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        print("helper requested OFFBOARD hover takeoff")

        takeoff_deadline = time.time() + max(10.0, args.takeoff_timeout_s)
        last_mode_request = 0.0
        while time.time() < takeoff_deadline:
            publish_hover_setpoint(target_hover_x_ned, target_hover_y_ned, target_hover_z_ned)
            spin_for(0.05)
            current_status = status.copy()
            current_px4 = px4_enu()
            now = time.time()
            if now - last_mode_request > 1.0:
                if current_status["arming_state"] != VehicleStatus.ARMING_STATE_ARMED:
                    publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                if current_status["nav_state"] != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                last_mode_request = now
            if (
                current_status["arming_state"] == VehicleStatus.ARMING_STATE_ARMED
                and current_status["nav_state"] == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and current_px4 is not None
                and current_px4[2] >= args.takeoff_altitude_m - args.takeoff_hover_tolerance_m
            ):
                print(
                    "takeoff_stable alt=%.2f nav=%s armed=%s"
                    % (current_px4[2], current_status["nav_state"], current_status["arming_state"])
                )
                break
            time.sleep(0.05)
        else:
            current_status = status.copy()
            current_px4 = px4_enu()
            node.destroy_subscription(lpos_sub)
            print(
                "takeoff_helper_timeout alt=%.2f nav=%s armed=%s"
                % (
                    current_px4[2] if current_px4 else -999.0,
                    current_status["nav_state"],
                    current_status["arming_state"],
                ),
                file=sys.stderr,
            )
            return False

        settle_deadline = time.time() + max(0.5, args.post_takeoff_settle_s)
        while time.time() < settle_deadline:
            publish_hover_setpoint(target_hover_x_ned, target_hover_y_ned, target_hover_z_ned)
            spin_for(0.05)
            time.sleep(0.05)

        mav = mavutil.mavlink_connection("udp:127.0.0.1:14540", source_system=253)
        mav.wait_heartbeat(timeout=10)
        try:
            mav.set_mode("LOITER")
            print("helper requested LOITER handover hold")
        except Exception as exc:
            node.destroy_subscription(lpos_sub)
            print(f"failed to request LOITER via MAVLink: {exc}", file=sys.stderr)
            return False

        handover_deadline = time.time() + 8.0
        while time.time() < handover_deadline:
            spin_for(0.1)
            current_status = status.copy()
            if (
                current_status["arming_state"] == VehicleStatus.ARMING_STATE_ARMED
                and current_status["nav_state"] != VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                break
        else:
            current_status = status.copy()
            node.destroy_subscription(lpos_sub)
            print(
                "failed to leave Offboard before Ego handover: "
                f"nav={current_status['nav_state']} armed={current_status['arming_state']}",
                file=sys.stderr,
            )
            return False

        node.destroy_subscription(lpos_sub)
        current_status = status.copy()
        print("handover_hold nav=%s armed=%s" % (current_status["nav_state"], current_status["arming_state"]))
        return True

    def frame_alignment_ok():
        px4_enu = frame_state["px4_enu"]
        planner_enu = frame_state["planner_enu"]
        if not args.require_frame_alignment:
            return True
        if px4_enu is None or planner_enu is None:
            print(
                "FRAME_ALIGNMENT missing "
                f"px4_enu={px4_enu is not None} planner_enu={planner_enu is not None}",
                file=sys.stderr,
            )
            return False
        horizontal_error = math.hypot(
            planner_enu[0] - px4_enu[0],
            planner_enu[1] - px4_enu[1],
        )
        vertical_error = abs(planner_enu[2] - px4_enu[2])
        print(
            "FRAME_ALIGNMENT "
            f"planner_enu={tuple(round(value, 3) for value in planner_enu)} "
            f"px4_enu={tuple(round(value, 3) for value in px4_enu)} "
            f"horizontal_error={horizontal_error:.3f} "
            f"vertical_error={vertical_error:.3f}"
        )
        if (
            horizontal_error > args.max_frame_horizontal_error
            or vertical_error > args.max_frame_vertical_error
        ):
            print(
                "Planner odom and PX4 local ENU are not aligned; refusing to publish flight goals. "
                f"limits=({args.max_frame_horizontal_error:.3f} m horizontal, "
                f"{args.max_frame_vertical_error:.3f} m vertical)",
                file=sys.stderr,
            )
            return False
        return True

    def wait_for_landed_disarmed():
        if args.no_arm or not args.land_on_exit:
            return

        print("Sending LAND command and waiting for disarm")
        deadline = time.time() + max(5.0, args.land_wait_timeout)
        last_land = 0.0
        last_print = 0.0
        while time.time() < deadline:
            now = time.time()
            if now - last_land > 0.5:
                publish_land()
                last_land = now
            rclpy.spin_once(node, timeout_sec=0.05)
            current_alt = samples[-1][1][2] if samples else float("nan")
            if now - last_print > 2.0:
                print(
                    f"landing_wait armed={status['arming_state']} nav={status['nav_state']} "
                    f"alt_enu={current_alt:.2f} failsafe={status['failsafe']}"
                )
                last_print = now
            if (
                status["arming_state"] == VehicleStatus.ARMING_STATE_DISARMED
                and (not samples or current_alt < args.land_complete_altitude_max_m)
            ):
                print("LAND_COMPLETE")
                return
            time.sleep(0.05)

        print(
            "LAND_TIMEOUT "
            f"arming_state={status['arming_state']} nav_state={status['nav_state']} "
            f"failsafe={status['failsafe']}",
            file=sys.stderr,
        )

    warmup_deadline = time.time() + max(2.0, args.initial_pose_timeout)
    while time.time() < warmup_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if (
            target is not None
            and status["arming_state"] is not None
            and (not args.require_frame_alignment or frame_state["planner_enu"] is not None)
        ):
            break
        time.sleep(0.1)

    if args.no_arm and target is None:
        print("Failed to determine a validation target from current vehicle position.")
        node.destroy_node()
        rclpy.shutdown()
        return

    if not args.no_arm:
        if not run_helper_takeoff_and_loiter_handover():
            wait_for_landed_disarmed()
            node.destroy_node()
            rclpy.shutdown()
            return
        spin_for(0.5)
        hover_planner = planner_enu()
        hover_px4 = px4_enu()
        if hover_planner is None or hover_px4 is None:
            print("Missing planner/PX4 pose after helper handover.", file=sys.stderr)
            wait_for_landed_disarmed()
            node.destroy_node()
            rclpy.shutdown()
            return
        if target is None:
            if validation_target is None:
                validation_target = choose_cross_obstacle_target(hover_px4, args.target_z)
            frame_offset = (
                hover_planner[0] - hover_px4[0],
                hover_planner[1] - hover_px4[1],
                hover_planner[2] - hover_px4[2],
            )
            target = (
                validation_target[0] + frame_offset[0],
                validation_target[1] + frame_offset[1],
                validation_target[2] + frame_offset[2],
            )
        elif validation_target is None:
            frame_offset = (
                hover_planner[0] - hover_px4[0],
                hover_planner[1] - hover_px4[1],
                hover_planner[2] - hover_px4[2],
            )
            validation_target = (
                target[0] - frame_offset[0],
                target[1] - frame_offset[1],
                target[2] - frame_offset[2],
            )
        elif args.target_frame == "px4_enu":
            frame_offset = (
                hover_planner[0] - hover_px4[0],
                hover_planner[1] - hover_px4[1],
                hover_planner[2] - hover_px4[2],
            )
            target = (
                validation_target[0] + frame_offset[0],
                validation_target[1] + frame_offset[1],
                validation_target[2] + frame_offset[2],
            )
        samples.clear()
        print(
            "hover frame planner=%s px4_enu=%s herr=%.3f verr=%.3f"
            % (
                tuple(round(value, 3) for value in hover_planner),
                tuple(round(value, 3) for value in hover_px4),
                math.hypot(hover_planner[0] - hover_px4[0], hover_planner[1] - hover_px4[1]),
                abs(hover_planner[2] - hover_px4[2]),
            )
        )

    if samples:
        initial_target_dist = dist_to_target(samples[-1][1], validation_target)
        if (not args.no_arm) and initial_target_dist < args.min_goal_distance:
            print(
                f"Target is already too close to current position: initial_target_dist={initial_target_dist:.3f} m < min_goal_distance={args.min_goal_distance:.3f} m",
                file=sys.stderr,
            )
            print(
                "Move the vehicle away from the current goal, restart the sim, or provide a farther --target-x/--target-y.",
                file=sys.stderr,
            )
            node.destroy_node()
            rclpy.shutdown()
            return

    if not frame_alignment_ok():
        node.destroy_node()
        rclpy.shutdown()
        return

    if args.no_arm:
        print(f"Short chain target: {validation_target}")
    else:
        print(f"Planner goal: {target}")
        print(f"Validation target crosses mid_obstacle if flown straight: {validation_target}")

    print(f"Initial status: {status}")

    chain_deadline = time.time() + max(5.0, args.initial_pose_timeout)
    while time.time() < chain_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        refresh_topic_presence()
        publish_goal()
        if all(
            [
                chain_state["odom_received"],
                chain_state["goal_publish_count"] > 0,
                (chain_state["position_cmd_received"] or chain_state["position_cmd_has_publisher"]),
            ]
        ):
            break
        time.sleep(0.05)

    print("CHAIN_STATUS")
    print("odom_received", chain_state["odom_received"])
    print("goal_publish_count", chain_state["goal_publish_count"])
    print("bspline_received", chain_state["bspline_received"])
    print("position_cmd_received", chain_state["position_cmd_received"])
    print("bspline_has_publisher", chain_state["bspline_has_publisher"])
    print("position_cmd_has_publisher", chain_state["position_cmd_has_publisher"])

    chain_ok = all(
        [
            chain_state["odom_received"],
            chain_state["goal_publish_count"] > 0,
            (chain_state["position_cmd_received"] or chain_state["position_cmd_has_publisher"]),
        ]
    )
    if args.require_chain and not chain_ok:
        print("Required planning chain is incomplete, aborting flight validation.")
        wait_for_landed_disarmed()
        node.destroy_node()
        rclpy.shutdown()
        return

    if args.no_arm:
        print("NO_ARM_CHAIN_ONLY")
        print("chain_ok", chain_ok)
        node.destroy_node()
        rclpy.shutdown()
        return

    start = time.time()
    last_print = -10.0
    initial_flight_point = samples[-1][1] if samples else None
    takeover_sent = False
    takeover_nav_seen = False
    while time.time() - start < args.timeout:
        elapsed = time.time() - start
        if args.takeover_after_s > 0.0 and not takeover_sent and elapsed >= args.takeover_after_s:
            publish_takeover_command()
            takeover_sent = True
            print(f"TAKEOVER_SENT command={args.takeover_command} elapsed={elapsed:.1f}s")
        if elapsed < 20.0 and not takeover_sent:
            publish_goal()
        rclpy.spin_once(node, timeout_sec=0.05)
        refresh_topic_presence()
        if takeover_sent:
            takeover_nav_seen = takeover_nav_seen or status["nav_state"] != VehicleStatus.NAVIGATION_STATE_OFFBOARD
            if status["arming_state"] == VehicleStatus.ARMING_STATE_DISARMED:
                print("TAKEOVER_DISARMED")
                break
        if samples and elapsed - last_print > 2.0:
            _, point, box_dist, target_dist = samples[-1]
            print(
                f"t={elapsed:5.1f}s "
                f"pos_enu=({point[0]: .2f},{point[1]: .2f},{point[2]: .2f}) "
                f"dist_to_box={box_dist: .2f} target_dist={target_dist: .2f} "
                f"armed={status['arming_state']} nav={status['nav_state']} failsafe={status['failsafe']}"
            )
            last_print = elapsed
        if samples and samples[-1][3] < args.target_threshold and elapsed > 10.0:
            print("Target reached threshold.")
            break
        if takeover_sent and elapsed > args.takeover_after_s + 12.0:
            print(
                "TAKEOVER_MONITOR_DONE "
                f"nav_seen={takeover_nav_seen} armed={status['arming_state']} nav={status['nav_state']}"
            )
            break
        if (
            initial_flight_point is not None
            and elapsed > 15.0
            and dist_to_target(samples[-1][1], initial_flight_point) < 0.5
            and samples[-1][3] > max(args.target_threshold * 2.0, 2.0)
            and not takeover_sent
        ):
            print(
                "Vehicle did not make meaningful progress toward the obstacle-crossing target. "
                f"arming_state={status['arming_state']} preflight={status['preflight']} nav_state={status['nav_state']}",
                file=sys.stderr,
            )
            break
        if status["failsafe"]:
            print("Failsafe detected, stopping test loop.")
            break
        time.sleep(0.05)

    if samples:
        min_box_dist = min(sample[2] for sample in samples)
        min_target_dist = min(sample[3] for sample in samples)
        final_point = samples[-1][1]
        max_alt = max(sample[1][2] for sample in samples)
        path_length = 0.0
        for idx in range(1, len(samples)):
            prev = samples[idx - 1][1]
            curr = samples[idx][1]
            path_length += dist_to_target(prev, curr)
        straight_line_distance = dist_to_target(samples[0][1], validation_target)
        straight_line_crosses_obstacle = line_intersects_aabb_xy(
            samples[0][1],
            validation_target,
            MID_OBSTACLE_MIN,
            MID_OBSTACLE_MAX,
        )
        target_behind_obstacle = points_on_opposite_x_sides(
            samples[0][1],
            validation_target,
            MID_OBSTACLE_MIN,
            MID_OBSTACLE_MAX,
        )
        target_landing_clearance = xy_clearance_to_aabb(validation_target, MID_OBSTACLE_MIN, MID_OBSTACLE_MAX)
        final_landing_clearance = xy_clearance_to_aabb(final_point, MID_OBSTACLE_MIN, MID_OBSTACLE_MAX)
        obstacle_touched = path_intersects_aabb_xy(samples, MID_OBSTACLE_MIN, MID_OBSTACLE_MAX)
        entered = min_box_dist < 0.0
        crossed_obstacle_side = (
            (samples[0][1][0] < MID_OBSTACLE_MIN[0] and final_point[0] > MID_OBSTACLE_MAX[0])
            or (samples[0][1][0] > MID_OBSTACLE_MAX[0] and final_point[0] < MID_OBSTACLE_MIN[0])
        )
        obstacle_between_ok = (
            not args.require_obstacle_between_start_and_goal
            or (straight_line_crosses_obstacle and target_behind_obstacle)
        )
        passed = (
            obstacle_between_ok
            and min_box_dist >= args.required_clearance
            and min_target_dist < args.target_threshold
            and not entered
            and not obstacle_touched
            and crossed_obstacle_side
            and target_landing_clearance >= args.landing_zone_clearance
            and final_landing_clearance >= args.landing_zone_clearance
            and not status["failsafe"]
            and path_length >= max(1.2, straight_line_distance + 0.5)
        )
        if takeover_sent:
            passed = takeover_nav_seen and not status["failsafe"]
        print("SUMMARY")
        print("samples", len(samples))
        print("final_pos_enu", tuple(round(value, 3) for value in final_point))
        print("min_dist_to_mid_obstacle_aabb_m", round(min_box_dist, 3))
        print("required_clearance_m", args.required_clearance)
        print("min_target_dist_m", round(min_target_dist, 3))
        print("path_length_m", round(path_length, 3))
        print("straight_line_distance_m", round(straight_line_distance, 3))
        print("straight_line_crosses_obstacle", straight_line_crosses_obstacle)
        print("target_behind_obstacle", target_behind_obstacle)
        print("target_landing_clearance_m", round(target_landing_clearance, 3))
        print("final_landing_clearance_m", round(final_landing_clearance, 3))
        print("landing_zone_clearance_required_m", args.landing_zone_clearance)
        print("max_alt_m", round(max_alt, 3))
        print("entered_obstacle_aabb", entered)
        print("obstacle_xy_touched", obstacle_touched)
        print("crossed_obstacle_side", crossed_obstacle_side)
        print("chain_ok", chain_ok)
        if takeover_sent:
            print("takeover_command", args.takeover_command)
            print("takeover_nav_seen", takeover_nav_seen)
            print("TAKEOVER_TEST", passed)
        else:
            print("PASS_CLEARANCE_TEST", passed)
    else:
        print("SUMMARY: no local position samples received")

    wait_for_landed_disarmed()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
