from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_fastlio_bridge = LaunchConfiguration("use_fastlio_bridge")
    use_sim_bridge = LaunchConfiguration("use_sim_bridge")
    use_depth_camera_fastlio = LaunchConfiguration("use_depth_camera_fastlio")
    use_native_3d_pointcloud = LaunchConfiguration("use_native_3d_pointcloud")
    use_scan_fallback = LaunchConfiguration("use_scan_fallback")
    fixed_goal_altitude_m = LaunchConfiguration("fixed_goal_altitude_m")
    lio_odom_topic = LaunchConfiguration("lio_odom_topic")
    planner_odom_topic = LaunchConfiguration("planner_odom_topic")
    local_map_topic = LaunchConfiguration("local_map_topic")
    position_cmd_topic = LaunchConfiguration("position_cmd_topic")
    planner_goal_topic = LaunchConfiguration("planner_goal_topic")
    vehicle_command_topic = LaunchConfiguration("vehicle_command_topic")
    vehicle_command_in_topic = LaunchConfiguration("vehicle_command_in_topic")
    vehicle_command_monitor_topic = LaunchConfiguration("vehicle_command_monitor_topic")
    vehicle_local_position_topic = LaunchConfiguration("vehicle_local_position_topic")
    vehicle_status_topic = LaunchConfiguration("vehicle_status_topic")
    max_vel = LaunchConfiguration("max_vel")
    max_acc = LaunchConfiguration("max_acc")
    planning_horizon = LaunchConfiguration("planning_horizon")
    emergency_time = LaunchConfiguration("emergency_time")
    obstacle_inflation = LaunchConfiguration("obstacle_inflation")
    collision_distance = LaunchConfiguration("collision_distance")
    collision_weight = LaunchConfiguration("collision_weight")
    replan_time_threshold = LaunchConfiguration("replan_time_threshold")
    replan_distance_threshold = LaunchConfiguration("replan_distance_threshold")
    control_points_distance = LaunchConfiguration("control_points_distance")
    max_jerk = LaunchConfiguration("max_jerk")
    ground_height = LaunchConfiguration("ground_height")
    virtual_ceil_height = LaunchConfiguration("virtual_ceil_height")
    visualization_truncate_height = LaunchConfiguration("visualization_truncate_height")
    map_size_x = LaunchConfiguration("map_size_x")
    map_size_y = LaunchConfiguration("map_size_y")
    map_size_z = LaunchConfiguration("map_size_z")
    astar_pool_size_x = LaunchConfiguration("astar_pool_size_x")
    astar_pool_size_y = LaunchConfiguration("astar_pool_size_y")
    astar_pool_size_z = LaunchConfiguration("astar_pool_size_z")
    gz_scan_topic = LaunchConfiguration("gz_scan_topic")
    native_pointcloud_topic = LaunchConfiguration("native_pointcloud_topic")
    pointcloud_drop_non_finite_points = LaunchConfiguration("pointcloud_drop_non_finite_points")
    pointcloud_min_world_z_m = LaunchConfiguration("pointcloud_min_world_z_m")
    pointcloud_max_world_z_m = LaunchConfiguration("pointcloud_max_world_z_m")
    pointcloud_self_filter_enabled = LaunchConfiguration("pointcloud_self_filter_enabled")
    pointcloud_self_filter_radius_xy_m = LaunchConfiguration("pointcloud_self_filter_radius_xy_m")
    pointcloud_self_filter_radius_z_m = LaunchConfiguration("pointcloud_self_filter_radius_z_m")
    gazebo_depth_topic = LaunchConfiguration("gazebo_depth_topic")
    depth_cloud_topic = LaunchConfiguration("depth_cloud_topic")
    sim_imu_topic = LaunchConfiguration("sim_imu_topic")
    use_ego_planner = LaunchConfiguration("use_ego_planner")
    use_simple_avoidance_fallback = LaunchConfiguration("use_simple_avoidance_fallback")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_fastlio_bridge", default_value="false"),
        DeclareLaunchArgument("use_sim_bridge", default_value="true"),
        DeclareLaunchArgument("use_depth_camera_fastlio", default_value="false"),
        DeclareLaunchArgument("use_ego_planner", default_value="false"),
        DeclareLaunchArgument("use_simple_avoidance_fallback", default_value="true"),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz2 with the single-drone Ego-Planner visualization profile",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("px4_trajectory_interface"),
                "launch",
                "ego_planner_single.rviz",
            ]),
            description="RViz2 config used for GUI avoidance validation",
        ),
        DeclareLaunchArgument(
            "use_native_3d_pointcloud",
            default_value="false",
            description="Use a native PointCloud2 topic as the planner map source in simulation",
        ),
        DeclareLaunchArgument(
            "use_scan_fallback",
            default_value="false",
            description="Fallback to Gazebo scan -> PointCloud2 when no native 3D point cloud is available",
        ),
        DeclareLaunchArgument("fixed_goal_altitude_m", default_value="1.5"),
        DeclareLaunchArgument("lio_odom_topic", default_value="/autonomy/lio_odometry"),
        DeclareLaunchArgument(
            "planner_odom_topic",
            default_value="/odom",
            description="ENU odometry topic consumed by Ego-Planner. FAST-LIO bridge publishes this from raw /Odometry.",
        ),
        DeclareLaunchArgument(
            "local_map_topic",
            default_value="/autonomy/local_map",
            description="PointCloud2 obstacle topic consumed by Ego-Planner and shown in RViz",
        ),
        DeclareLaunchArgument("position_cmd_topic", default_value="/planning/position_cmd"),
        DeclareLaunchArgument("planner_goal_topic", default_value="/move_base_simple/goal"),
        DeclareLaunchArgument("vehicle_command_topic", default_value="/fmu/out/vehicle_command"),
        DeclareLaunchArgument("vehicle_command_in_topic", default_value="/fmu/in/vehicle_command"),
        DeclareLaunchArgument(
            "vehicle_command_monitor_topic",
            default_value="/fmu/out/vehicle_command",
            description="Monitor PX4 vehicle commands to suspend trajectory output on LAND/RTL/disarm/mode takeover",
        ),
        DeclareLaunchArgument("vehicle_local_position_topic", default_value="/fmu/out/vehicle_local_position_v1"),
        DeclareLaunchArgument("vehicle_status_topic", default_value="/fmu/out/vehicle_status_v4"),
        DeclareLaunchArgument(
            "trajectory_auto_arm",
            default_value="false",
            description="Do not arm automatically from the planner chain; arm/takeoff stays under the pilot or validation script.",
        ),
        DeclareLaunchArgument("trajectory_auto_set_offboard", default_value="true"),
        DeclareLaunchArgument(
            "suspend_on_external_mode_command",
            default_value="true",
            description="Suspend trajectory output when PX4 receives LAND/RTL/disarm or a non-Offboard mode command",
        ),
        DeclareLaunchArgument(
            "require_armed_before_offboard",
            default_value="true",
            description="Do not automatically request Offboard while PX4 reports disarmed",
        ),
        DeclareLaunchArgument(
            "require_local_position_before_offboard",
            default_value="true",
            description="Require PX4 local ENU and planner command to agree before automatic Offboard/arm requests",
        ),
        DeclareLaunchArgument(
            "max_offboard_start_horizontal_error_m",
            default_value="1.2",
            description="Maximum horizontal mismatch allowed before automatic Offboard/arm request",
        ),
        DeclareLaunchArgument(
            "max_offboard_start_vertical_error_m",
            default_value="0.8",
            description="Maximum vertical mismatch allowed before automatic Offboard/arm request",
        ),
        DeclareLaunchArgument(
            "max_position_setpoint_step_horizontal_m",
            default_value="0.35",
            description="Clamp each planner position setpoint to this maximum horizontal step from current PX4 local ENU",
        ),
        DeclareLaunchArgument(
            "max_position_setpoint_step_vertical_m",
            default_value="0.18",
            description="Clamp each planner position setpoint to this maximum vertical step from current PX4 local ENU",
        ),
        DeclareLaunchArgument(
            "align_planner_frame_to_px4_local",
            default_value="true",
            description="Translate planner world position commands into the current PX4 local ENU frame at handover",
        ),
        DeclareLaunchArgument("max_vel", default_value="0.75"),
        DeclareLaunchArgument("max_acc", default_value="1.0"),
        DeclareLaunchArgument("planning_horizon", default_value="9.0"),
        DeclareLaunchArgument("emergency_time", default_value="0.8"),
        DeclareLaunchArgument("obstacle_inflation", default_value="0.40"),
        DeclareLaunchArgument(
            "collision_distance",
            default_value="0.45",
            description="Minimum optimizer clearance target around occupied cells",
        ),
        DeclareLaunchArgument(
            "collision_weight",
            default_value="4.0",
            description="Collision cost weight for safer obstacle clearance",
        ),
        DeclareLaunchArgument(
            "replan_time_threshold",
            default_value="0.20",
            description="Replan earlier in time so the vehicle starts detouring sooner",
        ),
        DeclareLaunchArgument(
            "replan_distance_threshold",
            default_value="0.30",
            description="Replan after smaller trajectory deviations for tighter obstacle response",
        ),
        DeclareLaunchArgument(
            "control_points_distance",
            default_value="0.25",
            description="Slightly denser B-spline control points for smoother close-obstacle detours",
        ),
        DeclareLaunchArgument(
            "max_jerk",
            default_value="2.5",
            description="Lower jerk limit to avoid aggressive late obstacle skimming",
        ),
        DeclareLaunchArgument(
            "ground_height",
            default_value="0.0",
            description="Minimum map height used by Ego-Planner to discourage near-ground paths",
        ),
        DeclareLaunchArgument(
            "virtual_ceil_height",
            default_value="3.2",
            description="Validation ceiling to discourage flying over the center obstacle",
        ),
        DeclareLaunchArgument(
            "visualization_truncate_height",
            default_value="3.5",
            description="Occupancy visualization clip height for the single-drone validation scene",
        ),
        DeclareLaunchArgument("map_size_x", default_value="30.0"),
        DeclareLaunchArgument("map_size_y", default_value="30.0"),
        DeclareLaunchArgument("map_size_z", default_value="4.0"),
        DeclareLaunchArgument(
            "astar_pool_size_x",
            default_value="256",
            description="A* grid pool size in x cells. Increase only after checking memory headroom.",
        ),
        DeclareLaunchArgument(
            "astar_pool_size_y",
            default_value="256",
            description="A* grid pool size in y cells. Increase only after checking memory headroom.",
        ),
        DeclareLaunchArgument(
            "astar_pool_size_z",
            default_value="128",
            description="A* grid pool size in z cells. Increase only after checking memory headroom.",
        ),
        DeclareLaunchArgument(
            "gz_scan_topic",
            default_value="/gazebo/room_obstacles/iris_rplidar/rplidar/link/laser/scan",
        ),
        DeclareLaunchArgument(
            "native_pointcloud_topic",
            default_value="/sim/mid360/points",
            description="Native PointCloud2 topic from a Gazebo Mid360-style simulated sensor",
        ),
        DeclareLaunchArgument(
            "pointcloud_drop_non_finite_points",
            default_value="true",
            description="Drop invalid XYZ samples before relaying point clouds into Ego-Planner",
        ),
        DeclareLaunchArgument(
            "pointcloud_min_world_z_m",
            default_value="0.25",
            description="Discard relayed planner map points below this world-frame height",
        ),
        DeclareLaunchArgument(
            "pointcloud_max_world_z_m",
            default_value="3.20",
            description="Discard relayed planner map points above this world-frame height",
        ),
        DeclareLaunchArgument(
            "pointcloud_self_filter_enabled",
            default_value="true",
            description="Remove points close to the vehicle before feeding Ego-Planner",
        ),
        DeclareLaunchArgument(
            "pointcloud_self_filter_radius_xy_m",
            default_value="1.00",
            description="Horizontal self-filter radius around the vehicle in the planner world frame",
        ),
        DeclareLaunchArgument(
            "pointcloud_self_filter_radius_z_m",
            default_value="1.00",
            description="Vertical self-filter half-height around the vehicle in the planner world frame",
        ),
        DeclareLaunchArgument(
            "gazebo_depth_topic",
            default_value="/gazebo/room_obstacles/iris_mid360_sim/mid360_front/link/depth_camera/image",
            description="Gazebo Classic transport depth image topic to convert into PointCloud2",
        ),
        DeclareLaunchArgument("depth_cloud_topic", default_value="/sim/mid360/points"),
        DeclareLaunchArgument("sim_imu_topic", default_value="/sim/imu"),
        Node(
            package="px4_nav2_bridge",
            executable="px4_local_position_nav2_bridge",
            name="px4_local_position_ego_bridge",
            output="screen",
            condition=IfCondition(use_sim_bridge),
            parameters=[{
                "use_sim_time": use_sim_time,
                "vehicle_local_position_topic": "/fmu/out/vehicle_local_position_v1",
                "odom_topic": lio_odom_topic,
                "map_frame_id": "world",
                "odom_frame_id": "world",
                "base_frame_id": "base_link",
                "publish_tf": False,
                "project_to_2d": False,
                "z_up": True,
            }],
        ),
        Node(
            package="px4_gazebo_depth_bridge",
            executable="gazebo_depth_to_pointcloud",
            name="gazebo_depth_to_pointcloud_front",
            output="screen",
            condition=UnlessCondition(use_native_3d_pointcloud),
            parameters=[{
                "gazebo_topic": gazebo_depth_topic,
                "pointcloud_topic": PythonExpression([
                    "'", use_depth_camera_fastlio, "' == 'true' and '",
                    depth_cloud_topic,
                    "' or '",
                    local_map_topic,
                    "'"
                ]),
                "frame_id": "world",
                "horizontal_fov_rad": 1.5708,
                "max_range_m": 20.0,
                "stride": 4,
                "x_offset_m": 0.10,
                "pitch_rad": -0.10,
                "use_vehicle_pose": True,
                "vehicle_local_position_topic": vehicle_local_position_topic,
                "pose_timeout_s": 1.0,
                "min_world_z_m": 0.20,
                "max_world_z_m": 3.00,
                "self_filter_radius_m": 0.60,
            }],
        ),
        Node(
            package="px4_obstacle_tools",
            executable="gz_scan_to_pointcloud",
            name="gz_scan_to_pointcloud_ego",
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", use_sim_bridge, "' == 'true' and '", use_scan_fallback, "' == 'true'"
            ])),
            parameters=[{
                "use_sim_time": use_sim_time,
                "gz_scan_topic": gz_scan_topic,
                "cloud_topic": local_map_topic,
                "frame_id": "world",
                "range_min_m": 0.1,
                "range_max_m": 20.0,
                "use_vehicle_pose": True,
                "vehicle_local_position_topic": "/fmu/out/vehicle_local_position_v1",
                "vehicle_frame_is_frd": True,
            }],
        ),
        Node(
            package="px4_obstacle_tools",
            executable="pointcloud_relay",
            name="native_pointcloud_to_local_map",
            output="screen",
            condition=IfCondition(use_native_3d_pointcloud),
            parameters=[{
                "input_topic": native_pointcloud_topic,
                "output_topic": local_map_topic,
                "output_frame_id": "world",
                "drop_non_finite_points": pointcloud_drop_non_finite_points,
                "min_world_z_m": pointcloud_min_world_z_m,
                "max_world_z_m": pointcloud_max_world_z_m,
                "self_filter_enabled": pointcloud_self_filter_enabled,
                "self_filter_radius_xy_m": pointcloud_self_filter_radius_xy_m,
                "self_filter_radius_z_m": pointcloud_self_filter_radius_z_m,
                "vehicle_local_position_topic": vehicle_local_position_topic,
                "vehicle_odometry_topic": planner_odom_topic,
            }],
        ),
        Node(
            package="px4_fastlio_bridge",
            executable="px4_imu_bridge",
            name="px4_imu_bridge",
            output="screen",
            condition=IfCondition(use_depth_camera_fastlio),
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": "/fmu/out/sensor_combined",
                "input_type": "sensor_combined",
                "output_topic": sim_imu_topic,
                "frame_id": "base_link",
            }],
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping_depth_camera",
            output="screen",
            condition=IfCondition(use_depth_camera_fastlio),
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("px4_fastlio_bridge"),
                    "config",
                    "depth_camera_fastlio.yaml",
                ]),
                {
                    "common.lid_topic": depth_cloud_topic,
                    "common.imu_topic": sim_imu_topic,
                },
            ],
        ),
        Node(
            package="px4_fastlio_bridge",
            executable="lio_odometry_bridge",
            name="lio_odometry_bridge",
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", use_fastlio_bridge, "' == 'true' or '", use_depth_camera_fastlio, "' == 'true'"
            ])),
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_odom_topic": "/Odometry",
                "output_odom_topic": lio_odom_topic,
                "output_nav2_odom_topic": "/odom",
                "output_position_topic": "/autonomy/lio_position_ned",
                "input_map_topic": "/Laser_map",
                "output_map_topic": local_map_topic,
                "input_registered_cloud_topic": "/cloud_registered",
                "output_registered_cloud_topic": "/autonomy/cloud_registered",
                "nav2_map_frame_id": "map",
                "nav2_odom_frame_id": "odom",
                "nav2_base_frame_id": "base_link",
                "publish_nav2_tf": False,
            }],
        ),
        Node(
            package="px4_nav2_bridge",
            executable="qgc_reposition_goal_bridge",
            name="qgc_ego_goal_bridge",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "vehicle_command_topic": vehicle_command_topic,
                "vehicle_command_in_topic": vehicle_command_in_topic,
                "vehicle_local_position_topic": vehicle_local_position_topic,
                "vehicle_status_topic": vehicle_status_topic,
                "goal_pose_topic": planner_goal_topic,
                "map_frame_id": "world",
                "fixed_goal_altitude_m": fixed_goal_altitude_m,
                "use_qgc_altitude": False,
                "require_external_command": True,
                "auto_request_ros2_mode": False,
                "enable_nav2_action": False,
                "require_armed_for_nav2_goal": False,
            }],
        ),
        Node(
            package="px4_nav2_bridge",
            executable="odom_path_visualizer",
            name="ego_odom_path_visualizer",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_odom_topic": planner_odom_topic,
                "path_topic": "/drone_0_vis/path",
                "path_frame_id": "world",
                "max_path_length": 2000,
                "min_pose_separation_m": 0.05,
                "min_pose_separation_yaw_rad": 0.10,
            }],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="ego_planner",
                    executable="ego_planner_node",
                    name="ego_planner_node",
                    output="screen",
                    condition=IfCondition(use_ego_planner),
                    remappings=[
                        ("odom_world", planner_odom_topic),
                        ("grid_map/odom", planner_odom_topic),
                        ("grid_map/cloud", local_map_topic),
                        ("goal_point", "/drone_0_plan_vis/goal_point"),
                        ("global_list", "/drone_0_plan_vis/global_list"),
                        ("init_list", "/drone_0_plan_vis/init_list"),
                        ("optimal_list", "/drone_0_plan_vis/optimal_list"),
                        ("a_star_list", "/drone_0_plan_vis/a_star_list"),
                        ("grid_map/occupancy_inflate", "/drone_0_grid/grid_map/occupancy_inflate"),
                        ("planning/bspline", "/planning/bspline"),
                    ],
                    parameters=[{
                        "fsm/flight_type": 1,
                        "fsm/thresh_replan_time": replan_time_threshold,
                        "fsm/thresh_no_replan_meter": replan_distance_threshold,
                        "fsm/planning_horizon": planning_horizon,
                        "fsm/planning_horizen_time": 2.0,
                        "fsm/emergency_time": emergency_time,
                        "fsm/realworld_experiment": False,
                        "fsm/fail_safe": True,
                        "fsm/waypoint_num": 0,
                        "grid_map/resolution": 0.15,
                        "grid_map/map_size_x": map_size_x,
                        "grid_map/map_size_y": map_size_y,
                        "grid_map/map_size_z": map_size_z,
                        "grid_map/local_update_range_x": 8.0,
                        "grid_map/local_update_range_y": 8.0,
                        "grid_map/local_update_range_z": 3.5,
                        "grid_map/obstacles_inflation": obstacle_inflation,
                        "grid_map/local_map_margin": 10,
                        "grid_map/ground_height": ground_height,
                        "grid_map/min_ray_length": 0.1,
                        "grid_map/max_ray_length": 20.0,
                        "grid_map/visualization_truncate_height": visualization_truncate_height,
                        "grid_map/virtual_ceil_height": virtual_ceil_height,
                        "grid_map/virtual_ceil_yp": 15.0,
                        "grid_map/virtual_ceil_yn": -15.0,
                        "grid_map/use_depth_filter": False,
                        "grid_map/pose_type": 0,
                        "grid_map/frame_id": "world",
                        "manager/max_vel": max_vel,
                        "manager/max_acc": max_acc,
                        "manager/max_jerk": max_jerk,
                        "manager/control_points_distance": control_points_distance,
                        "manager/feasibility_tolerance": 0.05,
                        "manager/planning_horizon": planning_horizon,
                        "manager/use_distinctive_trajs": True,
                        "manager/drone_id": -1,
                        "manager/astar_pool_size_x": astar_pool_size_x,
                        "manager/astar_pool_size_y": astar_pool_size_y,
                        "manager/astar_pool_size_z": astar_pool_size_z,
                        "optimization/lambda_smooth": 1.0,
                        "optimization/lambda_collision": collision_weight,
                        "optimization/lambda_feasibility": 0.1,
                        "optimization/lambda_fitness": 1.0,
                        "optimization/dist0": collision_distance,
                        "optimization/swarm_clearance": 0.5,
                        "optimization/max_vel": max_vel,
                        "optimization/max_acc": max_acc,
                        "bspline/limit_vel": max_vel,
                        "bspline/limit_acc": max_acc,
                        "bspline/limit_ratio": 1.1,
                        "prediction/obj_num": 0,
                        "prediction/lambda": 1.0,
                        "prediction/predict_rate": 1.0,
                    }],
                )
            ],
        ),
        Node(
            package="ego_planner",
            executable="traj_server",
            name="ego_traj_server",
            output="screen",
            condition=IfCondition(use_ego_planner),
            remappings=[
                ("planning/bspline", "/planning/bspline"),
                ("position_cmd", position_cmd_topic),
            ],
            parameters=[{
                "traj_server/time_forward": 0.8,
            }],
        ),
        Node(
            package="px4_trajectory_interface",
            executable="simple_avoidance_position_cmd",
            name="simple_avoidance_position_cmd",
            output="screen",
            condition=IfCondition(use_simple_avoidance_fallback),
            parameters=[{
                "use_sim_time": use_sim_time,
                "odom_topic": planner_odom_topic,
                "goal_topic": planner_goal_topic,
                "position_cmd_topic": position_cmd_topic,
                "max_speed_m_s": 0.75,
                "acceptance_radius_m": 0.35,
                "avoidance_clearance_m": 1.00,
                "takeoff_altitude_m": fixed_goal_altitude_m,
            }],
        ),
        Node(
            package="px4_trajectory_interface",
            executable="trajectory_interface",
            name="trajectory_interface",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "position_cmd_topic": position_cmd_topic,
                "vehicle_status_topic": vehicle_status_topic,
                "vehicle_local_position_topic": vehicle_local_position_topic,
                "offboard_control_mode_topic": "/fmu/in/offboard_control_mode",
                "trajectory_setpoint_topic": "/fmu/in/trajectory_setpoint",
                "vehicle_command_topic": vehicle_command_in_topic,
                "vehicle_command_monitor_topic": vehicle_command_monitor_topic,
                "command_timeout_s": 0.25,
                "offboard_setpoint_warmup_count": 20,
                "auto_set_offboard": LaunchConfiguration("trajectory_auto_set_offboard"),
                "auto_arm": LaunchConfiguration("trajectory_auto_arm"),
                "hold_position_on_timeout": False,
                "suspend_on_external_mode_command": LaunchConfiguration(
                    "suspend_on_external_mode_command"
                ),
                "require_armed_before_offboard": LaunchConfiguration("require_armed_before_offboard"),
                "require_local_position_before_offboard": LaunchConfiguration(
                    "require_local_position_before_offboard"
                ),
                "max_offboard_start_horizontal_error_m": LaunchConfiguration(
                    "max_offboard_start_horizontal_error_m"
                ),
                "max_offboard_start_vertical_error_m": LaunchConfiguration(
                    "max_offboard_start_vertical_error_m"
                ),
                "max_position_setpoint_step_horizontal_m": LaunchConfiguration(
                    "max_position_setpoint_step_horizontal_m"
                ),
                "max_position_setpoint_step_vertical_m": LaunchConfiguration(
                    "max_position_setpoint_step_vertical_m"
                ),
                "align_planner_frame_to_px4_local": LaunchConfiguration(
                    "align_planner_frame_to_px4_local"
                ),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="ego_planner_rviz",
            output="screen",
            condition=IfCondition(launch_rviz),
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
