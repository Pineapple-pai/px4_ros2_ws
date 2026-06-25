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
    fixed_goal_altitude_m = LaunchConfiguration("fixed_goal_altitude_m")
    lio_odom_topic = LaunchConfiguration("lio_odom_topic")
    local_map_topic = LaunchConfiguration("local_map_topic")
    position_cmd_topic = LaunchConfiguration("position_cmd_topic")
    planner_goal_topic = LaunchConfiguration("planner_goal_topic")
    vehicle_command_topic = LaunchConfiguration("vehicle_command_topic")
    vehicle_command_in_topic = LaunchConfiguration("vehicle_command_in_topic")
    vehicle_local_position_topic = LaunchConfiguration("vehicle_local_position_topic")
    vehicle_status_topic = LaunchConfiguration("vehicle_status_topic")
    max_vel = LaunchConfiguration("max_vel")
    max_acc = LaunchConfiguration("max_acc")
    planning_horizon = LaunchConfiguration("planning_horizon")
    emergency_time = LaunchConfiguration("emergency_time")
    obstacle_inflation = LaunchConfiguration("obstacle_inflation")
    map_size_x = LaunchConfiguration("map_size_x")
    map_size_y = LaunchConfiguration("map_size_y")
    map_size_z = LaunchConfiguration("map_size_z")
    gz_scan_topic = LaunchConfiguration("gz_scan_topic")
    depth_cloud_topic = LaunchConfiguration("depth_cloud_topic")
    sim_imu_topic = LaunchConfiguration("sim_imu_topic")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_fastlio_bridge", default_value="false"),
        DeclareLaunchArgument("use_sim_bridge", default_value="true"),
        DeclareLaunchArgument("use_depth_camera_fastlio", default_value="false"),
        DeclareLaunchArgument("fixed_goal_altitude_m", default_value="1.5"),
        DeclareLaunchArgument("lio_odom_topic", default_value="/autonomy/lio_odometry"),
        DeclareLaunchArgument("local_map_topic", default_value="/autonomy/local_map"),
        DeclareLaunchArgument("position_cmd_topic", default_value="/planning/position_cmd"),
        DeclareLaunchArgument("planner_goal_topic", default_value="/move_base_simple/goal"),
        DeclareLaunchArgument("vehicle_command_topic", default_value="/fmu/out/vehicle_command"),
        DeclareLaunchArgument("vehicle_command_in_topic", default_value="/fmu/in/vehicle_command"),
        DeclareLaunchArgument("vehicle_local_position_topic", default_value="/fmu/out/vehicle_local_position_v1"),
        DeclareLaunchArgument("vehicle_status_topic", default_value="/fmu/out/vehicle_status_v4"),
        DeclareLaunchArgument("max_vel", default_value="2.5"),
        DeclareLaunchArgument("max_acc", default_value="3.0"),
        DeclareLaunchArgument("planning_horizon", default_value="7.5"),
        DeclareLaunchArgument("emergency_time", default_value="1.0"),
        DeclareLaunchArgument("obstacle_inflation", default_value="0.25"),
        DeclareLaunchArgument("map_size_x", default_value="30.0"),
        DeclareLaunchArgument("map_size_y", default_value="30.0"),
        DeclareLaunchArgument("map_size_z", default_value="4.0"),
        DeclareLaunchArgument(
            "gz_scan_topic",
            default_value="/gazebo/room_obstacles/iris_rplidar/rplidar/link/laser/scan",
        ),
        DeclareLaunchArgument("depth_cloud_topic", default_value="/camera/depth/points"),
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
            }],
        ),
        Node(
            package="px4_obstacle_tools",
            executable="gz_scan_to_pointcloud",
            name="gz_scan_to_pointcloud_ego",
            output="screen",
            condition=IfCondition(use_sim_bridge),
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
            package="px4_fastlio_bridge",
            executable="px4_imu_bridge",
            name="px4_imu_bridge",
            output="screen",
            condition=IfCondition(use_depth_camera_fastlio),
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": "/fmu/out/vehicle_imu",
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
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="ego_planner",
                    executable="ego_planner_node",
                    name="ego_planner_node",
                    output="screen",
                    remappings=[
                        ("odom_world", lio_odom_topic),
                        ("grid_map/odom", lio_odom_topic),
                        ("grid_map/cloud", local_map_topic),
                        ("planning/bspline", "/planning/bspline"),
                    ],
                    parameters=[{
                        "fsm/flight_type": 1,
                        "fsm/thresh_replan_time": 1.0,
                        "fsm/thresh_no_replan_meter": 1.0,
                        "fsm/planning_horizon": planning_horizon,
                        "fsm/planning_horizen_time": 3.0,
                        "fsm/emergency_time": emergency_time,
                        "fsm/realworld_experiment": False,
                        "fsm/fail_safe": True,
                        "fsm/waypoint_num": 0,
                        "grid_map/resolution": 0.15,
                        "grid_map/map_size_x": map_size_x,
                        "grid_map/map_size_y": map_size_y,
                        "grid_map/map_size_z": map_size_z,
                        "grid_map/local_update_range_x": 6.0,
                        "grid_map/local_update_range_y": 6.0,
                        "grid_map/local_update_range_z": 3.0,
                        "grid_map/obstacles_inflation": obstacle_inflation,
                        "grid_map/local_map_margin": 10,
                        "grid_map/ground_height": -0.2,
                        "grid_map/use_depth_filter": False,
                        "grid_map/pose_type": 0,
                        "grid_map/frame_id": "world",
                        "manager/max_vel": max_vel,
                        "manager/max_acc": max_acc,
                        "manager/max_jerk": 4.0,
                        "manager/control_points_distance": 0.4,
                        "manager/feasibility_tolerance": 0.05,
                        "manager/planning_horizon": planning_horizon,
                        "manager/use_distinctive_trajs": False,
                        "manager/drone_id": -1,
                        "optimization/lambda_smooth": 1.0,
                        "optimization/lambda_collision": 0.5,
                        "optimization/lambda_feasibility": 0.1,
                        "optimization/lambda_fitness": 1.0,
                        "optimization/dist0": 0.5,
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
            executable="trajectory_interface",
            name="trajectory_interface",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "position_cmd_topic": position_cmd_topic,
                "vehicle_status_topic": vehicle_status_topic,
                "offboard_control_mode_topic": "/fmu/in/offboard_control_mode",
                "trajectory_setpoint_topic": "/fmu/in/trajectory_setpoint",
                "vehicle_command_topic": vehicle_command_in_topic,
                "publish_rate_hz": 50.0,
                "command_timeout_s": 0.25,
                "offboard_setpoint_warmup_count": 20,
                "auto_set_offboard": True,
                "auto_arm": False,
                "hold_position_on_timeout": True,
            }],
        ),
    ])
