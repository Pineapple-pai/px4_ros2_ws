from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_livox = LaunchConfiguration("use_livox")
    use_fastlio = LaunchConfiguration("use_fastlio")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_sim_time = LaunchConfiguration("use_sim_time")
    livox_config = LaunchConfiguration("livox_config")
    fastlio_config_path = LaunchConfiguration("fastlio_config_path")
    fastlio_config_file = LaunchConfiguration("fastlio_config_file")
    fastlio_rviz = LaunchConfiguration("fastlio_rviz")
    mission_size_m = LaunchConfiguration("mission_size_m")
    mission_altitude_m = LaunchConfiguration("mission_altitude_m")
    hold_time_s = LaunchConfiguration("hold_time_s")
    acceptance_radius_m = LaunchConfiguration("acceptance_radius_m")
    max_horizontal_velocity_m_s = LaunchConfiguration("max_horizontal_velocity_m_s")
    max_vertical_velocity_m_s = LaunchConfiguration("max_vertical_velocity_m_s")
    max_heading_rate_deg_s = LaunchConfiguration("max_heading_rate_deg_s")
    mission_timeout_s = LaunchConfiguration("mission_timeout_s")
    auto_rtl_after_finish = LaunchConfiguration("auto_rtl_after_finish")
    obstacle_distance_topic = LaunchConfiguration("obstacle_distance_topic")
    obstacle_stop_distance_m = LaunchConfiguration("obstacle_stop_distance_m")
    obstacle_abort_distance_m = LaunchConfiguration("obstacle_abort_distance_m")
    obstacle_hold_timeout_s = LaunchConfiguration("obstacle_hold_timeout_s")
    enable_obstacle_hold = LaunchConfiguration("enable_obstacle_hold")
    enable_local_avoidance = LaunchConfiguration("enable_local_avoidance")
    avoidance_trigger_distance_m = LaunchConfiguration("avoidance_trigger_distance_m")
    avoidance_clearance_m = LaunchConfiguration("avoidance_clearance_m")
    avoidance_lateral_offset_m = LaunchConfiguration("avoidance_lateral_offset_m")
    avoidance_forward_offset_m = LaunchConfiguration("avoidance_forward_offset_m")
    avoidance_speed_m_s = LaunchConfiguration("avoidance_speed_m_s")
    obstacle_data_timeout_s = LaunchConfiguration("obstacle_data_timeout_s")
    mission_file = LaunchConfiguration("mission_file")
    mission_waypoints_ned = LaunchConfiguration("mission_waypoints_ned")
    waypoints_topic = LaunchConfiguration("waypoints_topic")
    target_point_topic = LaunchConfiguration("target_point_topic")
    accept_runtime_target = LaunchConfiguration("accept_runtime_target")
    control_source = LaunchConfiguration("control_source")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    nav2_cmd_vel_topic = LaunchConfiguration("nav2_cmd_vel_topic")
    nav2_fixed_altitude_m = LaunchConfiguration("nav2_fixed_altitude_m")
    nav2_cmd_timeout_s = LaunchConfiguration("nav2_cmd_timeout_s")
    nav2_lookahead_time_s = LaunchConfiguration("nav2_lookahead_time_s")
    nav2_min_lookahead_m = LaunchConfiguration("nav2_min_lookahead_m")
    nav2_max_cmd_speed_m_s = LaunchConfiguration("nav2_max_cmd_speed_m_s")
    nav2_turn_in_place_yaw_rate_rad_s = LaunchConfiguration("nav2_turn_in_place_yaw_rate_rad_s")
    nav2_turn_speed_scale = LaunchConfiguration("nav2_turn_speed_scale")
    nav2_turn_max_lookahead_m = LaunchConfiguration("nav2_turn_max_lookahead_m")
    nav2_obstacle_slowdown_distance_m = LaunchConfiguration("nav2_obstacle_slowdown_distance_m")
    nav2_obstacle_min_speed_m_s = LaunchConfiguration("nav2_obstacle_min_speed_m_s")
    nav2_emergency_retreat_distance_m = LaunchConfiguration("nav2_emergency_retreat_distance_m")
    nav2_emergency_retreat_speed_m_s = LaunchConfiguration("nav2_emergency_retreat_speed_m_s")
    nav2_require_ros2_control = LaunchConfiguration("nav2_require_ros2_control")
    nav2_required_nav_state = LaunchConfiguration("nav2_required_nav_state")
    nav2_status_timeout_s = LaunchConfiguration("nav2_status_timeout_s")
    qgc_auto_request_ros2_mode = LaunchConfiguration("qgc_auto_request_ros2_mode")
    qgc_mode_request_period_s = LaunchConfiguration("qgc_mode_request_period_s")
    qgc_mode_request_hold_s = LaunchConfiguration("qgc_mode_request_hold_s")
    qgc_reposition_goal_bridge = LaunchConfiguration("qgc_reposition_goal_bridge")
    nav2_odom_source = LaunchConfiguration("nav2_odom_source")
    nav2_cloud_topic = LaunchConfiguration("nav2_cloud_topic")
    launch_gz_scan_to_pointcloud = LaunchConfiguration("launch_gz_scan_to_pointcloud")
    octomap_resolution = LaunchConfiguration("octomap_resolution")
    octomap_min_z = LaunchConfiguration("octomap_min_z")
    octomap_max_z = LaunchConfiguration("octomap_max_z")
    launch_obstacle_sim = LaunchConfiguration("launch_obstacle_sim")
    obstacle_sim_mode = LaunchConfiguration("obstacle_sim_mode")
    launch_gz_scan_distance = LaunchConfiguration("launch_gz_scan_distance")
    launch_gz_six_direction_distance = LaunchConfiguration("launch_gz_six_direction_distance")
    launch_laserscan_min_distance = LaunchConfiguration("launch_laserscan_min_distance")
    gz_scan_topic = LaunchConfiguration("gz_scan_topic")
    gz_lidar_scan_topic = LaunchConfiguration("gz_lidar_scan_topic")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    launch_pointcloud_min_distance = LaunchConfiguration("launch_pointcloud_min_distance")
    launch_pointcloud_front_min_distance = LaunchConfiguration("launch_pointcloud_front_min_distance")
    launch_pointcloud_up_min_distance = LaunchConfiguration("launch_pointcloud_up_min_distance")
    launch_pointcloud_down_min_distance = LaunchConfiguration("launch_pointcloud_down_min_distance")
    default_livox_config = ""
    default_fastlio_config_path = PathJoinSubstitution([
        FindPackageShare("px4_fastlio_bridge"),
        "config",
    ])
    default_nav2_params_file = PathJoinSubstitution([
        FindPackageShare("px4_autonomy_bringup"),
        "config",
        "nav2_params.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_livox", default_value="false"),
        DeclareLaunchArgument("use_fastlio", default_value="false"),
        DeclareLaunchArgument("use_nav2", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("livox_config", default_value=default_livox_config),
        DeclareLaunchArgument("fastlio_config_path", default_value=default_fastlio_config_path),
        DeclareLaunchArgument("fastlio_config_file", default_value="mid360.yaml"),
        DeclareLaunchArgument("fastlio_rviz", default_value="false"),
        DeclareLaunchArgument("mission_size_m", default_value="2.5"),
        DeclareLaunchArgument("mission_altitude_m", default_value="2.0"),
        DeclareLaunchArgument("hold_time_s", default_value="2.0"),
        DeclareLaunchArgument("acceptance_radius_m", default_value="0.35"),
        DeclareLaunchArgument("max_horizontal_velocity_m_s", default_value="0.8"),
        DeclareLaunchArgument("max_vertical_velocity_m_s", default_value="0.5"),
        DeclareLaunchArgument("max_heading_rate_deg_s", default_value="45.0"),
        DeclareLaunchArgument("mission_timeout_s", default_value="120.0"),
        DeclareLaunchArgument("auto_rtl_after_finish", default_value="false"),
        DeclareLaunchArgument("obstacle_distance_topic", default_value="/perception/min_obstacle_distance"),
        DeclareLaunchArgument("obstacle_stop_distance_m", default_value="2.0"),
        DeclareLaunchArgument("obstacle_abort_distance_m", default_value="1.0"),
        DeclareLaunchArgument("obstacle_hold_timeout_s", default_value="5.0"),
        DeclareLaunchArgument("enable_obstacle_hold", default_value="true"),
        DeclareLaunchArgument("enable_local_avoidance", default_value="true"),
        DeclareLaunchArgument("avoidance_trigger_distance_m", default_value="3.0"),
        DeclareLaunchArgument("avoidance_clearance_m", default_value="2.0"),
        DeclareLaunchArgument("avoidance_lateral_offset_m", default_value="1.5"),
        DeclareLaunchArgument("avoidance_forward_offset_m", default_value="2.0"),
        DeclareLaunchArgument("avoidance_speed_m_s", default_value="0.45"),
        DeclareLaunchArgument("obstacle_data_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("mission_file", default_value=""),
        DeclareLaunchArgument("mission_waypoints_ned", default_value=""),
        DeclareLaunchArgument("waypoints_topic", default_value="/autonomy/waypoints_ned"),
        DeclareLaunchArgument("target_point_topic", default_value="/autonomy/target_ned"),
        DeclareLaunchArgument("accept_runtime_target", default_value="true"),
        DeclareLaunchArgument("control_source", default_value="mission",
                              description="Set to nav2_cmd_vel when Nav2 owns horizontal planning"),
        DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params_file),
        DeclareLaunchArgument("nav2_cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("nav2_fixed_altitude_m", default_value="2.0"),
        DeclareLaunchArgument("nav2_cmd_timeout_s", default_value="0.5"),
        DeclareLaunchArgument("nav2_lookahead_time_s", default_value="1.0"),
        DeclareLaunchArgument("nav2_min_lookahead_m", default_value="0.35"),
        DeclareLaunchArgument("nav2_max_cmd_speed_m_s", default_value="0.45"),
        DeclareLaunchArgument("nav2_turn_in_place_yaw_rate_rad_s", default_value="0.35"),
        DeclareLaunchArgument("nav2_turn_speed_scale", default_value="0.25"),
        DeclareLaunchArgument("nav2_turn_max_lookahead_m", default_value="0.20"),
        DeclareLaunchArgument("nav2_obstacle_slowdown_distance_m", default_value="3.0"),
        DeclareLaunchArgument("nav2_obstacle_min_speed_m_s", default_value="0.10"),
        DeclareLaunchArgument("nav2_emergency_retreat_distance_m", default_value="0.8"),
        DeclareLaunchArgument("nav2_emergency_retreat_speed_m_s", default_value="0.25"),
        DeclareLaunchArgument("nav2_require_ros2_control", default_value="true"),
        DeclareLaunchArgument("nav2_required_nav_state", default_value="23"),
        DeclareLaunchArgument("nav2_status_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("qgc_auto_request_ros2_mode", default_value="true"),
        DeclareLaunchArgument("qgc_mode_request_period_s", default_value="0.25"),
        DeclareLaunchArgument("qgc_mode_request_hold_s", default_value="30.0"),
        DeclareLaunchArgument("qgc_reposition_goal_bridge", default_value="true"),
        DeclareLaunchArgument("nav2_odom_source", default_value="px4_local",
                              description="px4_local uses PX4 local position; fastlio uses FAST-LIO bridge"),
        DeclareLaunchArgument("nav2_cloud_topic", default_value="/autonomy/cloud_registered"),
        DeclareLaunchArgument("launch_gz_scan_to_pointcloud", default_value="false",
                              description="Use Gazebo Classic scan as a simulated Nav2 point cloud"),
        DeclareLaunchArgument("octomap_resolution", default_value="0.10"),
        DeclareLaunchArgument("octomap_min_z", default_value="-0.5"),
        DeclareLaunchArgument("octomap_max_z", default_value="2.0"),
        DeclareLaunchArgument("launch_obstacle_sim", default_value="false"),
        DeclareLaunchArgument("obstacle_sim_mode", default_value="safe"),
        DeclareLaunchArgument("launch_gz_scan_distance", default_value="false",
                              description="Compute min distance directly from Gazebo transport scan"),
        DeclareLaunchArgument("launch_gz_six_direction_distance", default_value="false",
                              description="Compute and print front/back/left/right/up/down distances"),
        DeclareLaunchArgument("launch_laserscan_min_distance", default_value="false",
                              description="Compute min distance from ROS LaserScan"),
        DeclareLaunchArgument("gz_scan_topic", default_value="/gazebo/room_obstacles/iris_rplidar/rplidar/link/laser/scan",
                              description="Gazebo transport scan topic (Range or LaserScan) for gz_scan_min_distance"),
        DeclareLaunchArgument("gz_lidar_scan_topic", default_value="/world/room_obstacles/model/x500_lidar_front_0/link/lidar_sensor_link/sensor/lidar/scan",
                              description="Full Gazebo transport topic for lidar scan"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/livox/lidar",
                              description="Output PointCloud2 topic"),
        DeclareLaunchArgument("launch_pointcloud_min_distance", default_value="true",
                              description="Compute min distance from PointCloud2"),
        DeclareLaunchArgument("launch_pointcloud_front_min_distance", default_value="false",
                              description="Compute front ROI min distance from PointCloud2"),
        DeclareLaunchArgument("launch_pointcloud_up_min_distance", default_value="false",
                              description="Compute upward ROI min distance from PointCloud2"),
        DeclareLaunchArgument("launch_pointcloud_down_min_distance", default_value="false",
                              description="Compute downward ROI min distance from PointCloud2"),
        # Livox lidar
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", use_livox, "' == 'true' and '", use_fastlio, "' != 'true'"
            ])),
            parameters=[{
                "xfer_format": 1,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "lvx_file_path": "/home/livox/livox_test.lvx",
                "user_config_path": livox_config,
                "cmdline_input_bd_code": "livox0000000001",
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("px4_fastlio_bridge"),
                    "launch",
                    "fastlio_mapping.launch.py",
                ])
            ),
            condition=IfCondition(use_fastlio),
            launch_arguments={
                "use_livox": use_livox,
                "use_sim_time": use_sim_time,
                "livox_config": livox_config,
                "fastlio_config_path": fastlio_config_path,
                "fastlio_config_file": fastlio_config_file,
                "rviz": fastlio_rviz,
            }.items(),
        ),
        Node(
            package="px4_nav2_bridge",
            executable="px4_local_position_nav2_bridge",
            name="px4_local_position_nav2_bridge",
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", use_nav2, "' == 'true' and '", nav2_odom_source, "' == 'px4_local'"
            ])),
            parameters=[{
                "use_sim_time": use_sim_time,
                "vehicle_local_position_topic": "/fmu/out/vehicle_local_position_v1",
                "odom_topic": "/odom",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "base_frame_id": "base_link",
                "publish_tf": True,
                "project_to_2d": True,
            }],
        ),
        Node(
            package="px4_obstacle_tools",
            executable="gz_scan_to_pointcloud",
            name="gz_scan_to_pointcloud_nav2",
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", use_nav2, "' == 'true' and '", launch_gz_scan_to_pointcloud, "' == 'true'"
            ])),
            parameters=[{
                "use_sim_time": use_sim_time,
                "gz_scan_topic": gz_scan_topic,
                "cloud_topic": nav2_cloud_topic,
                "frame_id": "base_link",
                "range_min_m": 0.1,
                "range_max_m": 20.0,
                "use_vehicle_pose": False,
                "vehicle_local_position_topic": "/fmu/out/vehicle_local_position_v1",
                "vehicle_frame_is_frd": True,
            }],
        ),
        Node(
            package="octomap_server",
            executable="octomap_server_node",
            name="octomap_server",
            output="screen",
            condition=IfCondition(use_nav2),
            parameters=[{
                "use_sim_time": use_sim_time,
                "frame_id": "map",
                "base_frame_id": "base_link",
                "resolution": octomap_resolution,
                "pointcloud_min_z": octomap_min_z,
                "pointcloud_max_z": octomap_max_z,
                "occupancy_min_z": octomap_min_z,
                "occupancy_max_z": octomap_max_z,
                "filter_ground": True,
            }],
            remappings=[
                ("cloud_in", nav2_cloud_topic),
                ("projected_map", "/map"),
            ],
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "launch", "nav2_bringup", "navigation_launch.py",
                ["use_sim_time:=", use_sim_time],
                ["params_file:=", nav2_params_file],
            ],
            output="screen",
            condition=IfCondition(use_nav2),
        ),
        Node(
            package="px4_nav2_bridge",
            executable="qgc_reposition_goal_bridge",
            name="qgc_reposition_goal_bridge",
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", use_nav2, "' == 'true' and '", qgc_reposition_goal_bridge, "' == 'true'"
            ])),
            parameters=[{
                "use_sim_time": use_sim_time,
                "vehicle_command_topic": "/fmu/out/vehicle_command",
                "vehicle_command_in_topic": "/fmu/in/vehicle_command",
                "vehicle_local_position_topic": "/fmu/out/vehicle_local_position_v1",
                "vehicle_status_topic": "/fmu/out/vehicle_status_v4",
                "nav2_action_name": "navigate_to_pose",
                "goal_pose_topic": "/autonomy/qgc_goal_pose",
                "map_frame_id": "map",
                "fixed_goal_altitude_m": 0.0,
                "use_qgc_altitude": False,
                "require_external_command": True,
                "auto_request_ros2_mode": qgc_auto_request_ros2_mode,
                "ros2_nav_state": nav2_required_nav_state,
                "require_armed_for_nav2_goal": True,
                "mode_request_period_s": qgc_mode_request_period_s,
                "mode_request_hold_s": qgc_mode_request_hold_s,
            }],
        ),
        # PX4 autonomy mode
        Node(
            package="px4_autonomy_mode",
            executable="px4_autonomy_mode",
            name="px4_autonomy_mode",
            output="screen",
            parameters=[{
                "mission_size_m": mission_size_m,
                "mission_altitude_m": mission_altitude_m,
                "hold_time_s": hold_time_s,
                "acceptance_radius_m": acceptance_radius_m,
                "max_horizontal_velocity_m_s": max_horizontal_velocity_m_s,
                "max_vertical_velocity_m_s": max_vertical_velocity_m_s,
                "max_heading_rate_deg_s": max_heading_rate_deg_s,
                "mission_timeout_s": mission_timeout_s,
                "auto_rtl_after_finish": auto_rtl_after_finish,
                "obstacle_distance_topic": obstacle_distance_topic,
                "front_obstacle_distance_topic": "/perception/front_obstacle_distance",
                "left_obstacle_distance_topic": "/perception/left_obstacle_distance",
                "right_obstacle_distance_topic": "/perception/right_obstacle_distance",
                "up_obstacle_distance_topic": "/perception/up_obstacle_distance",
                "down_obstacle_distance_topic": "/perception/down_obstacle_distance",
                "obstacle_stop_distance_m": obstacle_stop_distance_m,
                "obstacle_abort_distance_m": obstacle_abort_distance_m,
                "obstacle_hold_timeout_s": obstacle_hold_timeout_s,
                "enable_obstacle_hold": enable_obstacle_hold,
                "enable_local_avoidance": enable_local_avoidance,
                "avoidance_trigger_distance_m": avoidance_trigger_distance_m,
                "avoidance_clearance_m": avoidance_clearance_m,
                "avoidance_lateral_offset_m": avoidance_lateral_offset_m,
                "avoidance_forward_offset_m": avoidance_forward_offset_m,
                "avoidance_speed_m_s": avoidance_speed_m_s,
                "obstacle_data_timeout_s": obstacle_data_timeout_s,
                "mission_file": mission_file,
                "mission_waypoints_ned": mission_waypoints_ned,
                "waypoints_topic": waypoints_topic,
                "target_point_topic": target_point_topic,
                "accept_runtime_target": accept_runtime_target,
                "control_source": control_source,
                "nav2_cmd_vel_topic": nav2_cmd_vel_topic,
                "nav2_fixed_altitude_m": nav2_fixed_altitude_m,
                "nav2_cmd_timeout_s": nav2_cmd_timeout_s,
                "nav2_lookahead_time_s": nav2_lookahead_time_s,
                "nav2_min_lookahead_m": nav2_min_lookahead_m,
                "nav2_max_cmd_speed_m_s": nav2_max_cmd_speed_m_s,
                "nav2_turn_in_place_yaw_rate_rad_s": nav2_turn_in_place_yaw_rate_rad_s,
                "nav2_turn_speed_scale": nav2_turn_speed_scale,
                "nav2_turn_max_lookahead_m": nav2_turn_max_lookahead_m,
                "nav2_obstacle_slowdown_distance_m": nav2_obstacle_slowdown_distance_m,
                "nav2_obstacle_min_speed_m_s": nav2_obstacle_min_speed_m_s,
                "nav2_emergency_retreat_distance_m": nav2_emergency_retreat_distance_m,
                "nav2_emergency_retreat_speed_m_s": nav2_emergency_retreat_speed_m_s,
                "nav2_require_ros2_control": nav2_require_ros2_control,
                "nav2_required_nav_state": nav2_required_nav_state,
                "nav2_status_timeout_s": nav2_status_timeout_s,
                "nav2_vehicle_status_topic": "/fmu/out/vehicle_status_v4",
                "nav2_vehicle_control_mode_topic": "/fmu/out/vehicle_control_mode",
                "nav2_control_authority_topic": "/autonomy/nav2_control_authority_ok",
            }],
        ),
        # Obstacle distance simulator (for testing without real lidar)
        Node(
            package="px4_obstacle_tools",
            executable="obstacle_distance_sim",
            name="obstacle_distance_sim",
            output="screen",
            condition=IfCondition(launch_obstacle_sim),
            parameters=[{
                "topic": obstacle_distance_topic,
                "mode": obstacle_sim_mode,
                "safe_distance_m": 5.0,
                "hold_distance_m": 1.5,
                "abort_distance_m": 0.5,
                "publish_rate_hz": 5.0,
                "wave_period_s": 12.0,
            }],
        ),
        Node(
            package="px4_obstacle_tools",
            executable="gz_scan_min_distance",
            name="gz_scan_min_distance",
            output="screen",
            condition=IfCondition(launch_gz_scan_distance),
            parameters=[{
                "gz_scan_topic": gz_scan_topic,
                "distance_topic": obstacle_distance_topic,
                "no_obstacle_distance_m": 20.0,
            }],
        ),
        Node(
            package="px4_obstacle_tools",
            executable="laserscan_min_distance",
            name="laserscan_min_distance",
            output="screen",
            condition=IfCondition(launch_laserscan_min_distance),
            parameters=[{
                "scan_topic": gz_lidar_scan_topic,
                "distance_topic": obstacle_distance_topic,
                "no_obstacle_distance_m": 20.0,
            }],
        ),
        Node(
            package="px4_obstacle_tools",
            executable="gz_six_direction_distance",
            name="gz_six_direction_distance",
            output="screen",
            condition=IfCondition(launch_gz_six_direction_distance),
            parameters=[{
                "gz_scan_topic": gz_scan_topic,
                "vehicle_local_position_topic": "/fmu/out/vehicle_local_position_v1",
                "room_height_m": 6.0,
                "no_obstacle_distance_m": 20.0,
                "sector_width_deg": 35.0,
                "print_rate_hz": 1.0,
            }],
        ),
        # PointCloud2 min distance
        Node(
            package="px4_obstacle_tools",
            executable="pointcloud_min_distance",
            name="pointcloud_min_distance",
            output="screen",
            condition=IfCondition(launch_pointcloud_min_distance),
            parameters=[{
                "pointcloud_topic": pointcloud_topic,
                "distance_topic": obstacle_distance_topic,
                "max_distance_m": 20.0,
                "min_distance_m": 0.1,
                "min_x_m": 0.0,
                "max_x_m": 20.0,
                "min_y_m": -2.0,
                "max_y_m": 2.0,
                "min_z_m": -1.5,
                "max_z_m": 1.5,
                "distance_mode": "euclidean",
                "no_obstacle_distance_m": 20.0,
            }],
        ),
        Node(
            package="px4_obstacle_tools",
            executable="pointcloud_min_distance",
            name="pointcloud_front_min_distance",
            output="screen",
            condition=IfCondition(launch_pointcloud_front_min_distance),
            parameters=[{
                "pointcloud_topic": pointcloud_topic,
                "distance_topic": "/perception/front_obstacle_distance",
                "max_distance_m": 20.0,
                "min_distance_m": 0.1,
                "min_x_m": 0.0,
                "max_x_m": 20.0,
                "min_y_m": -2.0,
                "max_y_m": 2.0,
                "min_z_m": -1.5,
                "max_z_m": 1.5,
                "distance_mode": "x",
                "no_obstacle_distance_m": 20.0,
            }],
        ),
        Node(
            package="px4_obstacle_tools",
            executable="pointcloud_min_distance",
            name="pointcloud_up_min_distance",
            output="screen",
            condition=IfCondition(launch_pointcloud_up_min_distance),
            parameters=[{
                "pointcloud_topic": pointcloud_topic,
                "distance_topic": "/perception/up_obstacle_distance",
                "max_distance_m": 20.0,
                "min_distance_m": 0.1,
                "min_x_m": -2.0,
                "max_x_m": 2.0,
                "min_y_m": -2.0,
                "max_y_m": 2.0,
                "min_z_m": 0.2,
                "max_z_m": 3.0,
                "distance_mode": "z",
                "no_obstacle_distance_m": 20.0,
            }],
        ),
        Node(
            package="px4_obstacle_tools",
            executable="pointcloud_min_distance",
            name="pointcloud_down_min_distance",
            output="screen",
            condition=IfCondition(launch_pointcloud_down_min_distance),
            parameters=[{
                "pointcloud_topic": pointcloud_topic,
                "distance_topic": "/perception/down_obstacle_distance",
                "max_distance_m": 20.0,
                "min_distance_m": 0.1,
                "min_x_m": -2.0,
                "max_x_m": 2.0,
                "min_y_m": -2.0,
                "max_y_m": 2.0,
                "min_z_m": -3.0,
                "max_z_m": -0.2,
                "distance_mode": "z",
                "no_obstacle_distance_m": 20.0,
            }],
        ),
    ])
