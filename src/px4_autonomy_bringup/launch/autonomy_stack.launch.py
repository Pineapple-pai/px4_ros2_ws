from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_livox = LaunchConfiguration("use_livox")
    use_fastlio = LaunchConfiguration("use_fastlio")
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
    default_livox_config = PathJoinSubstitution([
        FindPackageShare("livox_ros_driver2"),
        "config",
        "MID360_config.json",
    ])
    default_fastlio_config_path = PathJoinSubstitution([
        FindPackageShare("px4_fastlio_bridge"),
        "config",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_livox", default_value="false"),
        DeclareLaunchArgument("use_fastlio", default_value="false"),
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
