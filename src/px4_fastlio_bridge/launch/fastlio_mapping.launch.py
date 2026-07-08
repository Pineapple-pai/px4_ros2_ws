from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_livox = LaunchConfiguration("use_livox")
    use_sim_time = LaunchConfiguration("use_sim_time")
    fastlio_config_path = LaunchConfiguration("fastlio_config_path")
    fastlio_config_file = LaunchConfiguration("fastlio_config_file")
    livox_config = LaunchConfiguration("livox_config")
    launch_rviz = LaunchConfiguration("rviz")
    sim_lidar_topic = LaunchConfiguration("sim_lidar_topic")
    sim_pattern_mode = LaunchConfiguration("sim_pattern_mode")
    sim_snapshot_time = LaunchConfiguration("sim_snapshot_time")
    sim_line_phase_jitter_ratio = LaunchConfiguration("sim_line_phase_jitter_ratio")
    sim_azimuth_jitter_ratio = LaunchConfiguration("sim_azimuth_jitter_ratio")
    output_nav2_odom_topic = LaunchConfiguration("output_nav2_odom_topic")
    nav2_map_frame_id = LaunchConfiguration("nav2_map_frame_id")
    nav2_odom_frame_id = LaunchConfiguration("nav2_odom_frame_id")
    nav2_base_frame_id = LaunchConfiguration("nav2_base_frame_id")
    publish_nav2_tf = LaunchConfiguration("publish_nav2_tf")

    default_fastlio_config_path = PathJoinSubstitution([
        FindPackageShare("px4_fastlio_bridge"),
        "config",
    ])
    default_livox_config = ""

    return LaunchDescription([
        DeclareLaunchArgument("use_livox", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("fastlio_config_path", default_value=default_fastlio_config_path),
        DeclareLaunchArgument("fastlio_config_file", default_value="mid360.yaml"),
        DeclareLaunchArgument("livox_config", default_value=default_livox_config),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("sim_lidar_topic", default_value="/sim/mid360/points_raw"),
        DeclareLaunchArgument("sim_pattern_mode", default_value="1"),
        DeclareLaunchArgument("sim_snapshot_time", default_value="true"),
        DeclareLaunchArgument("sim_line_phase_jitter_ratio", default_value="0.15"),
        DeclareLaunchArgument("sim_azimuth_jitter_ratio", default_value="0.05"),
        DeclareLaunchArgument("output_nav2_odom_topic", default_value="/odom"),
        DeclareLaunchArgument("nav2_map_frame_id", default_value="map"),
        DeclareLaunchArgument("nav2_odom_frame_id", default_value="odom"),
        DeclareLaunchArgument("nav2_base_frame_id", default_value="base_link"),
        DeclareLaunchArgument("publish_nav2_tf", default_value="true"),

        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            condition=IfCondition(use_livox),
            parameters=[{
                "xfer_format": 1,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "user_config_path": livox_config,
                "cmdline_input_bd_code": "livox0000000001",
            }],
        ),

        Node(
            package="px4_fastlio_bridge",
            executable="mid360_sim_bridge",
            name="mid360_sim_bridge",
            output="screen",
            condition=IfCondition(PythonExpression(["'", use_livox, "' == 'false'"])),
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": sim_lidar_topic,
                "output_topic": "/livox/lidar",
                "frame_id": "livox_frame",
                "num_lines": 32,
                "min_range_m": 0.2,
                "max_range_m": 40.0,
                "min_vertical_angle_rad": -0.2617993877991494,
                "max_vertical_angle_rad": 0.2617993877991494,
                "default_reflectivity": 80.0,
                "tag": 16,
                "pattern_mode": sim_pattern_mode,
                "snapshot_time": sim_snapshot_time,
                "line_phase_jitter_ratio": sim_line_phase_jitter_ratio,
                "azimuth_jitter_ratio": sim_azimuth_jitter_ratio,
            }],
        ),

        Node(
            package="px4_fastlio_bridge",
            executable="px4_imu_bridge",
            name="px4_imu_bridge",
            output="screen",
            condition=IfCondition(PythonExpression(["'", use_livox, "' == 'false'"])),
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_type": "sensor_combined",
                "input_topic": "/fmu/out/sensor_combined",
                "output_topic": "/livox/imu",
                "frame_id": "livox_frame",
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("fast_lio"),
                    "launch",
                    "mapping.launch.py",
                ])
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "config_path": fastlio_config_path,
                "config_file": fastlio_config_file,
                "rviz": launch_rviz,
            }.items(),
        ),

        Node(
            package="px4_fastlio_bridge",
            executable="lio_odometry_bridge",
            name="lio_odometry_bridge",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_odom_topic": "/Odometry",
                "output_odom_topic": "/autonomy/lio_odometry",
                "output_nav2_odom_topic": output_nav2_odom_topic,
                "output_position_topic": "/autonomy/lio_position_ned",
                "input_map_topic": "/Laser_map",
                "output_map_topic": "/autonomy/local_map",
                "input_registered_cloud_topic": "/cloud_registered",
                "output_registered_cloud_topic": "/autonomy/cloud_registered",
                "nav2_map_frame_id": nav2_map_frame_id,
                "nav2_odom_frame_id": nav2_odom_frame_id,
                "nav2_base_frame_id": nav2_base_frame_id,
                "publish_nav2_tf": publish_nav2_tf,
                "enable_sanity_check": True,
                "max_position_norm_m": 100.0,
                "max_position_delta_m": 2.0,
                "max_velocity_m_s": 5.0,
                "reset_after_rejects": 50,
            }],
        ),
    ])
