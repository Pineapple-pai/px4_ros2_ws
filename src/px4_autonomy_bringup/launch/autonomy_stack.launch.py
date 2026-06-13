from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_livox = LaunchConfiguration("use_livox")
    livox_config = LaunchConfiguration("livox_config")
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
    target_point_topic = LaunchConfiguration("target_point_topic")
    accept_runtime_target = LaunchConfiguration("accept_runtime_target")
    launch_obstacle_sim = LaunchConfiguration("launch_obstacle_sim")
    obstacle_sim_mode = LaunchConfiguration("obstacle_sim_mode")

    default_livox_config = PathJoinSubstitution([
        FindPackageShare("livox_ros_driver2"),
        "config",
        "MID360_config.json",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_livox", default_value="false"),
        DeclareLaunchArgument("livox_config", default_value=default_livox_config),
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
        DeclareLaunchArgument("target_point_topic", default_value="/autonomy/target_ned"),
        DeclareLaunchArgument("accept_runtime_target", default_value="true"),
        DeclareLaunchArgument("launch_obstacle_sim", default_value="false"),
        DeclareLaunchArgument("obstacle_sim_mode", default_value="safe"),
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
                "lvx_file_path": "/home/livox/livox_test.lvx",
                "user_config_path": livox_config,
                "cmdline_input_bd_code": "livox0000000001",
            }],
        ),
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
                "obstacle_stop_distance_m": obstacle_stop_distance_m,
                "obstacle_abort_distance_m": obstacle_abort_distance_m,
                "obstacle_hold_timeout_s": obstacle_hold_timeout_s,
                "enable_obstacle_hold": enable_obstacle_hold,
                "target_point_topic": target_point_topic,
                "accept_runtime_target": accept_runtime_target,
            }],
        ),
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
    ])
