from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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

    return LaunchDescription([
        DeclareLaunchArgument("mission_size_m", default_value="8.0"),
        DeclareLaunchArgument("mission_altitude_m", default_value="4.0"),
        DeclareLaunchArgument("hold_time_s", default_value="2.0"),
        DeclareLaunchArgument("acceptance_radius_m", default_value="0.6"),
        DeclareLaunchArgument("max_horizontal_velocity_m_s", default_value="2.0"),
        DeclareLaunchArgument("max_vertical_velocity_m_s", default_value="1.0"),
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
    ])
