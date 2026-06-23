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
    ])
