from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("topic", default_value="/perception/min_obstacle_distance"),
        DeclareLaunchArgument("mode", default_value="safe"),
        DeclareLaunchArgument("safe_distance_m", default_value="5.0"),
        DeclareLaunchArgument("hold_distance_m", default_value="1.5"),
        DeclareLaunchArgument("abort_distance_m", default_value="0.5"),
        DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("wave_period_s", default_value="12.0"),
        Node(
            package="px4_obstacle_tools",
            executable="obstacle_distance_sim",
            name="obstacle_distance_sim",
            output="screen",
            parameters=[{
                "topic": LaunchConfiguration("topic"),
                "mode": LaunchConfiguration("mode"),
                "safe_distance_m": LaunchConfiguration("safe_distance_m"),
                "hold_distance_m": LaunchConfiguration("hold_distance_m"),
                "abort_distance_m": LaunchConfiguration("abort_distance_m"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "wave_period_s": LaunchConfiguration("wave_period_s"),
            }],
        ),
    ])
