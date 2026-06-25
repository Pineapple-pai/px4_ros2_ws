from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_livox = LaunchConfiguration("use_livox")
    use_sim_time = LaunchConfiguration("use_sim_time")
    fastlio_config_path = LaunchConfiguration("fastlio_config_path")
    fastlio_config_file = LaunchConfiguration("fastlio_config_file")
    livox_config = LaunchConfiguration("livox_config")
    launch_rviz = LaunchConfiguration("rviz")
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
            }],
        ),
    ])
