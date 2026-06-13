import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('tf2_eigen_tuner')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'default.rviz')

    # 1. 坐标轴发布者节点
    axes_node = Node(
        package='tf2_eigen_tuner',
        executable='axes_publisher_node',
        name='axes_publisher_node',
        output='screen'
    )

    # 2. RViz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # 3. 新增 rqt_reconfigure 节点（一键直接调出调参界面）
    rqt_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        output='screen'
    )

    return LaunchDescription([
        axes_node,
        rviz_node,
        rqt_node
    ])

