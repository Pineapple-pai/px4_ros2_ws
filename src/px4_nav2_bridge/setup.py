from setuptools import setup

package_name = "px4_nav2_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="p",
    maintainer_email="p@example.com",
    description="Bridge QGC/PX4 reposition commands into Navigation2 goals.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "qgc_reposition_goal_bridge = px4_nav2_bridge.qgc_reposition_goal_bridge:main",
            "px4_local_position_nav2_bridge = px4_nav2_bridge.px4_local_position_nav2_bridge:main",
            "odom_path_visualizer = px4_nav2_bridge.odom_path_visualizer:main",
        ],
    },
)
