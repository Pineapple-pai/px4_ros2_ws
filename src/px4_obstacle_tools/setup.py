from setuptools import setup

package_name = "px4_obstacle_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/obstacle_sim.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="p",
    maintainer_email="p@example.com",
    description="Small ROS 2 tools for simulating obstacle-distance inputs.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "gz_scan_min_distance = px4_obstacle_tools.gz_scan_min_distance:main",
            "laserscan_min_distance = px4_obstacle_tools.laserscan_min_distance:main",
            "obstacle_distance_sim = px4_obstacle_tools.obstacle_distance_sim:main",
            "pointcloud_min_distance = px4_obstacle_tools.pointcloud_min_distance:main",
        ],
    },
)
