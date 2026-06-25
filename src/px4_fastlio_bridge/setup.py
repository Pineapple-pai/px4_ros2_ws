from glob import glob
from setuptools import setup

package_name = "px4_fastlio_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="p",
    maintainer_email="p@example.com",
    description="Bridge utilities for running FAST-LIO2 with the PX4 ROS 2 autonomy stack.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "lio_odometry_bridge = px4_fastlio_bridge.lio_odometry_bridge:main",
            "px4_imu_bridge = px4_fastlio_bridge.px4_imu_bridge:main",
        ],
    },
)
