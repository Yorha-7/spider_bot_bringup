from setuptools import find_packages, setup
from glob import glob
import os

package_name = "big_bertha_hardware_bringup"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jayesh",
    maintainer_email="scientistn1420@gmail.com",
    description="Hardware interface for Big Bertha: MPU6050 IMU + PCA9685 servo driver",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "hardware_bridge = big_bertha_hardware_bringup.hardware_bridge_node:main",
        ],
    },
)
