from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("big_bertha_hardware_bringup"),
        "config",
        "hardware_bridge.yaml",
    )

    return LaunchDescription([
        Node(
            package="big_bertha_hardware_bringup",
            executable="hardware_bridge",
            name="hardware_bridge",
            parameters=[config_path],
            output="screen",
        ),
    ])
