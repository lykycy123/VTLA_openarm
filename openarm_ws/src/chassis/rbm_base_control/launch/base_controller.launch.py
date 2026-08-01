from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = Path(get_package_share_directory("rbm_base_control")) / "config" / "base_controller.yaml"
    return LaunchDescription(
        [
            Node(
                package="rbm_base_control",
                executable="base_controller",
                name="base_controller",
                output="screen",
                parameters=[str(config)],
            )
        ]
    )
