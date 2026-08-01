#!/usr/bin/env python3
"""Launch XR Bridge node for publishing VR controller data to ROS 2 topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "publish_rate",
            default_value="50.0",
            description="Publish rate in Hz",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="headset_world",
            description="Frame ID for pose messages",
        ),
        Node(
            package="openarm_xr_bridge",
            executable="xr_bridge_node",
            name="xr_bridge_node",
            output="screen",
            parameters=[
                {"publish_rate": LaunchConfiguration("publish_rate")},
                {"frame_id": LaunchConfiguration("frame_id")},
            ],
        ),
    ])
