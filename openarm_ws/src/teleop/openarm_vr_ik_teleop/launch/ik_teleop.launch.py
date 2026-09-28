# Launch file for IK teleop node (stage 4 full-chain).
# Use: conda activate openarm && ros2 launch openarm_vr_ik_teleop ik_teleop.launch.py
# Tuning: ros2 launch openarm_vr_ik_teleop ik_teleop.launch.py scale_factor:=0.6

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    script_path = PathJoinSubstitution([
        FindPackageShare("openarm_vr_ik_teleop"),
        "scripts",
        "run_ik_teleop.sh",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "scale_factor",
            default_value="1.0",
            description="VR delta to robot motion scale (0.5~1.0).",
        ),
        DeclareLaunchArgument(
            "grip_threshold",
            default_value="0.9",
            description="Grip value above which arm control is active.",
        ),
        SetEnvironmentVariable(
            "IK_SCALE_FACTOR",
            LaunchConfiguration("scale_factor"),
        ),
        SetEnvironmentVariable(
            "IK_GRIP_THRESHOLD",
            LaunchConfiguration("grip_threshold"),
        ),
        ExecuteProcess(
            cmd=["bash", script_path],
            output="screen",
        ),
    ])
