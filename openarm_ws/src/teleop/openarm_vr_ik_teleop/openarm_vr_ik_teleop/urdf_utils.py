"""URDF 加载与 mesh 替换，供 check_urdf_placo 与 ik_teleop_node 共用。"""

import os
import re
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory

import placo


def _strip_meshes_for_placo(urdf_str: str) -> str:
    """将 mesh 替换为 box，规避 Assimp 解析 .dae 时的 'Not enough data for accessor' 错误。"""
    return re.sub(
        r'<mesh\s+filename="[^"]+"\s*scale="[^"]+"\s*/>',
        '<box size="0.001 0.001 0.001"/>',
        urdf_str,
    )


def build_openarm_urdf_path() -> str:
    """生成 OpenArm 双臂 URDF 并返回临时文件路径。调用方负责 os.unlink。"""
    desc_share = get_package_share_directory("openarm_description")
    xacro_path = os.path.join(desc_share, "urdf", "robot", "v10.urdf.xacro")

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": "v10",
            "bimanual": "true",
            "use_fake_hardware": "false",
            "ros2_control": "false",
            "right_can_interface": "can0",
            "left_can_interface": "can1",
        },
    ).toprettyxml(indent="  ")

    robot_description = _strip_meshes_for_placo(robot_description)

    f = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
    f.write(robot_description)
    f.close()
    return f.name


LEFT_ARM_JOINT_NAMES = [
    "openarm_left_joint1",
    "openarm_left_joint2",
    "openarm_left_joint3",
    "openarm_left_joint4",
    "openarm_left_joint5",
    "openarm_left_joint6",
    "openarm_left_joint7",
]
RIGHT_ARM_JOINT_NAMES = [
    "openarm_right_joint1",
    "openarm_right_joint2",
    "openarm_right_joint3",
    "openarm_right_joint4",
    "openarm_right_joint5",
    "openarm_right_joint6",
    "openarm_right_joint7",
]


# Placo state.q 布局（固定基座）：q[0:7] 基座，q[7:14] 左臂7关节，q[14:16] 左指，q[16:23] 右臂7关节，q[23:25] 右指
LEFT_ARM_Q_SLICE = slice(7, 14)
RIGHT_ARM_Q_SLICE = slice(16, 23)
