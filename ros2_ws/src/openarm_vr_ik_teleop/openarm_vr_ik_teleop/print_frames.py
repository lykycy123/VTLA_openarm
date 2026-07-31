#!/usr/bin/env python3
"""打印 VR 与 OpenArm 末端坐标系，用于排查对准问题。"""

import numpy as np

from openarm_vr_ik_teleop.geometry_utils import R_HEADSET_TO_WORLD
from openarm_vr_ik_teleop.urdf_utils import build_openarm_urdf_path

import placo
import os


def _format_axes(R, name, in_world=True):
    """R 为 3x3 旋转矩阵，列向量为各轴在目标系下的表示。"""
    x, y, z = R[:, 0], R[:, 1], R[:, 2]
    suffix = " (在 robot world 下)" if in_world else ""
    lines = [
        f"  {name}{suffix}:",
        f"    +X 轴: [{x[0]:+.3f}, {x[1]:+.3f}, {x[2]:+.3f}]",
        f"    +Y 轴: [{y[0]:+.3f}, {y[1]:+.3f}, {y[2]:+.3f}]",
        f"    +Z 轴: [{z[0]:+.3f}, {z[1]:+.3f}, {z[2]:+.3f}]",
    ]
    return "\n".join(lines)


def main():
    print()
    print("========== VR / OpenArm 坐标系 ==========")
    print()

    # VR: R_HEADSET_TO_WORLD 的列 = headset 各轴在 world 下的向量
    print("1. VR 手柄 (headset_world) 经 R_HEADSET_TO_WORLD 变换到 robot world:")
    print(_format_axes(R_HEADSET_TO_WORLD, "VR 轴在 world 下"))
    print("   即: VR +X/Y/Z 指向 world 的列向量")
    print()

    urdf_path = build_openarm_urdf_path()
    try:
        robot = placo.RobotWrapper(urdf_path)
    finally:
        os.unlink(urdf_path)
    robot.state.q[:7] = np.array([0, 0, 0, 0, 0, 0, 1])
    robot.update_kinematics()

    for link in ("openarm_left_link7", "openarm_right_link7"):
        T = robot.get_T_world_frame(link)
        R = T[:3, :3]
        xyz = T[:3, 3]
        print(f"2. {link}:")
        print(f"   位置: [{xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}]")
        print(_format_axes(R, link))
        print()

    print("========== 说明 ==========")
    print("VR 期望: 手柄向前 -> 末端向前, 手柄向右 -> 末端向右")
    print("若存在偏转, 需调整 geometry_utils.R_HEADSET_TO_WORLD")
    print()


if __name__ == "__main__":
    main()
