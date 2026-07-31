#!/usr/bin/env python3
"""
阶段 2 验证脚本：用 Placo 加载 OpenArm 双臂 URDF，打印 joint 顺序与末端 link。

运行前提：source install/setup.bash
依赖：openarm_description（已编译）、placo、xacro
"""

import os
import sys

import placo

from openarm_vr_ik_teleop.urdf_utils import build_openarm_urdf_path


def main():
    try:
        urdf_path = build_openarm_urdf_path()
    except Exception as e:
        print(f"[ERROR] 无法生成 URDF，请先 source install/setup.bash: {e}")
        sys.exit(1)

    try:
        robot = placo.RobotWrapper(urdf_path)
        print("[OK] Placo 成功加载 OpenArm 双臂 URDF")
        print()

        joint_names = list(robot.model.names)
        print("Joint 顺序与数量:", len(joint_names))
        for i, name in enumerate(joint_names):
            print(f"  [{i}] {name}")
        print()

        frame_names = []
        if hasattr(robot, "frame_names"):
            frame_names = robot.frame_names()
        elif hasattr(robot.model, "frames"):
            frame_names = [f.name for f in robot.model.frames]
        if frame_names:
            print("Frame 列表 (含末端 link):")
            for name in sorted(frame_names):
                print(f"  {name}")
        print()
        print("末端 link (link7，IK 目标):")
        for name in ["openarm_left_link7", "openarm_right_link7"]:
            found = name in frame_names if frame_names else None
            if found is True:
                print(f"  [OK] {name}")
            elif found is False:
                print(f"  [?] {name} 未在 frame 列表中")
            else:
                print(f"  [预期] {name}")

        expected_joints = [
            "openarm_left_joint1",
            "openarm_left_joint2",
            "openarm_left_joint3",
            "openarm_left_joint4",
            "openarm_left_joint5",
            "openarm_left_joint6",
            "openarm_left_joint7",
            "openarm_right_joint1",
            "openarm_right_joint2",
            "openarm_right_joint3",
            "openarm_right_joint4",
            "openarm_right_joint5",
            "openarm_right_joint6",
            "openarm_right_joint7",
        ]
        missing = [j for j in expected_joints if j not in joint_names]
        if missing:
            print()
            print("[WARN] 以下 joint 未在 model 中出现:", missing)
        else:
            print()
            print("[OK] 所有预期 joint 均已加载")
    finally:
        os.unlink(urdf_path)


if __name__ == "__main__":
    main()
