"""位姿增量计算，用于 VR 手柄增量控制。"""

import numpy as np

# VR 手柄坐标系到机器人世界坐标系的旋转
# 假设 VR (PICO/XRoboToolkit) 使用 Unity 约定: X右 Y上 Z前
# ROS/Placo 世界系: X前 Y左 Z上
# 列向量: VR +X -> world [0,-1,0], +Y -> [0,0,1], +Z -> [1,0,0]
R_HEADSET_TO_WORLD = np.array(
    [[0, 0, -1], [-1, 0, 0], [0, 1, 0]],
    dtype=np.float64,
)


def _quat_inverse(q):
    """四元数求逆，q 为 (w,x,y,z)。"""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _quat_multiply(q1, q2):
    """四元数乘法。"""
    w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]
    x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]
    y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]
    z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
    return np.array([w, x, y, z])


def _quat_about_axis(angle, axis):
    """绕轴旋转角度的单位四元数。"""
    ha = angle / 2
    s = np.sin(ha)
    return np.array([np.cos(ha), axis[0] * s, axis[1] * s, axis[2] * s])


def quat_from_rot(R):
    """3x3 旋转矩阵转四元数 (w,x,y,z)。"""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z])


def quat_diff_as_angle_axis(q1: np.ndarray, q2: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """q1 到 q2 的旋转差，表示为角轴向量。四元数 (w,x,y,z)。"""
    q1_inv = _quat_inverse(q1)
    delta_q = _quat_multiply(q2, q1_inv)
    angle = 2.0 * np.arccos(np.clip(delta_q[0], -1.0, 1.0))
    if angle < eps:
        return np.zeros(3)
    axis = delta_q[1:] / np.sin(angle / 2.0)
    return axis * angle


def apply_delta_pose(
    source_pos: np.ndarray,
    source_rot: np.ndarray,
    delta_pos: np.ndarray,
    delta_rot: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """在位姿上施加位移与旋转增量。source_rot 为 (w,x,y,z)。"""
    target_pos = source_pos + delta_pos
    angle = np.linalg.norm(delta_rot)
    if angle < 1e-6:
        return target_pos, source_rot.copy()
    axis = delta_rot / angle
    rot_delta_quat = _quat_about_axis(angle, axis)
    target_rot = _quat_multiply(rot_delta_quat, source_rot)
    return target_pos, target_rot
