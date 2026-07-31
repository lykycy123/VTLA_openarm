#!/usr/bin/env python3
"""
阶段 3 IK 遥操作节点：订阅 VR 话题，Placo IK 解算，发布到 forward_position_controller。

订阅：/joint_states, /vr/left_controller/pose, /vr/right_controller/pose,
      /vr/left_grip, /vr/right_grip, /vr/left_gripper, /vr/right_gripper
发布：/left_forward_position_controller/commands, /right_forward_position_controller/commands
Action：left_gripper_controller, right_gripper_controller (GripperCommand)
"""

import os

import numpy as np
import placo
import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray

from openarm_vr_ik_teleop.geometry_utils import (
    R_HEADSET_TO_WORLD,
    apply_delta_pose,
    quat_diff_as_angle_axis,
    quat_from_rot,
)
from openarm_vr_ik_teleop.urdf_utils import (
    LEFT_ARM_Q_SLICE,
    RIGHT_ARM_Q_SLICE,
    build_openarm_urdf_path,
)


def _quat_to_4x4(q):
    """四元数 (w,x,y,z) 转 4x4 齐次矩阵。"""
    w, x, y, z = q[0], q[1], q[2], q[3]
    R = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    T = np.eye(4)
    T[:3, :3] = R
    return T


class IkTeleopNode(Node):
    def __init__(self):
        super().__init__("ik_teleop_node")

        self.declare_parameter("scale_factor", 0.8)
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("grip_threshold", 0.9)
        self.declare_parameter("gripper_open_m", 0.044)
        self.declare_parameter("gripper_closed_m", 0.0)
        self.declare_parameter("gripper_update_throttle_s", 0.1)
        scale = float(os.environ.get("IK_SCALE_FACTOR", self.get_parameter("scale_factor").value))
        rate = self.get_parameter("control_rate").value
        self.grip_threshold = float(os.environ.get("IK_GRIP_THRESHOLD", self.get_parameter("grip_threshold").value))
        self.gripper_open_m = self.get_parameter("gripper_open_m").value
        self.gripper_closed_m = self.get_parameter("gripper_closed_m").value
        self.gripper_throttle_s = self.get_parameter("gripper_update_throttle_s").value

        urdf_path = build_openarm_urdf_path()
        try:
            self.robot = placo.RobotWrapper(urdf_path)
        finally:
            os.unlink(urdf_path)

        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.dt = 1.0 / rate
        self.solver.mask_fbase(True)

        # 初始 q：基座单位四元数 + 零位
        self.robot.state.q[:7] = np.array([0, 0, 0, 0, 0, 0, 1])
        self.robot.update_kinematics()

        self.left_task = self.solver.add_frame_task(
            "openarm_left_link7",
            self.robot.get_T_world_frame("openarm_left_link7"),
        )
        self.left_task.configure("left_ee", "soft", 1.0)
        self.right_task = self.solver.add_frame_task(
            "openarm_right_link7",
            self.robot.get_T_world_frame("openarm_right_link7"),
        )
        self.right_task.configure("right_ee", "soft", 1.0)

        # 冗余自由度：仅用 kinetic_energy 正则。manipulability/joints_task 与 OpenArm 双臂模型
        # 组合易触发 QP NaN，故不启用。若有关节跳变，可降低 scale_factor。
        self.solver.add_kinetic_energy_regularization_task(1e-6)

        # VR 遥操作专用：收紧左右 joint4 的 lower，防止 IK 解算跳变（仅本节点生效）
        joint_names = list(self.robot.model.names)
        for j in ("openarm_left_joint4", "openarm_right_joint4"):
            if j in joint_names:
                self.robot.set_joint_limits(j, 0.15, 2.443461)

        self.scale = scale
        self.ref_left = None  # (xyz, quat)
        self.ref_right = None
        self.ref_ctrl_left = None  # (xyz, quat) 手柄首次激活时的位姿
        self.ref_ctrl_right = None
        self.active_left = False
        self.active_right = False

        self.joint_state = None
        self._joint_state_lock = None
        import threading

        self._joint_state_lock = threading.Lock()

        self.sub_joint = self.create_subscription(
            JointState, "/joint_states", self._cb_joint_states, 10
        )
        self.sub_left_pose = self.create_subscription(
            PoseStamped, "/vr/left_controller/pose", self._cb_left_pose, 10
        )
        self.sub_right_pose = self.create_subscription(
            PoseStamped, "/vr/right_controller/pose", self._cb_right_pose, 10
        )
        self.sub_left_grip = self.create_subscription(
            Float32, "/vr/left_grip", self._cb_left_grip, 10
        )
        self.sub_right_grip = self.create_subscription(
            Float32, "/vr/right_grip", self._cb_right_grip, 10
        )
        self.sub_left_gripper = self.create_subscription(
            Float32, "/vr/left_gripper", self._cb_left_gripper, 10
        )
        self.sub_right_gripper = self.create_subscription(
            Float32, "/vr/right_gripper", self._cb_right_gripper, 10
        )

        self._action_left_gripper = ActionClient(
            self, GripperCommand, "left_gripper_controller/gripper_cmd"
        )
        self._action_right_gripper = ActionClient(
            self, GripperCommand, "right_gripper_controller/gripper_cmd"
        )

        self.pub_left_cmd = self.create_publisher(
            Float64MultiArray, "/left_forward_position_controller/commands", 10
        )
        self.pub_right_cmd = self.create_publisher(
            Float64MultiArray, "/right_forward_position_controller/commands", 10
        )

        self._left_pose = None
        self._right_pose = None
        self._left_grip = 0.0
        self._right_grip = 0.0
        self._left_gripper = 0.0
        self._right_gripper = 0.0
        self._last_left_gripper_goal = None
        self._last_right_gripper_goal = None
        self._last_left_gripper_time = 0.0
        self._last_right_gripper_time = 0.0

        self.timer = self.create_timer(1.0 / rate, self._timer_callback)
        self.get_logger().info(
            "IK teleop node started, scale=%.2f, rate=%.0f Hz" % (scale, rate)
        )

    def _cb_joint_states(self, msg: JointState):
        with self._joint_state_lock:
            self.joint_state = msg

    def _cb_left_pose(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        self._left_pose = (np.array([p.x, p.y, p.z]), np.array([o.w, o.x, o.y, o.z]))

    def _cb_right_pose(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        self._right_pose = (np.array([p.x, p.y, p.z]), np.array([o.w, o.x, o.y, o.z]))

    def _cb_left_grip(self, msg: Float32):
        self._left_grip = msg.data

    def _cb_right_grip(self, msg: Float32):
        self._right_grip = msg.data

    def _cb_left_gripper(self, msg: Float32):
        self._left_gripper = max(0.0, min(1.0, msg.data))

    def _cb_right_gripper(self, msg: Float32):
        self._right_gripper = max(0.0, min(1.0, msg.data))

    def _gripper_value_to_position(self, value: float) -> float:
        """将 0~1 扳机值映射为夹爪开口 (m)。按下扳机(1)=闭合，松开(0)=全开。"""
        return self.gripper_closed_m + (1.0 - value) * (self.gripper_open_m - self.gripper_closed_m)

    def _send_gripper_goal(self, client, value: float, last_goal, last_time: float) -> tuple:
        """按节流发送夹爪目标。返回 (new_last_goal, new_last_time)。"""
        pos = self._gripper_value_to_position(value)
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - last_time
        changed = last_goal is None or abs(pos - last_goal) > 0.002
        if not changed and dt < self.gripper_throttle_s:
            return last_goal, last_time
        if not client.server_is_ready():
            return last_goal, last_time
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = pos
        goal_msg.command.max_effort = 0.0
        client.send_goal_async(goal_msg)
        return pos, now

    def _sync_from_joint_states(self):
        """用 /joint_states 更新 Placo 的 q。"""
        if self.joint_state is None:
            return
        with self._joint_state_lock:
            names = list(self.joint_state.name)
            pos = list(self.joint_state.position)
        for i, name in enumerate(
            [
                "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
                "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
                "openarm_left_joint7",
            ]
        ):
            if name in names:
                self.robot.state.q[7 + i] = pos[names.index(name)]
        for i, name in enumerate(
            [
                "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
                "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
                "openarm_right_joint7",
            ]
        ):
            if name in names:
                self.robot.state.q[16 + i] = pos[names.index(name)]
        self.robot.update_kinematics()

    def _get_ee_pose(self, link: str):
        """获取末端 link 的世界位姿 (xyz, quat_wxyz)。"""
        T = self.robot.get_T_world_frame(link)
        xyz = T[:3, 3].copy()
        R = T[:3, :3]
        # 旋转矩阵转四元数 (w,x,y,z)
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
        return xyz, np.array([w, x, y, z])

    def _process_controller_pose(
        self, ctrl_xyz, ctrl_quat,
        ref_ee_xyz, ref_ee_quat,
        ref_ctrl_xyz, ref_ctrl_quat,
    ):
        """计算从 ref 到当前的 delta，并应用到 EE 目标。"""
        ctrl_xyz_world = R_HEADSET_TO_WORLD @ ctrl_xyz
        R_q = quat_from_rot(R_HEADSET_TO_WORLD)
        ctrl_quat_world = _quat_multiply(
            _quat_multiply(R_q, ctrl_quat), _quat_inverse(R_q)
        )
        delta_xyz = (ctrl_xyz_world - ref_ctrl_xyz) * self.scale
        delta_rot = quat_diff_as_angle_axis(ref_ctrl_quat, ctrl_quat_world)
        return apply_delta_pose(
            ref_ee_xyz, ref_ee_quat, delta_xyz, delta_rot
        )

    def _timer_callback(self):
        self._sync_from_joint_states()

        # 夹爪：扳机 0~1 映射为开口，节流发送 Action（与 IK 解算独立）
        self._last_left_gripper_goal, self._last_left_gripper_time = self._send_gripper_goal(
            self._action_left_gripper, self._left_gripper,
            self._last_left_gripper_goal, self._last_left_gripper_time,
        )
        self._last_right_gripper_goal, self._last_right_gripper_time = self._send_gripper_goal(
            self._action_right_gripper, self._right_gripper,
            self._last_right_gripper_goal, self._last_right_gripper_time,
        )

        if self._left_pose is not None:
            ctrl_xyz, ctrl_quat = self._left_pose
            if self._left_grip > self.grip_threshold:
                if not self.active_left:
                    self.active_left = True
                    self.ref_left = self._get_ee_pose("openarm_left_link7")
                    R_q = quat_from_rot(R_HEADSET_TO_WORLD)
                    self.ref_ctrl_left = (
                        R_HEADSET_TO_WORLD @ ctrl_xyz,
                        _quat_multiply(
                            _quat_multiply(R_q, ctrl_quat),
                            _quat_inverse(R_q),
                        ),
                    )
                target_xyz, target_quat = self._process_controller_pose(
                    ctrl_xyz, ctrl_quat,
                    self.ref_left[0], self.ref_left[1],
                    self.ref_ctrl_left[0], self.ref_ctrl_left[1],
                )
                T = _quat_to_4x4(target_quat)
                T[:3, 3] = target_xyz
                self.left_task.T_world_frame = T
            else:
                if self.active_left:
                    self.active_left = False
                    self.ref_left = None
                    self.ref_ctrl_left = None

        if self._right_pose is not None:
            ctrl_xyz, ctrl_quat = self._right_pose
            if self._right_grip > self.grip_threshold:
                if not self.active_right:
                    self.active_right = True
                    self.ref_right = self._get_ee_pose("openarm_right_link7")
                    R_q = quat_from_rot(R_HEADSET_TO_WORLD)
                    self.ref_ctrl_right = (
                        R_HEADSET_TO_WORLD @ ctrl_xyz,
                        _quat_multiply(
                            _quat_multiply(R_q, ctrl_quat),
                            _quat_inverse(R_q),
                        ),
                    )
                target_xyz, target_quat = self._process_controller_pose(
                    ctrl_xyz, ctrl_quat,
                    self.ref_right[0], self.ref_right[1],
                    self.ref_ctrl_right[0], self.ref_ctrl_right[1],
                )
                T = _quat_to_4x4(target_quat)
                T[:3, 3] = target_xyz
                self.right_task.T_world_frame = T
            else:
                if self.active_right:
                    self.active_right = False
                    self.ref_right = None
                    self.ref_ctrl_right = None

        if self.joint_state is None:
            return

        try:
            self.solver.solve(True)
        except RuntimeError as e:
            self.get_logger().warn("IK solve failed: %s" % e)
            return

        left_cmd = Float64MultiArray()
        left_cmd.data = list(self.robot.state.q[LEFT_ARM_Q_SLICE])
        self.pub_left_cmd.publish(left_cmd)
        right_cmd = Float64MultiArray()
        right_cmd.data = list(self.robot.state.q[RIGHT_ARM_Q_SLICE])
        self.pub_right_cmd.publish(right_cmd)

def _quat_inverse(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _quat_multiply(q1, q2):
    w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]
    x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]
    y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]
    z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
    return np.array([w, x, y, z])


def main(args=None):
    rclpy.init(args=args)
    node = IkTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
