#!/usr/bin/env python3
"""
阶段 3 IK 遥操作节点：订阅 VR 话题，Placo IK 解算，发布到 forward_position_controller。

订阅：/joint_states, /vr/left_controller/pose, /vr/right_controller/pose,
      /vr/left_grip, /vr/right_grip, /vr/left_gripper, /vr/right_gripper,
      /vr/controller_input
发布：/left_forward_position_controller/commands, /right_forward_position_controller/commands
Action：left_gripper_controller, right_gripper_controller (GripperCommand)

起始位姿与运行中复位对齐 MoveIt group_state「hands_up」：
  X 键 → 左臂回 hands_up；A 键 → 右臂回 hands_up。
退出 VR 遥操作时双臂回到 MoveIt group_state「home」（全零）。
"""

import os
import signal
import threading
import time

import numpy as np
import placo
import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float32, Float64MultiArray

from openarm_vr_ik_teleop.geometry_utils import (
    R_HEADSET_TO_WORLD,
    apply_delta_pose,
    quat_diff_as_angle_axis,
    quat_from_rot,
)
from openarm_vr_ik_teleop.urdf_utils import (
    LEFT_ARM_JOINT_NAMES,
    LEFT_ARM_Q_SLICE,
    RIGHT_ARM_JOINT_NAMES,
    RIGHT_ARM_Q_SLICE,
    build_openarm_urdf_path,
)

# 与 openarm_bimanual.srdf group_state 一致
HANDS_UP_LEFT = np.array([0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0], dtype=np.float64)
HANDS_UP_RIGHT = np.array([0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0], dtype=np.float64)
HOME_LEFT = np.zeros(7, dtype=np.float64)
HOME_RIGHT = np.zeros(7, dtype=np.float64)

# /vr/controller_input buttons: [A, B, X, Y, ...]
JOY_BTN_A = 0
JOY_BTN_X = 2


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
        self.declare_parameter("hands_up_duration_s", 2.0)
        self.declare_parameter("home_duration_s", 2.5)
        scale = float(os.environ.get("IK_SCALE_FACTOR", self.get_parameter("scale_factor").value))
        rate = self.get_parameter("control_rate").value
        self.grip_threshold = float(os.environ.get("IK_GRIP_THRESHOLD", self.get_parameter("grip_threshold").value))
        self.gripper_open_m = self.get_parameter("gripper_open_m").value
        self.gripper_closed_m = self.get_parameter("gripper_closed_m").value
        self.gripper_throttle_s = self.get_parameter("gripper_update_throttle_s").value
        self.hands_up_duration_s = float(self.get_parameter("hands_up_duration_s").value)
        self.home_duration_s = float(self.get_parameter("home_duration_s").value)
        self._shutting_down = False

        urdf_path = build_openarm_urdf_path()
        try:
            self.robot = placo.RobotWrapper(urdf_path)
        finally:
            os.unlink(urdf_path)

        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.dt = 1.0 / rate
        self.solver.mask_fbase(True)

        # 初始 q：基座单位四元数 + hands_up（与 MoveIt 一致）
        self.robot.state.q[:7] = np.array([0, 0, 0, 0, 0, 0, 1])
        self.robot.state.q[LEFT_ARM_Q_SLICE] = HANDS_UP_LEFT
        self.robot.state.q[RIGHT_ARM_Q_SLICE] = HANDS_UP_RIGHT
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
        self._joint_state_lock = threading.Lock()

        # 关节空间回 hands_up：None 或 dict(start, target, t0)
        self._home_left = None
        self._home_right = None
        self._initial_home_started = False
        self._prev_btn_a = 0
        self._prev_btn_x = 0

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
        self.sub_joy = self.create_subscription(
            Joy, "/vr/controller_input", self._cb_controller_input, 10
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
            "IK teleop node started, scale=%.2f, rate=%.0f Hz; "
            "start/reset=hands_up (X=left, A=right); exit→home"
            % (scale, rate)
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _cb_joint_states(self, msg: JointState):
        with self._joint_state_lock:
            self.joint_state = msg
        if not self._initial_home_started:
            self._initial_home_started = True
            self._start_hands_up("left", reason="startup")
            self._start_hands_up("right", reason="startup")

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

    def _cb_controller_input(self, msg: Joy):
        btn_a = int(msg.buttons[JOY_BTN_A]) if len(msg.buttons) > JOY_BTN_A else 0
        btn_x = int(msg.buttons[JOY_BTN_X]) if len(msg.buttons) > JOY_BTN_X else 0
        # X → 左臂 hands_up；A → 右臂 hands_up（上升沿触发）
        if btn_x and not self._prev_btn_x:
            self._start_hands_up("left", reason="X button")
        if btn_a and not self._prev_btn_a:
            self._start_hands_up("right", reason="A button")
        self._prev_btn_a = btn_a
        self._prev_btn_x = btn_x

    def _arm_q_from_joint_states(self, side: str, fallback: np.ndarray | None = None) -> np.ndarray | None:
        if self.joint_state is None:
            return None
        names = LEFT_ARM_JOINT_NAMES if side == "left" else RIGHT_ARM_JOINT_NAMES
        if fallback is None:
            fallback = HANDS_UP_LEFT if side == "left" else HANDS_UP_RIGHT
        with self._joint_state_lock:
            js_names = list(self.joint_state.name)
            js_pos = list(self.joint_state.position)
        q = np.zeros(7, dtype=np.float64)
        for i, name in enumerate(names):
            if name in js_names:
                q[i] = float(js_pos[js_names.index(name)])
            else:
                q[i] = float(fallback[i])
        return q

    def _start_hands_up(self, side: str, reason: str = "") -> None:
        if self._shutting_down:
            return
        start = self._arm_q_from_joint_states(side)
        if start is None:
            start = np.zeros(7, dtype=np.float64)
        target = HANDS_UP_LEFT if side == "left" else HANDS_UP_RIGHT
        move = {
            "start": start.copy(),
            "target": target.copy(),
            "t0": self._now(),
        }
        if side == "left":
            self._home_left = move
            self.active_left = False
            self.ref_left = None
            self.ref_ctrl_left = None
        else:
            self._home_right = move
            self.active_right = False
            self.ref_right = None
            self.ref_ctrl_right = None
        self.get_logger().info(
            "Moving %s arm to hands_up (%s)" % (side, reason or "request")
        )

    def _publish_arm_cmd(self, side: str, q: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = [float(x) for x in q]
        if side == "left":
            self.pub_left_cmd.publish(msg)
        else:
            self.pub_right_cmd.publish(msg)

    def _update_hands_up(self, side: str) -> bool:
        """推进 hands_up 插值；返回 True 表示本周期该臂仍在复位中。"""
        home = self._home_left if side == "left" else self._home_right
        if home is None:
            return False
        duration = max(0.2, self.hands_up_duration_s)
        alpha = (self._now() - home["t0"]) / duration
        if alpha >= 1.0:
            q = home["target"]
            self._publish_arm_cmd(side, q)
            if side == "left":
                self.robot.state.q[LEFT_ARM_Q_SLICE] = q
                self._home_left = None
            else:
                self.robot.state.q[RIGHT_ARM_Q_SLICE] = q
                self._home_right = None
            self.robot.update_kinematics()
            return False
        # smoothstep
        s = alpha * alpha * (3.0 - 2.0 * alpha)
        q = (1.0 - s) * home["start"] + s * home["target"]
        self._publish_arm_cmd(side, q)
        return True

    def go_home_blocking(self) -> None:
        """退出前将双臂插值到 MoveIt home（全零），阻塞直到完成。"""
        if self._shutting_down:
            return
        self._shutting_down = True
        self.active_left = False
        self.active_right = False
        self.ref_left = None
        self.ref_right = None
        self.ref_ctrl_left = None
        self.ref_ctrl_right = None
        self._home_left = None
        self._home_right = None

        start_l = self._arm_q_from_joint_states("left", HOME_LEFT)
        start_r = self._arm_q_from_joint_states("right", HOME_RIGHT)
        if start_l is None:
            start_l = HOME_LEFT.copy()
        if start_r is None:
            start_r = HOME_RIGHT.copy()

        duration = max(0.5, self.home_duration_s)
        self.get_logger().info(
            "VR teleop exiting: moving both arms to home (%.1fs)..." % duration
        )
        t0 = time.monotonic()
        dt = 0.02
        while True:
            alpha = (time.monotonic() - t0) / duration
            if alpha >= 1.0:
                self._publish_arm_cmd("left", HOME_LEFT)
                self._publish_arm_cmd("right", HOME_RIGHT)
                break
            s = alpha * alpha * (3.0 - 2.0 * alpha)
            self._publish_arm_cmd("left", (1.0 - s) * start_l + s * HOME_LEFT)
            self._publish_arm_cmd("right", (1.0 - s) * start_r + s * HOME_RIGHT)
            try:
                rclpy.spin_once(self, timeout_sec=0.0)
            except Exception:
                pass
            time.sleep(dt)

        # 多发几次终点，提高退出瞬间命令被控制器吃到的概率
        for _ in range(5):
            self._publish_arm_cmd("left", HOME_LEFT)
            self._publish_arm_cmd("right", HOME_RIGHT)
            try:
                rclpy.spin_once(self, timeout_sec=0.0)
            except Exception:
                pass
            time.sleep(0.05)
        self.get_logger().info("Both arms at home.")

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
        if self._shutting_down:
            return
        # 夹爪：扳机 0~1 映射为开口，节流发送 Action（与 IK 解算独立）
        self._last_left_gripper_goal, self._last_left_gripper_time = self._send_gripper_goal(
            self._action_left_gripper, self._left_gripper,
            self._last_left_gripper_goal, self._last_left_gripper_time,
        )
        self._last_right_gripper_goal, self._last_right_gripper_time = self._send_gripper_goal(
            self._action_right_gripper, self._right_gripper,
            self._last_right_gripper_goal, self._last_right_gripper_time,
        )

        left_homing = self._update_hands_up("left")
        right_homing = self._update_hands_up("right")

        want_left = (
            not left_homing
            and self._left_pose is not None
            and self._left_grip > self.grip_threshold
        )
        want_right = (
            not right_homing
            and self._right_pose is not None
            and self._right_grip > self.grip_threshold
        )

        # 未握把跟随时跳过 Placo QP（复位中的臂已在上面发命令）
        if not want_left and not want_right:
            if self.active_left or self.active_right:
                self.active_left = False
                self.active_right = False
                self.ref_left = None
                self.ref_right = None
                self.ref_ctrl_left = None
                self.ref_ctrl_right = None
            return

        self._sync_from_joint_states()
        if self.joint_state is None:
            return

        if want_left and self._left_pose is not None:
            ctrl_xyz, ctrl_quat = self._left_pose
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
        elif self.active_left and not left_homing:
            self.active_left = False
            self.ref_left = None
            self.ref_ctrl_left = None

        if want_right and self._right_pose is not None:
            ctrl_xyz, ctrl_quat = self._right_pose
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
        elif self.active_right and not right_homing:
            self.active_right = False
            self.ref_right = None
            self.ref_ctrl_right = None

        try:
            self.solver.solve(True)
        except RuntimeError as e:
            self.get_logger().warn("IK solve failed: %s" % e)
            return

        # 复位中的臂由插值独占命令，避免 IK 覆盖
        if not left_homing and want_left:
            self._publish_arm_cmd("left", self.robot.state.q[LEFT_ARM_Q_SLICE])
        if not right_homing and want_right:
            self._publish_arm_cmd("right", self.robot.state.q[RIGHT_ARM_Q_SLICE])

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

    def _request_shutdown(signum, _frame):
        # 将 SIGTERM（菜单关窗）也走优雅退出，以便先回 home
        raise KeyboardInterrupt

    try:
        signal.signal(signal.SIGTERM, _request_shutdown)
    except Exception:
        pass

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.go_home_blocking()
        except Exception as exc:
            try:
                node.get_logger().error("Failed to go home on exit: %s" % exc)
            except Exception:
                pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
