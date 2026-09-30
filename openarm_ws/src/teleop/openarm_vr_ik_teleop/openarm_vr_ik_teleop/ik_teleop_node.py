#!/usr/bin/env python3
"""
阶段 3 IK 遥操作节点：订阅 VR 话题，Placo IK 解算，发布到 forward_position_controller。

订阅：/joint_states, /vr/left_controller/pose, /vr/right_controller/pose,
      /vr/left_grip, /vr/right_grip, /vr/left_gripper, /vr/right_gripper,
      /vr/controller_input
发布：/left_forward_position_controller/commands, /right_forward_position_controller/commands
Action：left_gripper_controller, right_gripper_controller (GripperCommand)

启动后插值到 MoveIt「hands_up」，握把激活后从当前末端增量跟随（缓入，避免接手抖动）。
运行中复位：X=左臂 hands_up，A=右臂 hands_up。
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
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
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

        self.declare_parameter("scale_factor", 1.0)
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("grip_threshold", 0.9)
        self.declare_parameter("gripper_open_m", 0.044)
        self.declare_parameter("gripper_closed_m", 0.0)
        self.declare_parameter("gripper_update_throttle_s", 0.1)
        self.declare_parameter("hands_up_duration_s", 2.0)
        self.declare_parameter("home_duration_s", 2.5)
        self.declare_parameter("max_joint_vel_rad_s", 3.0)
        self.declare_parameter("vr_pose_lpf", 0.50)
        self.declare_parameter("grip_release_threshold", 0.55)
        self.declare_parameter("engage_blend_s", 0.20)
        scale = float(os.environ.get("IK_SCALE_FACTOR", self.get_parameter("scale_factor").value))
        rate = float(self.get_parameter("control_rate").value)
        self.dt = 1.0 / max(1.0, rate)
        self.grip_threshold = float(os.environ.get("IK_GRIP_THRESHOLD", self.get_parameter("grip_threshold").value))
        self.gripper_open_m = self.get_parameter("gripper_open_m").value
        self.gripper_closed_m = self.get_parameter("gripper_closed_m").value
        self.gripper_throttle_s = self.get_parameter("gripper_update_throttle_s").value
        self.hands_up_duration_s = float(self.get_parameter("hands_up_duration_s").value)
        self.home_duration_s = float(self.get_parameter("home_duration_s").value)
        self.max_joint_vel = float(self.get_parameter("max_joint_vel_rad_s").value)
        self.vr_pose_lpf = float(self.get_parameter("vr_pose_lpf").value)
        self.grip_release_threshold = float(self.get_parameter("grip_release_threshold").value)
        self.engage_blend_s = float(self.get_parameter("engage_blend_s").value)
        self._shutting_down = False

        urdf_path = build_openarm_urdf_path()
        try:
            self.robot = placo.RobotWrapper(urdf_path)
        finally:
            os.unlink(urdf_path)

        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.dt = self.dt
        self.solver.mask_fbase(True)

        # 初始 q 与 hands_up 对齐，任务目标与遥操作起始姿态一致
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

        # 冗余自由度：仅用 kinetic_energy 正则。权重大于 1e-6，减轻奇异附近抖动。
        self.solver.add_kinetic_energy_regularization_task(1e-4)

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
        self._seeded_from_joints = False
        self._prev_btn_a = 0
        self._prev_btn_x = 0
        self._last_cmd_left = HANDS_UP_LEFT.copy()
        self._last_cmd_right = HANDS_UP_RIGHT.copy()
        self._filt_left_pose = None
        self._filt_right_pose = None
        self._engage_left_t0 = None
        self._engage_right_t0 = None

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

        # 夹爪 action 客户端单独放在一个节点里，并用独立执行器线程 spin。
        # 原因：rclpy 只要订阅了 gripper action 的 status 话题（ActionClient 内部会订阅），
        # 执行器就会忙轮询（实测 ~65% 一个核；两个夹爪客户端 ~93%）。
        # 若与 50Hz 控制循环共用同一个执行器，定时器会被饿死（指令实测掉到 ~6Hz），
        # 表现就是"刚开始正常、越用机械臂越跟不上"。
        self._gripper_node = rclpy.create_node("ik_gripper_action_client")
        self._action_left_gripper = ActionClient(
            self._gripper_node, GripperCommand, "left_gripper_controller/gripper_cmd"
        )
        self._action_right_gripper = ActionClient(
            self._gripper_node, GripperCommand, "right_gripper_controller/gripper_cmd"
        )
        self._gripper_executor = SingleThreadedExecutor()
        self._gripper_executor.add_node(self._gripper_node)
        self._gripper_thread = threading.Thread(
            target=self._spin_gripper_clients, daemon=True
        )
        self._gripper_thread.start()

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

        self.timer = self.create_timer(self.dt, self._timer_callback)
        self.get_logger().info(
            "IK teleop node started, scale=%.2f, rate=%.0f Hz; "
            "hold hands_up; grip follow with blend-in; X/A→hands_up; exit→home"
            % (scale, rate)
        )

    def _now(self) -> float:
        return time.monotonic()

    def _cb_joint_states(self, msg: JointState):
        with self._joint_state_lock:
            self.joint_state = msg
        if not self._seeded_from_joints:
            left = self._arm_q_from_joint_states("left", HOME_LEFT)
            right = self._arm_q_from_joint_states("right", HOME_RIGHT)
            if left is None or right is None:
                return
            self._last_cmd_left = left.copy()
            self._last_cmd_right = right.copy()
            self.robot.state.q[LEFT_ARM_Q_SLICE] = left
            self.robot.state.q[RIGHT_ARM_Q_SLICE] = right
            self.robot.update_kinematics()
            self.left_task.T_world_frame = self.robot.get_T_world_frame(
                "openarm_left_link7"
            )
            self.right_task.T_world_frame = self.robot.get_T_world_frame(
                "openarm_right_link7"
            )
            self._seeded_from_joints = True
            self.get_logger().info(
                "Seeded IK from /joint_states; moving both arms to hands_up"
            )
            self._start_hands_up("left", reason="startup")
            self._start_hands_up("right", reason="startup")

    def _cb_left_pose(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        raw = (np.array([p.x, p.y, p.z]), np.array([o.w, o.x, o.y, o.z]))
        self._filt_left_pose = self._filter_pose(self._filt_left_pose, raw[0], raw[1])
        self._left_pose = self._filt_left_pose

    def _cb_right_pose(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        raw = (np.array([p.x, p.y, p.z]), np.array([o.w, o.x, o.y, o.z]))
        self._filt_right_pose = self._filter_pose(self._filt_right_pose, raw[0], raw[1])
        self._right_pose = self._filt_right_pose

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
            self._engage_left_t0 = None
        else:
            self._home_right = move
            self.active_right = False
            self.ref_right = None
            self.ref_ctrl_right = None
            self._engage_right_t0 = None
        self.get_logger().info(
            "Moving %s arm to hands_up (%s)" % (side, reason or "request")
        )

    def _publish_arm_cmd(self, side: str, q: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = [float(x) for x in q]
        if side == "left":
            self._last_cmd_left = np.asarray(q, dtype=np.float64).copy()
            self.pub_left_cmd.publish(msg)
        else:
            self._last_cmd_right = np.asarray(q, dtype=np.float64).copy()
            self.pub_right_cmd.publish(msg)

    def _slew_toward(self, current: np.ndarray, target: np.ndarray) -> np.ndarray:
        max_step = max(1e-4, self.max_joint_vel) * self.dt
        delta = np.asarray(target, dtype=np.float64) - np.asarray(current, dtype=np.float64)
        mag = float(np.max(np.abs(delta)))
        if mag <= max_step:
            return np.asarray(target, dtype=np.float64).copy()
        return np.asarray(current, dtype=np.float64) + delta * (max_step / mag)

    def _filter_pose(self, prev, xyz: np.ndarray, quat: np.ndarray):
        alpha = min(1.0, max(0.05, self.vr_pose_lpf))
        if prev is None:
            return xyz.copy(), quat.copy()
        pxyz, pquat = prev
        xyz_f = (1.0 - alpha) * pxyz + alpha * xyz
        q = quat.copy()
        if float(np.dot(pquat, q)) < 0.0:
            q = -q
        quat_f = (1.0 - alpha) * pquat + alpha * q
        n = float(np.linalg.norm(quat_f))
        if n < 1e-9:
            quat_f = pquat
        else:
            quat_f = quat_f / n
        return xyz_f, quat_f

    def _grip_wants(self, side: str, homing: bool) -> bool:
        if homing or self._shutting_down:
            return False
        pose = self._left_pose if side == "left" else self._right_pose
        if pose is None:
            return False
        grip = self._left_grip if side == "left" else self._right_grip
        active = self.active_left if side == "left" else self.active_right
        if active:
            return grip > self.grip_release_threshold
        return grip > self.grip_threshold

    def _engage_blend(self, t0) -> float:
        if t0 is None:
            return 1.0
        duration = max(0.05, self.engage_blend_s)
        alpha = (self._now() - t0) / duration
        if alpha >= 1.0:
            return 1.0
        if alpha <= 0.0:
            return 0.0
        return float(alpha * alpha * (3.0 - 2.0 * alpha))

    def _blend_quat(self, q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
        q1 = q1.copy()
        if float(np.dot(q0, q1)) < 0.0:
            q1 = -q1
        q = (1.0 - alpha) * q0 + alpha * q1
        n = float(np.linalg.norm(q))
        if n < 1e-9:
            return q0.copy()
        return q / n

    def _set_arm_q(self, side: str, q: np.ndarray) -> None:
        if side == "left":
            self.robot.state.q[LEFT_ARM_Q_SLICE] = q
        else:
            self.robot.state.q[RIGHT_ARM_Q_SLICE] = q

    def _arm_q_from_solver(self, side: str) -> np.ndarray:
        sl = LEFT_ARM_Q_SLICE if side == "left" else RIGHT_ARM_Q_SLICE
        return np.asarray(self.robot.state.q[sl], dtype=np.float64).copy()

    def _update_hands_up(self, side: str) -> bool:
        """推进 hands_up 插值；返回 True 表示本周期该臂仍在复位中。"""
        home = self._home_left if side == "left" else self._home_right
        if home is None:
            return False
        duration = max(0.2, self.hands_up_duration_s)
        alpha = (self._now() - home["t0"]) / duration
        if alpha >= 1.0:
            q = home["target"]
            self._set_arm_q(side, q)
            self.robot.update_kinematics()
            self._publish_arm_cmd(side, q)
            if side == "left":
                self._home_left = None
            else:
                self._home_right = None
            return False
        s = alpha * alpha * (3.0 - 2.0 * alpha)
        q = (1.0 - s) * home["start"] + s * home["target"]
        self._set_arm_q(side, q)
        self._publish_arm_cmd(side, q)
        return True

    def stop_publishing(self) -> None:
        """立刻停止周期发布，避免退出时与外部 go_home 抢同一控制器。"""
        self._shutting_down = True
        self.active_left = False
        self.active_right = False
        self._home_left = None
        self._home_right = None
        try:
            self.timer.cancel()
        except Exception:
            pass
        try:
            self._gripper_executor.shutdown(timeout_sec=0.5)
        except Exception:
            pass

    def destroy_gripper_node(self) -> None:
        """释放为夹爪 action 客户端单独创建的节点。"""
        try:
            self._gripper_executor.shutdown(timeout_sec=0.5)
        except Exception:
            pass
        try:
            self._gripper_node.destroy_node()
        except Exception:
            pass

    def _spin_gripper_clients(self) -> None:
        """独立线程里 spin 夹爪 action 客户端节点，避免拖慢主控制循环。"""
        while rclpy.ok() and not self._shutting_down:
            try:
                self._gripper_executor.spin_once(timeout_sec=0.05)
            except Exception:
                break

    def go_home_blocking(self) -> None:
        """退出前将双臂插值到 MoveIt home（全零），阻塞直到完成。"""
        if self._shutting_down:
            return
        self.stop_publishing()

        start_l = self._last_cmd_left.copy()
        start_r = self._last_cmd_right.copy()

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
            time.sleep(dt)

        for _ in range(5):
            self._publish_arm_cmd("left", HOME_LEFT)
            self._publish_arm_cmd("right", HOME_RIGHT)
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

        want_left = self._grip_wants("left", left_homing)
        want_right = self._grip_wants("right", right_homing)

        # 未跟随的臂持续发布上一命令，避免控制器掉回全零/失力下垂
        if self._seeded_from_joints:
            if not want_left and not left_homing:
                if self.active_left:
                    self.active_left = False
                    self.ref_left = None
                    self.ref_ctrl_left = None
                    self._engage_left_t0 = None
                self._publish_arm_cmd("left", self._last_cmd_left)
            if not want_right and not right_homing:
                if self.active_right:
                    self.active_right = False
                    self.ref_right = None
                    self.ref_ctrl_right = None
                    self._engage_right_t0 = None
                self._publish_arm_cmd("right", self._last_cmd_right)

        if not want_left and not want_right:
            return

        # 用上一拍命令作为 IK 种子，不要每周期用滞后的 joint_states 重种子
        self._set_arm_q("left", self._last_cmd_left)
        self._set_arm_q("right", self._last_cmd_right)
        self.robot.update_kinematics()

        hold_left_q = self._last_cmd_left.copy()
        hold_right_q = self._last_cmd_right.copy()

        if want_left and self._left_pose is not None:
            ctrl_xyz, ctrl_quat = self._left_pose
            if not self.active_left:
                self.active_left = True
                self._engage_left_t0 = self._now()
                self._set_arm_q("left", self._last_cmd_left)
                self.robot.update_kinematics()
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
            blend = self._engage_blend(self._engage_left_t0)
            if blend < 1.0:
                target_xyz = self.ref_left[0] + blend * (target_xyz - self.ref_left[0])
                target_quat = self._blend_quat(self.ref_left[1], target_quat, blend)
            T = _quat_to_4x4(target_quat)
            T[:3, 3] = target_xyz
            self.left_task.T_world_frame = T
            self.left_task.configure("left_ee", "soft", 1.0)
        else:
            T = self.robot.get_T_world_frame("openarm_left_link7")
            self.left_task.T_world_frame = T
            self.left_task.configure("left_ee", "soft", 1e-3)

        if want_right and self._right_pose is not None:
            ctrl_xyz, ctrl_quat = self._right_pose
            if not self.active_right:
                self.active_right = True
                self._engage_right_t0 = self._now()
                self._set_arm_q("right", self._last_cmd_right)
                self.robot.update_kinematics()
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
            blend = self._engage_blend(self._engage_right_t0)
            if blend < 1.0:
                target_xyz = self.ref_right[0] + blend * (target_xyz - self.ref_right[0])
                target_quat = self._blend_quat(self.ref_right[1], target_quat, blend)
            T = _quat_to_4x4(target_quat)
            T[:3, 3] = target_xyz
            self.right_task.T_world_frame = T
            self.right_task.configure("right_ee", "soft", 1.0)
        else:
            T = self.robot.get_T_world_frame("openarm_right_link7")
            self.right_task.T_world_frame = T
            self.right_task.configure("right_ee", "soft", 1e-3)

        try:
            self.solver.solve(True)
        except RuntimeError as e:
            self.get_logger().warn("IK solve failed: %s" % e)
            if want_left and not left_homing:
                self._publish_arm_cmd("left", self._last_cmd_left)
            if want_right and not right_homing:
                self._publish_arm_cmd("right", self._last_cmd_right)
            return

        # 未激活的臂保持上一命令，避免双臂 QP 把它拽走
        if not want_left:
            self._set_arm_q("left", hold_left_q)
        if not want_right:
            self._set_arm_q("right", hold_right_q)

        if not left_homing and want_left:
            q_ik = self._arm_q_from_solver("left")
            if np.any(np.isnan(q_ik)):
                self.get_logger().warn("IK NaN on left arm, holding last command")
                self._publish_arm_cmd("left", self._last_cmd_left)
            else:
                self._publish_arm_cmd("left", self._slew_toward(self._last_cmd_left, q_ik))
        if not right_homing and want_right:
            q_ik = self._arm_q_from_solver("right")
            if np.any(np.isnan(q_ik)):
                self.get_logger().warn("IK NaN on right arm, holding last command")
                self._publish_arm_cmd("right", self._last_cmd_right)
            else:
                self._publish_arm_cmd("right", self._slew_toward(self._last_cmd_right, q_ik))

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
        # 回 home 由菜单 0 / launch 脚本在本节点停止发布之后独占执行。
        # 这里若再插值，会与 go_home_arms 抢 /commands，表现为剧烈抖动并停在 hands_up。
        try:
            node.stop_publishing()
        except Exception:
            pass
        try:
            node.destroy_gripper_node()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
