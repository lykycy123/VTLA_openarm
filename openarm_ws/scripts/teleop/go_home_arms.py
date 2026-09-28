#!/usr/bin/env python3
"""在 forward_position_controller 仍运行时，将双臂插值到 MoveIt home（全零）。

供菜单 0 / 关闭 VR 前调用，避免与 bringup 一同被杀导致回不到 home。
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

HOME = np.zeros(7, dtype=np.float64)
LEFT_NAMES = [
    "openarm_left_joint1",
    "openarm_left_joint2",
    "openarm_left_joint3",
    "openarm_left_joint4",
    "openarm_left_joint5",
    "openarm_left_joint6",
    "openarm_left_joint7",
]
RIGHT_NAMES = [
    "openarm_right_joint1",
    "openarm_right_joint2",
    "openarm_right_joint3",
    "openarm_right_joint4",
    "openarm_right_joint5",
    "openarm_right_joint6",
    "openarm_right_joint7",
]


class GoHomeNode(Node):
    def __init__(self) -> None:
        super().__init__("go_home_arms")
        self._joint_state: JointState | None = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.pub_left = self.create_publisher(
            Float64MultiArray, "/left_forward_position_controller/commands", 10
        )
        self.pub_right = self.create_publisher(
            Float64MultiArray, "/right_forward_position_controller/commands", 10
        )

    def _on_js(self, msg: JointState) -> None:
        self._joint_state = msg

    def _q_from_js(self, names: list[str]) -> np.ndarray:
        q = HOME.copy()
        if self._joint_state is None:
            return q
        js_names = list(self._joint_state.name)
        js_pos = list(self._joint_state.position)
        for i, name in enumerate(names):
            if name in js_names:
                q[i] = float(js_pos[js_names.index(name)])
        return q

    def _publish(self, pub, q: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = [float(x) for x in q]
        pub.publish(msg)

    def run(self, duration_s: float = 7.0) -> None:
        deadline = time.monotonic() + 2.0
        while self._joint_state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        start_l = self._q_from_js(LEFT_NAMES)
        start_r = self._q_from_js(RIGHT_NAMES)
        self.get_logger().info(
            "Moving both arms to home over %.1fs (controllers must stay up)..."
            % duration_s
        )

        t0 = time.monotonic()
        while True:
            alpha = (time.monotonic() - t0) / max(0.5, duration_s)
            if alpha >= 1.0:
                self._publish(self.pub_left, HOME)
                self._publish(self.pub_right, HOME)
                break
            s = alpha * alpha * (3.0 - 2.0 * alpha)
            self._publish(self.pub_left, (1.0 - s) * start_l + s * HOME)
            self._publish(self.pub_right, (1.0 - s) * start_r + s * HOME)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

        for _ in range(10):
            self._publish(self.pub_left, HOME)
            self._publish(self.pub_right, HOME)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)
        self.get_logger().info("Both arms commanded to home.")


def main() -> int:
    rclpy.init()
    node = GoHomeNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
