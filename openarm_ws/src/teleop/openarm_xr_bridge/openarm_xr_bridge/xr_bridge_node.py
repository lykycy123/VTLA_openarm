#!/usr/bin/env python3
"""
XR Bridge Node: Publishes VR controller poses and inputs to ROS 2 topics.

Requires:
- XRoboToolkit PC Service running
- xrobotoolkit_sdk in Python 3.10 conda env (ROS 2 Humble 要求，openarm 需用 python=3.10 重建)

Topics published:
- /vr/left_controller/pose, /vr/right_controller/pose (geometry_msgs/PoseStamped)
- /vr/left_grip, /vr/right_grip (std_msgs/Float32) — 握把，使能手臂运动
- /vr/left_gripper, /vr/right_gripper (std_msgs/Float32) — 扳机，映射夹爪 (0~1)
- /vr/controller_input (sensor_msgs/Joy) — 按钮 ABXY、摇杆，预留
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy


class XrBridgeNode(Node):
    def __init__(self):
        super().__init__("xr_bridge_node")

        # 30Hz 足够遥操作；全身 VR（臂+底盘）时比 50Hz 更省 CPU/DDS
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("frame_id", "headset_world")
        publish_rate = self.get_parameter("publish_rate").value
        self.frame_id = self.get_parameter("frame_id").value

        # Publishers
        self.pub_left_pose = self.create_publisher(
            PoseStamped, "/vr/left_controller/pose", 10
        )
        self.pub_right_pose = self.create_publisher(
            PoseStamped, "/vr/right_controller/pose", 10
        )
        self.pub_left_grip = self.create_publisher(Float32, "/vr/left_grip", 10)
        self.pub_right_grip = self.create_publisher(Float32, "/vr/right_grip", 10)
        self.pub_left_gripper = self.create_publisher(
            Float32, "/vr/left_gripper", 10
        )
        self.pub_right_gripper = self.create_publisher(
            Float32, "/vr/right_gripper", 10
        )
        self.pub_controller_input = self.create_publisher(
            Joy, "/vr/controller_input", 10
        )

        # Initialize XRoboToolkit SDK
        try:
            import xrobotoolkit_sdk as xrt

            self.xrt = xrt
            self.xrt.init()
            self.get_logger().info("XRoboToolkit SDK initialized.")
        except ImportError as e:
            self.get_logger().error(
                "xrobotoolkit_sdk not found. Install it from XRoboToolkit-PC-Service "
                "and run this node in the same conda/venv (e.g. conda activate openarm)."
            )
            raise
        except Exception as e:
            self.get_logger().error(
                "XRoboToolkit SDK init failed. Is XRoboToolkit PC Service running? "
                "Start the PC Service before launching this node. Error: %s" % e
            )
            raise

        # Timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info(
            "XR Bridge node started, publishing at %.1f Hz" % publish_rate
        )

    def _pose_to_msg(self, pose_array) -> PoseStamped:
        """Convert SDK pose [x, y, z, qx, qy, qz, qw] to PoseStamped."""
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(pose_array[0])
        msg.pose.position.y = float(pose_array[1])
        msg.pose.position.z = float(pose_array[2])
        msg.pose.orientation.x = float(pose_array[3])
        msg.pose.orientation.y = float(pose_array[4])
        msg.pose.orientation.z = float(pose_array[5])
        msg.pose.orientation.w = float(pose_array[6])
        return msg

    def timer_callback(self):
        try:
            left_pose = self.xrt.get_left_controller_pose()
            right_pose = self.xrt.get_right_controller_pose()
            left_grip = self.xrt.get_left_grip()
            right_grip = self.xrt.get_right_grip()
            left_trigger = self.xrt.get_left_trigger()
            right_trigger = self.xrt.get_right_trigger()
            left_axis = self.xrt.get_left_axis()
            right_axis = self.xrt.get_right_axis()
        except Exception as e:
            self.get_logger().error("XRoboToolkit SDK read failed: %s" % e)
            return

        self.pub_left_pose.publish(self._pose_to_msg(left_pose))
        self.pub_right_pose.publish(self._pose_to_msg(right_pose))

        # 握把：使能手臂运动 (grip > 0.9 激活)
        g = Float32()
        g.data = float(left_grip)
        self.pub_left_grip.publish(g)
        g.data = float(right_grip)
        self.pub_right_grip.publish(g)

        # 扳机：映射夹爪 (0~1)
        g.data = float(left_trigger)
        self.pub_left_gripper.publish(g)
        g.data = float(right_trigger)
        self.pub_right_gripper.publish(g)

        # 按钮 + 摇杆：sensor_msgs/Joy 预留
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = self.frame_id
        joy_msg.axes = [float(left_axis[0]), float(left_axis[1]), float(right_axis[0]), float(right_axis[1])]
        joy_msg.buttons = [
            int(self.xrt.get_A_button()),
            int(self.xrt.get_B_button()),
            int(self.xrt.get_X_button()),
            int(self.xrt.get_Y_button()),
            int(self.xrt.get_left_menu_button()),
            int(self.xrt.get_right_menu_button()),
            int(self.xrt.get_left_axis_click()),
            int(self.xrt.get_right_axis_click()),
        ]
        self.pub_controller_input.publish(joy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = XrBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
