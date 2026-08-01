import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


class BaseController(Node):
    """Convert XR joystick input to ROS 2 cmd_vel messages."""

    def __init__(self) -> None:
        super().__init__("base_controller")

        self.declare_parameter("linear_speed", 0.1)
        self.declare_parameter("angular_speed", 0.2)
        self.declare_parameter("deadzone", 0.1)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.deadzone = float(self.get_parameter("deadzone").value)

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joy_subscription = self.create_subscription(
            Joy, "/vr/controller_input", self.publish_cmd_vel, 10
        )
        self.last_command: tuple[float, float] | None = None

        self.get_logger().info(
            "Listening on /vr/controller_input: left Y controls "
            "forward/backward, right X controls rotation."
        )

    def apply_deadzone(self, value: float) -> float:
        """Remove small joystick drift around its center position."""
        return 0.0 if abs(value) < self.deadzone else value

    def publish_cmd_vel(self, msg: Joy) -> None:
        if len(msg.axes) < 3:
            self.get_logger().warning(
                "Ignoring /vr/controller_input with fewer than 3 axes.",
                throttle_duration_sec=5.0,
            )
            return

        linear_x = self.apply_deadzone(float(msg.axes[1])) * self.linear_speed

        # ROS positive angular.z turns left, while positive joystick X points
        # right, so the joystick value is inverted.
        angular_z = -self.apply_deadzone(float(msg.axes[2])) * self.angular_speed

        command = (round(linear_x, 3), round(angular_z, 3))
        # 摇杆未变时不重复发 cmd_vel，减少底盘驱动与 DDS 负担
        if command == self.last_command:
            return

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(
            f"cmd_vel linear.x={command[0]:.3f}, angular.z={command[1]:.3f}"
        )
        self.last_command = command

    def stop(self) -> None:
        """Publish a zero velocity command before shutting down."""
        self.cmd_vel_publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseController()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
