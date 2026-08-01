#!/usr/bin/env python3
"""
全向移动底盘 ROS2 控制节点
订阅 /exo/gamepad_keys 话题获取摇杆数据，控制底盘电机

话题订阅:
- /exo/gamepad_keys (sensor_msgs/Joy): 摇杆数据
  * axes[0]: 左摇杆X (左右平移)
  * axes[1]: 左摇杆Y (前后移动)
  * axes[2]: 右摇杆X (旋转)
  * buttons[1]: 摇杆控制状态 (1=启用, 0=禁用)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from motor_speed_controller import MotorSpeedController


class OmniMoverNode(Node):
    """全向移动底盘 ROS2 节点"""
    
    def __init__(self):
        super().__init__('omni_mover_node')
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_speed', 20.0)
        self.declare_parameter('min_speed', -20.0)
        self.declare_parameter('speed_multiplier', 6.0)
        self.declare_parameter('rotation_scale', 1.5)
        self.declare_parameter('gamepad_topic', '/exo/gamepad_keys')
        
        # 获取参数
        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        max_speed = self.get_parameter('max_speed').value
        min_speed = self.get_parameter('min_speed').value
        self.speed_multiplier = self.get_parameter('speed_multiplier').value
        self.rotation_scale = self.get_parameter('rotation_scale').value
        gamepad_topic = self.get_parameter('gamepad_topic').value
        
        # 控制状态
        self.control_enabled = False
        self.motor_started = False
        
        # 初始化电机控制器
        try:
            self.motor_controller = MotorSpeedController(
                serial_port=serial_port,
                baudrate=baudrate,
                max_speed=max_speed,
                min_speed=min_speed,
                enable_ids=[0x101, 0x102, 0x103],
                speed_ids=[0x201, 0x202, 0x203]
            )
            self.get_logger().info(f'电机控制器初始化成功: {serial_port}')
        except Exception as e:
            self.get_logger().error(f'电机控制器初始化失败: {e}')
            self.motor_controller = None
        
        # 订阅摇杆话题
        self.joy_sub = self.create_subscription(
            Joy,
            gamepad_topic,
            self.joy_callback,
            10
        )
        
        self.get_logger().info(
            f'全向移动底盘节点已启动'
            f'\n  订阅话题: {gamepad_topic}'
            f'\n  串口: {serial_port}'
            f'\n  速度乘数: {self.speed_multiplier}'
            f'\n  旋转缩放: {self.rotation_scale}'
        )
    
    def joy_callback(self, msg: Joy):
        """处理摇杆数据回调"""
        if self.motor_controller is None:
            return
        
        # 检查控制状态 (buttons[1] = 摇杆控制状态)
        if len(msg.buttons) > 1:
            new_control_state = msg.buttons[1] == 1
            
            # 状态变化检测
            if new_control_state != self.control_enabled:
                self.control_enabled = new_control_state
                if self.control_enabled:
                    if not self.motor_started:
                        self.motor_controller.start()
                        self.motor_started = True
                    self.get_logger().info('✅ 底盘控制已启用')
                else:
                    self.motor_controller.set_target_speeds(0.0, 0.0, 0.0)
                    self.get_logger().info('⏸️ 底盘控制已暂停')
        
        # 如果控制未启用，不发送速度命令
        if not self.control_enabled:
            return
        
        # 获取摇杆数据
        if len(msg.axes) >= 3:
            axis_x = msg.axes[0]      # 左摇杆X: 左右平移
            axis_y = msg.axes[1]      # 左摇杆Y: 前后移动
            axis_rot = msg.axes[2] * self.rotation_scale  # 右摇杆X: 旋转
            
            # 计算三轮全向底盘电机速度
            motor1_speed = self.speed_multiplier * (
                -(2.0/3.0) * math.sin(math.pi/3) * axis_y + 
                (1.0/3.0) * axis_x + 
                (1.0/3.0) * axis_rot
            )
            motor2_speed = self.speed_multiplier * (
                0 * axis_y - 
                (2.0/3.0) * axis_x + 
                (2.0/3.0) * math.sin(math.pi/6) * axis_rot
            )
            motor3_speed = self.speed_multiplier * (
                (2.0/3.0) * math.sin(math.pi/3) * axis_y + 
                (1.0/3.0) * axis_x + 
                (1.0/3.0) * axis_rot
            )
            
            # 发送速度到电机
            self.motor_controller.set_target_speeds(motor1_speed, motor2_speed, motor3_speed)
    
    def destroy_node(self):
        """节点销毁时清理资源"""
        if self.motor_controller is not None and self.motor_started:
            self.motor_controller.stop()
            self.get_logger().info('电机已停止')
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = OmniMoverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

