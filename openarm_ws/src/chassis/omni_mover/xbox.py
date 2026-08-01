# import pygame
# import math
# import sys
# import time
# from motor_speed_controller import MotorSpeedController

# # 初始化模块
# pygame.init()
# pygame.joystick.init()

# # 全局变量
# joystick = None  # 手柄对象（初始为None）
# control_enabled = False  # 电机控制使能标志
# running = True

# # 初始化电机控制器
# motor_controller = MotorSpeedController(
#     serial_port='/dev/ttyACM0',
#     max_speed=20.0,
#     min_speed=-20.0,
#     enable_ids=[0x101, 0x102, 0x103],
#     speed_ids=[0x201, 0x202, 0x203]
# )

# def check_joystick_connection():
#     """检查并初始化手柄连接"""
#     global joystick
#     # 如果当前没有手柄，尝试获取新连接的手柄
#     if joystick is None:
#         joystick_count = pygame.joystick.get_count()
#         if joystick_count > 0:
#             joystick = pygame.joystick.Joystick(0)
#             joystick.init()
#             print(f"\n✅ 已连接游戏手柄: {joystick.get_name()}")
#             print("操作说明：A键使能电机，B键失能电机，左摇杆控制速度\n")
#             return True
#         else:
#             return False
#     return True

# def handle_joystick_disconnect():
#     """处理手柄断开连接"""
#     global joystick, control_enabled
#     if joystick is not None:
#         joystick.quit()
#         joystick = None
#         print("\n❌ 游戏手柄已断开连接")
#     # 手柄断开时自动失能电机
#     if control_enabled:
#         motor_controller.stop()
#         control_enabled = False
#         print("⚠️  电机已自动失能（因手柄断开）")

# # 初始提示
# print("等待游戏手柄连接...（可随时插拔重连）")
# check_joystick_connection()  # 首次检查

# clock = pygame.time.Clock()

# try:
#     while running:
#         # 事件处理
#         for event in pygame.event.get():
#             if event.type == pygame.QUIT:
#                 running = False
            
#             # 手柄连接事件
#             elif event.type == pygame.JOYDEVICEADDED:
#                 check_joystick_connection()
            
#             # 手柄断开事件
#             elif event.type == pygame.JOYDEVICEREMOVED:
#                 handle_joystick_disconnect()
            
#             # 手柄按键事件（仅当手柄连接时处理）
#             elif event.type == pygame.JOYBUTTONDOWN and joystick is not None:
#                 # A键（按钮0）：使能电机
#                 if event.button == 0:
#                     if not control_enabled:
#                         motor_controller.start()
#                         control_enabled = True
#                         print("\n✅ 电机已使能，进入控制模式")
#                     else:
#                         print("\n⚠️  电机已处于使能状态")
                
#                 # B键（按钮1）：失能电机
#                 elif event.button == 1:
#                     if control_enabled:
#                         motor_controller.stop()
#                         control_enabled = False
#                         print("\n✅ 电机已失能，退出控制模式")
#                     else:
#                         print("\n⚠️  电机已处于失能状态")

#         # 检查手柄连接状态（每秒更新一次提示）
#         if not check_joystick_connection():
#             print(f"\r⌛ 未检测到游戏手柄，等待连接...", end="")
#         else:
#             # 手柄连接时，处理电机控制
#             if control_enabled:
#                 axis0 = joystick.get_axis(0)
#                 axis1 = joystick.get_axis(1)
#                 axis3 = joystick.get_axis(3) * 1.5

#                 # 计算电机速度
#                 motor1_speed = 6.0 * (-(2.0/3.0)*math.sin(math.pi/3)*axis1 + (1.0/3.0)*axis0 + (1.0/3.0)*axis3)
#                 motor2_speed = 6.0 * (0*axis1 - (2.0/3.0)*axis0 + (2.0/3.0)*math.sin(math.pi/6)*axis3)
#                 motor3_speed = 6.0 * ((2.0/3.0)*math.sin(math.pi/3)*axis1 + (1.0/3.0)*axis0 + (1.0/3.0)*axis3)

#                 motor_controller.set_target_speeds(motor1_speed, motor2_speed, motor3_speed)
#                 print(f"\r电机1: {motor1_speed:.2f} | 电机2: {motor2_speed:.2f} | 电机3: {motor3_speed:.2f} | 状态: 已使能", end="")
#             else:
#                 print(f"\r控制状态：未使能（A键启动，B键停止）", end="")

#         clock.tick(50)  # 保持50Hz频率

# except KeyboardInterrupt:
#     print("\n\n程序已手动退出")
# finally:
#     # 清理资源
#     if control_enabled:
#         motor_controller.stop()
#     if joystick is not None:
#         joystick.quit()
#     pygame.joystick.quit()
#     pygame.quit()
#     print("资源已释放，程序结束")



import pygame
import math
import sys
import time
from motor_speed_controller import MotorSpeedController

# 初始化模块
pygame.init()
pygame.joystick.init()

# 全局变量
joystick = None  # 手柄对象（初始为None）
control_enabled = False  # 电机控制使能标志
running = True

# 速度模式配置：0-中速(6)，1-高速(12)，2-低速(4)
speed_mode = 0  # 初始为中速
speed_multipliers = [6.0, 12.0, 4.0]  # 对应模式的速度乘数
speed_mode_names = ["中速", "高速", "低速"]  # 模式名称（用于提示）

# 初始化电机控制器
motor_controller = MotorSpeedController(
    serial_port='/dev/ttyACM0',
    max_speed=20.0,
    min_speed=-20.0,
    enable_ids=[0x101, 0x102, 0x103],
    speed_ids=[0x201, 0x202, 0x203]
)

def check_joystick_connection():
    """检查并初始化手柄连接"""
    global joystick, speed_mode
    # 如果当前没有手柄，尝试获取新连接的手柄
    if joystick is None:
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            # 连接后重置为中速模式
            speed_mode = 0
            print(f"\n✅ 已连接游戏手柄: {joystick.get_name()}")
            print("操作说明：")
            print("  A键：使能电机，进入控制模式")
            print("  B键：失能电机，退出控制模式")
            print("  X键：切换速度模式（中速→高速→低速循环）")
            print("  左摇杆：控制电机速度\n")
            return True
        else:
            return False
    return True

def handle_joystick_disconnect():
    """处理手柄断开连接"""
    global joystick, control_enabled
    if joystick is not None:
        joystick.quit()
        joystick = None
        print("\n❌ 游戏手柄已断开连接")
    # 手柄断开时自动失能电机
    if control_enabled:
        motor_controller.stop()
        control_enabled = False
        print("⚠️  电机已自动失能（因手柄断开）")

def switch_speed_mode():
    """切换速度模式（循环切换）"""
    global speed_mode
    speed_mode = (speed_mode + 1) % 3  # 0→1→2→0循环
    print(f"\n⚡ 已切换到{speed_mode_names[speed_mode]}模式（乘数：{speed_multipliers[speed_mode]}）")

# 初始提示
print("等待游戏手柄连接...（可随时插拔重连）")
check_joystick_connection()  # 首次检查

clock = pygame.time.Clock()

try:
    while running:
        # 事件处理
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            # 手柄连接事件
            elif event.type == pygame.JOYDEVICEADDED:
                check_joystick_connection()
            
            # 手柄断开事件
            elif event.type == pygame.JOYDEVICEREMOVED:
                handle_joystick_disconnect()
            
            # 手柄按键事件（仅当手柄连接时处理）
            elif event.type == pygame.JOYBUTTONDOWN and joystick is not None:
                # A键（按钮0）：使能电机
                if event.button == 0:
                    if not control_enabled:
                        motor_controller.start()
                        control_enabled = True
                        print(f"\n✅ 电机已使能，当前速度模式：{speed_mode_names[speed_mode]}（乘数：{speed_multipliers[speed_mode]}）")
                    else:
                        print("\n⚠️  电机已处于使能状态")
                
                # B键（按钮1）：失能电机
                elif event.button == 1:
                    if control_enabled:
                        motor_controller.stop()
                        control_enabled = False
                        print("\n✅ 电机已失能，退出控制模式")
                    else:
                        print("\n⚠️  电机已处于失能状态")
                
                # X键（通常是按钮2）：切换速度模式
                elif event.button == 2:
                    if control_enabled:  # 仅在使能状态下允许切换
                        switch_speed_mode()
                    else:
                        print("\n⚠️  请先按A键使能电机，再切换速度模式")

        # 检查手柄连接状态（每秒更新一次提示）
        if not check_joystick_connection():
            print(f"\r⌛ 未检测到游戏手柄，等待连接...", end="")
        else:
            # 手柄连接时，处理电机控制
            if control_enabled:
                # 获取当前速度乘数
                current_multiplier = speed_multipliers[speed_mode]
                
                # 读取摇杆值
                axis0 = joystick.get_axis(0)
                axis1 = joystick.get_axis(1)
                axis3 = joystick.get_axis(3) * 1.5

                # 计算电机速度（使用当前模式的乘数）
                motor1_speed = current_multiplier * (-(2.0/3.0)*math.sin(math.pi/3)*axis1 + (1.0/3.0)*axis0 + (1.0/3.0)*axis3)
                motor2_speed = current_multiplier * (0*axis1 - (2.0/3.0)*axis0 + (2.0/3.0)*math.sin(math.pi/6)*axis3)
                motor3_speed = current_multiplier * ((2.0/3.0)*math.sin(math.pi/3)*axis1 + (1.0/3.0)*axis0 + (1.0/3.0)*axis3)

                motor_controller.set_target_speeds(motor1_speed, motor2_speed, motor3_speed)
                print(f"\r电机1: {motor1_speed:.2f} | 电机2: {motor2_speed:.2f} | 电机3: {motor3_speed:.2f} | 状态: 已使能 | 速度模式: {speed_mode_names[speed_mode]}", end="")
            else:
                print(f"\r控制状态：未使能（A键启动，B键停止，X键切换速度模式） | 当前速度模式: {speed_mode_names[speed_mode]}", end="")

        clock.tick(50)  # 保持50Hz频率

except KeyboardInterrupt:
    print("\n\n程序已手动退出")
finally:
    # 清理资源
    if control_enabled:
        motor_controller.stop()
    if joystick is not None:
        joystick.quit()
    pygame.joystick.quit()
    pygame.quit()
    print("资源已释放，程序结束")
