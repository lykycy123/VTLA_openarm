import serial
import numpy as np
import threading
import time
import struct
from typing import List


class MotorSpeedController:
    def __init__(self, 
                 serial_port: str = '/dev/ttyACM0', 
                 baudrate: int = 115200,
                 max_speed: float = 20.0, 
                 min_speed: float = -20.0,
                 enable_ids: List[int] = [0x101, 0x102, 0x103],
                 speed_ids: List[int] = [0x201, 0x202, 0x203]):
        """
        电机速度控制器初始化
        
        :param serial_port: 串口设备路径
        :param baudrate: 波特率
        :param max_speed: 最大速度限制（默认20）
        :param min_speed: 最小速度限制（默认-20）
        :param enable_ids: 使能/失能电机地址列表
        :param speed_ids: 速度控制地址列表（需与电机一一对应）
        """
        # 串口配置
        self.serial_ = serial.Serial(
            port=serial_port,
            baudrate=baudrate,
            timeout=0.1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False
        )
        
        # 电机地址配置
        self.enable_ids = enable_ids
        self.speed_ids = speed_ids
        if len(self.enable_ids) != 3 or len(self.speed_ids) != 3:
            raise ValueError("电机地址列表必须包含3个地址")
        
        # 速度限制
        self.max_speed = max_speed
        self.min_speed = min_speed
        
        # 发送帧格式（固定头部）
        self.send_data_frame = np.array(
            [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 
             0, 0, 0, 0, 0x00, 0x08, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)
        
        # 实时速度缓存（带线程锁）
        self._target_speeds = [0.0, 0.0, 0.0]
        self._speed_lock = threading.Lock()
        
        # 控制线程与状态
        self._control_thread = None
        self._is_running = False

    def _limit_speed(self, speed: float) -> float:
        """限制速度在配置范围内"""
        return max(min(speed, self.max_speed), self.min_speed)
    
    def _float_to_bytes(self, speed: float) -> bytes:
        """将浮点数转换为4字节小端序"""
        limited_speed = self._limit_speed(speed)
        return struct.pack('<f', limited_speed)
    
    def _send_speed(self, target_id: int, speed: float) -> bool:
        """向单个电机发送速度指令"""
        try:
            speed_bytes = self._float_to_bytes(speed)
            # 构造8字节数据（前4字节速度，后4字节补0）
            data_buf = np.array(list(speed_bytes) + [0x00]*4, dtype=np.uint8)
            
            # 设置目标地址
            self.send_data_frame[13] = target_id & 0xff
            self.send_data_frame[14] = (target_id >> 8) & 0xff
            self.send_data_frame[21:29] = data_buf
            
            if not self.serial_.is_open:
                self.serial_.open()
            
            self.serial_.write(bytes(self.send_data_frame.T))
            return True
        except Exception as e:
            print(f"发送速度到0x{target_id:03X}失败: {e}")
            return False
    
    def _control_loop(self):
        """100Hz速度控制循环（内部线程）"""
        interval = 0.01  # 100Hz间隔
        while self._is_running:
            start_time = time.perf_counter()
            
            # 线程安全获取速度
            with self._speed_lock:
                speeds = self._target_speeds.copy()
            
            # 向三个电机发送速度
            for i in range(3):
                self._send_speed(self.speed_ids[i], speeds[i])
            
            # 补偿耗时，保证频率
            elapsed = time.perf_counter() - start_time
            sleep_time = max(0, interval - elapsed)
            time.sleep(sleep_time)
    
    def set_target_speeds(self, v1: float, v2: float, v3: float):
        """
        设置三个电机的目标速度（实时生效）
        
        :param v1: 第一个电机速度（对应speed_ids[0]）
        :param v2: 第二个电机速度（对应speed_ids[1]）
        :param v3: 第三个电机速度（对应speed_ids[2]）
        """
        with self._speed_lock:
            self._target_speeds = [v1, v2, v3]
    
    def start(self):
        """启动速度控制（使能电机并开始发送指令）"""
        if self._is_running:
            print("控制器已在运行")
            return
        
        # 使能所有电机
        print("使能所有电机...")
        for idx in self.enable_ids:
            self._send_command(idx, 0xFC)
            time.sleep(0.05)
        
        # 启动控制线程
        self._is_running = True
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()
        print("速度控制已启动（100Hz）")
    
    def stop(self):
        """停止速度控制（失能电机并释放资源）"""
        if not self._is_running:
            print("控制器未在运行")
            return
        
        # 停止线程
        self._is_running = False
        if self._control_thread:
            self._control_thread.join()
        
        # 失能所有电机
        print("失能所有电机...")
        for idx in self.enable_ids:
            self._send_command(idx, 0xFD)
            time.sleep(0.05)
        
        # 关闭串口
        if self.serial_.is_open:
            self.serial_.close()
        print("速度控制已停止")
    
    def _send_command(self, target_id: int, cmd: int):
        """发送使能/失能命令（内部使用）"""
        try:
            data_buf = np.array([0xFF]*7 + [cmd], np.uint8)
            self.send_data_frame[13] = target_id & 0xff
            self.send_data_frame[14] = (target_id >> 8) & 0xff
            self.send_data_frame[21:29] = data_buf
            
            if not self.serial_.is_open:
                self.serial_.open()
            
            self.serial_.write(bytes(self.send_data_frame.T))
        except Exception as e:
            print(f"发送命令到0x{target_id:03X}失败: {e}")


# 示例使用代码
if __name__ == "__main__":
    try:
        # 初始化控制器（可自定义速度限制）
        controller = MotorSpeedController(
            serial_port='/dev/ttyACM0',
            max_speed=20.0,
            min_speed=-20.0
        )
        
        # 启动控制
        controller.start()
        
        # 示例：设置速度（可在外部实时调用）
        controller.set_target_speeds(6.0, 6.0, 6.0)  # 三个电机均设为6
        
        # 运行5秒
        time.sleep(5)
        
        # 示例：中途修改速度
        controller.set_target_speeds(3.0, 3.0, 3.0)
        time.sleep(2)
        
    except KeyboardInterrupt:
        print("用户中断")
    finally:
        # 停止控制（确保失能）
        controller.stop()
