import socket
import struct
import time

# CRC8查找表（与ESP32端一致）
CRC8_TABLE = [
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
]

def calculate_crc8(data):
    """计算CRC8校验值"""
    crc = 0x00
    for byte in data:
        crc = CRC8_TABLE[crc ^ byte]
    return crc

def verify_crc8(data):
    """验证接收数据的CRC8"""
    if len(data) < 2:
        return False
    received_crc = data[-1]
    calculated_crc = calculate_crc8(data[:-1])
    return received_crc == calculated_crc

class ESP32Controller:
    def __init__(self, ip='192.168.0.17', port=8080):
        self.ip = ip
        self.port = port
        self.sock = None
    
    def connect(self):
        """连接ESP32"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip, self.port))
        print(f"已连接到 {self.ip}:{self.port}")
    
    def disconnect(self):
        """断开连接"""
        if self.sock:
            self.sock.close()
            print("已断开连接")
    
    def send_command(self, data):
        """发送带CRC校验的命令"""
        # 计算CRC并添加到数据末尾
        crc = calculate_crc8(data)
        command = data + bytes([crc])
        self.sock.send(command)
        print(f"发送: {list(command)}")
    
    def receive_response(self, expected_len):
        """接收并验证响应"""
        response = self.sock.recv(expected_len)
        print(f"接收: {list(response)}")
        
        if not verify_crc8(response):
            print("警告：CRC校验失败！")
            return None
        
        print("CRC校验通过")
        return response[:-1]  # 返回去除CRC的数据
    
    def emergency_stop(self):
        """急停"""
        print("\n=== 发送急停命令 ===")
        self.send_command(bytes([0x00]))
        response = self.receive_response(3)
        if response and response[1] == 0x01:
            print("急停成功！")
        return response
    
    def control_gpio(self, pin_states):
        """
        控制GPIO
        pin_states: [state0, state1, state2, state3]
        state: 0=低, 1=高, 0xFF=不变
        """
        print(f"\n=== 控制GPIO: {pin_states} ===")
        command = bytes([0x01] + pin_states)
        self.send_command(command)
        time.sleep(0.5)
        response = self.receive_response(5)
        if response:
            print(f"GPIO状态: {list(response)}")
        return response
    
    def control_motor(self, motor_id, direction, pulse_width, pulse_interval, steps=0):
        """
        控制步进电机
        motor_id: 电机编号 (0, 1, 2...)
        direction: 0=正转, 1=反转
        pulse_width: 脉冲宽度（微秒）
        pulse_interval: 脉冲间隔（微秒）
        steps: 步数（0=持续运行）
        """
        print(f"\n=== 控制电机{motor_id} ===")
        print(f"方向: {'反转' if direction else '正转'}")
        print(f"脉冲宽度: {pulse_width}μs")
        print(f"脉冲间隔: {pulse_interval}μs")
        print(f"步数: {steps if steps > 0 else '持续运行'}")
        
        command = bytes([
            0x02,
            motor_id,
            direction,
            (pulse_width >> 8) & 0xFF,
            pulse_width & 0xFF,
            (pulse_interval >> 8) & 0xFF,
            pulse_interval & 0xFF,
            (steps >> 24) & 0xFF,
            (steps >> 16) & 0xFF,
            (steps >> 8) & 0xFF,
            steps & 0xFF
        ])
        self.send_command(command)
        time.sleep(0.5)
        response = self.receive_response(4)
        if response and response[2] == 0x01:
            print(f"电机{motor_id}启动成功！")
        return response
    
    def stop_motor(self, motor_id):
        """停止指定电机"""
        print(f"\n=== 停止电机{motor_id} ===")
        command = bytes([0x03, 0x01, motor_id])
        self.send_command(command)
        response = self.receive_response(5)
        if response and response[3] == 0x01:
            print(f"电机{motor_id}已停止")
        return response
    
    def query_motor(self, motor_id):
        """查询指定电机状态"""
        print(f"\n=== 查询电机{motor_id}状态 ===")
        command = bytes([0x03, 0x02, motor_id])
        self.send_command(command)
        response = self.receive_response(8)
        if response:
            running = response[3]
            steps_left = (response[4] << 16) | (response[5] << 8) | response[6]
            print(f"运行状态: {'运行中' if running else '已停止'}")
            print(f"剩余步数: {steps_left}")
        return response
    
    def query_all_motors(self):
        """查询所有电机状态"""
        print("\n=== 查询所有电机状态 ===")
        command = bytes([0x03, 0x03])
        self.send_command(command)
        # 响应长度 = 2字节头 + 电机数量 + 1字节CRC
        response = self.receive_response(6)  # 假设3个电机
        if response:
            motor_count = len(response) - 2
            for i in range(motor_count):
                status = response[2 + i]
                print(f"电机{i}: {'运行中' if status else '已停止'}")
        return response


# 使用示例
if __name__ == "__main__":
    # 连接ESP32
    esp32 = ESP32Controller("192.168.100.159", 45678)
    
    try:
        esp32.connect()
        
        # 示例1: 控制GPIO
        print("\n" + "="*50)
        print("示例1: 控制GPIO")
        print("="*50)
        esp32.control_gpio([1, 0, 0, 0])  # GPIO: 高,低,高,低
        time.sleep(2)
        esp32.control_gpio([0, 1, 0, 0])  # GPIO: 高,低,高,低
        time.sleep(2)
        esp32.control_gpio([0, 0, 1, 0])  # GPIO: 高,低,高,低
        time.sleep(2)
        esp32.control_gpio([0, 0, 0, 1])  # GPIO: 高,低,高,低
        time.sleep(2)
        esp32.control_gpio([0, 0, 0, 0])  # GPIO: 高,低,高,低
        time.sleep(2)
        esp32.control_gpio([1, 1, 1, 1])  # GPIO: 高,低,高,低
        time.sleep(2)
        esp32.control_gpio([0, 0, 0, 0])  # GPIO: 高,低,高,低
        # 示例2: 控制电机0运行1000步
        '''
        print("\n" + "="*50)
        print("示例2: 控制电机0")
        print("="*50)
        esp32.control_motor(
            motor_id=0,
            direction=0,         # 正转
            pulse_width=100,       # 5微秒
            pulse_interval=100,  # 500微秒
            steps=6400          # 1000步
        )
        '''
        # 等待一段时间
        time.sleep(2)

        
        # 示例3: 查询电机0状态
        '''
        print("\n" + "="*50)
        print("示例3: 查询电机状态")
        print("="*50)
        esp32.query_motor(0)
        '''
        # # 示例4: 同时控制多个电机
        # print("\n" + "="*50)
        # print("示例4: 同时控制多个电机")
        # print("="*50)
        # esp32.control_motor(0, 0, 5, 500, 2000)   # 电机0
        # esp32.control_motor(1, 1, 10, 300, 1500)  # 电机1
        # esp32.control_motor(2, 0, 8, 400, 1000)   # 电机2
        
        time.sleep(1)
        '''
        # 示例5: 查询所有电机状态
        print("\n" + "="*50)
        print("示例5: 查询所有电机")
        print("="*50)
        esp32.query_all_motors()
        
        # 示例6: 停止单个电机
        print("\n" + "="*50)
        print("示例6: 停止电机1")
        print("="*50)
        esp32.stop_motor(0)
        '''
        # 示例7: 急停（可选，谨慎使用）
        # print("\n" + "="*50)
        # print("示例7: 急停所有设备")
        # print("="*50)
        # esp32.emergency_stop()
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        esp32.disconnect()


# 简化的快速调用函数
def quick_control(ip, motor_id, steps, speed=500):
    """
    快速控制函数
    ip: ESP32的IP地址
    motor_id: 电机编号
    steps: 步数（正数正转，负数反转）
    speed: 速度（脉冲间隔微秒，越小越快）
    """
    esp = ESP32Controller(ip)
    try:
        esp.connect()
        direction = 1 if steps < 0 else 0
        steps = abs(steps)
        esp.control_motor(motor_id, direction, 5, speed, steps)
    finally:
        esp.disconnect()


# 快速使用示例
# quick_control("192.168.1.100", motor_id=0, steps=1000, speed=500)  # 电机0正转1000步
# quick_control("192.168.1.100", motor_id=1, steps=-500, speed=300)  # 电机1反转500步