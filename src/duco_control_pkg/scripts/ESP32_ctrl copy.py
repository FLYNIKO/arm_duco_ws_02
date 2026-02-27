import socket
import signal
import struct
import sys
import time
import rospy
from key_input_pkg.msg import KeyInput
from std_msgs.msg import Float64MultiArray


# /keyinput 话题的数组第一个元素缓存（按位：第8位=急停，第25~28位=1~4）
_keyinput_value = [0]
_keyinput_prev = [0]
_spray_state_value = [0]
_spray_state_prev = [0]

ip = '192.168.0.210'
port = 45678
recv_timeout = 5.0

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
    def __init__(self, ip=ip, port=port, recv_timeout=recv_timeout):
        self.ip = ip
        self.port = port
        self.sock = None
        self.recv_timeout = recv_timeout  # 接收响应超时（秒），应对网络/ESP32 延迟
        self.last_key_time = time.time()
        self.last_spray_state_time = time.time()
        self.connect_failed = False
        rospy.init_node("esp32_ctrl", anonymous=True)
        rospy.Subscriber("/key_input", KeyInput, self._keyinput_cb, queue_size=1)
        rospy.Subscriber('/Duco_state', Float64MultiArray, self._spray_state_cb, queue_size=1)
        self.rate = rospy.Rate(20)  # 20Hz 轮询按键
    
    def connect(self):
        """连接ESP32"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip, self.port))
        self.sock.settimeout(self.recv_timeout)
        rospy.loginfo(f"| ESP32 |：已连接到 {self.ip}:{self.port}")
    
    def connect_with_retry(self, gpio_states, retry_interval=5):
        """
        尝试连接ESP32；若失败则每隔 retry_interval 秒重试，直到连接成功或 ROS 关闭。
        返回 True 表示连接成功，False 表示在 ROS 已关闭的情况下退出重试。
        """
        while not rospy.is_shutdown():
            try:
                self.connect()
                self.connect_failed = False
                esp32.control_gpio(gpio_states)
                return True
            except (socket.error, OSError) as e:
                self.connect_failed = True
                rospy.logwarn(f"| ESP32 |：连接到 {self.ip}:{self.port} 失败：{e}，{retry_interval} 秒后重试...")
                time.sleep(retry_interval)
        rospy.loginfo("| ESP32 |：ROS 已关闭，停止连接重试。")
        return False
    
    def disconnect(self):
        """断开连接"""
        if self.sock:
            self.sock.close()
            rospy.loginfo("| ESP32 |：已断开连接")
    
    def send_command(self, data):
        """发送带CRC校验的命令"""
        # 计算CRC并添加到数据末尾
        crc = calculate_crc8(data)
        command = data + bytes([crc])
        self.sock.send(command)
        rospy.loginfo(f"| ESP32 |：发送: {list(command)}")
    
    def receive_response(self, expected_len):
        """
        接收并验证响应。因网络/ESP32 延迟，响应可能晚到或分多次到达，故循环收满 expected_len 字节。
        超时由 self.recv_timeout 控制，超时或 CRC 失败时返回 None。
        """
        if not self.sock:
            return None
        buf = b''
        try:
            while len(buf) < expected_len:
                need = expected_len - len(buf)
                chunk = self.sock.recv(need)
                if not chunk:
                    rospy.logwarn("| ESP32 |：接收: 连接已关闭，未收满数据，尝试重新连接...")
                    # 连接被对端关闭，尝试重连，当前命令结果置为 None 交给上层逻辑决定是否重发
                    try:
                        self.disconnect()
                    except Exception:
                        pass
                    self.connect_with_retry(gpio_states=gpio_states, retry_interval=5)
                    return None
                buf += chunk
            rospy.loginfo(f"| ESP32 |：接收: {list(buf)}")
        except socket.timeout:
            rospy.logwarn(f"| ESP32 |：接收: 超时（{self.recv_timeout}s），仅收到 {len(buf)}/{expected_len} 字节，尝试重新连接...")
            # 超时未收到完整数据，认为连接异常，先断开再重连
            try:
                self.disconnect()
            except Exception:
                pass
            self.connect_with_retry(gpio_states=gpio_states, retry_interval=5)
            return None

        if not verify_crc8(buf):
            rospy.logwarn("| ESP32 |：警告：CRC校验失败！")
            return None
        rospy.loginfo("| ESP32 |：CRC校验通过")
        return buf[:-1]  # 返回去除CRC的数据
    
    def emergency_stop(self):
        """急停"""
        rospy.loginfo("| ESP32 |：发送急停命令")
        self.send_command(bytes([0x00]))
        response = self.receive_response(3)
        if response and response[1] == 0x01:
            rospy.loginfo("| ESP32 |：急停成功！")
        return response
    
    def control_gpio(self, pin_states):
        """
        控制GPIO
        pin_states: [state0, state1, state2, state3]
        state: 0=低, 1=高, 0xFF=不变
        """
        rospy.loginfo(f"| ESP32 |：控制GPIO: {pin_states}")
        command = bytes([0x01] + pin_states)
        self.send_command(command)
        time.sleep(0.5)
        response = self.receive_response(5)
        if response:
            rospy.loginfo(f"| ESP32 |：GPIO状态: {list(response)}")
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
        rospy.loginfo(f"| ESP32 |：控制电机{motor_id}")
        rospy.loginfo(f"| ESP32 |：方向: {'反转' if direction else '正转'}")
        rospy.loginfo(f"| ESP32 |：脉冲宽度: {pulse_width}μs")
        rospy.loginfo(f"| ESP32 |：脉冲间隔: {pulse_interval}μs")
        rospy.loginfo(f"| ESP32 |：步数: {steps if steps > 0 else '持续运行'}")
        
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
            rospy.loginfo(f"| ESP32 |：电机{motor_id}启动成功！")
        return response
    
    def stop_motor(self, motor_id):
        """停止指定电机"""
        rospy.loginfo(f"| ESP32 |：停止电机{motor_id}")
        command = bytes([0x03, 0x01, motor_id])
        self.send_command(command)
        response = self.receive_response(5)
        if response and response[3] == 0x01:
            rospy.loginfo(f"| ESP32 |：电机{motor_id}已停止")
        return response
    
    def query_motor(self, motor_id):
        """查询指定电机状态"""
        rospy.loginfo(f"| ESP32 |：查询电机{motor_id}状态")
        command = bytes([0x03, 0x02, motor_id])
        self.send_command(command)
        response = self.receive_response(8)
        if response:
            running = response[3]
            steps_left = (response[4] << 16) | (response[5] << 8) | response[6]
            rospy.loginfo(f"| ESP32 |：运行状态: {'运行中' if running else '已停止'}")
            rospy.loginfo(f"| ESP32 |：剩余步数: {steps_left}")
        return response
    
    def query_all_motors(self):
        """查询所有电机状态"""
        rospy.loginfo(f"| ESP32 |：查询所有电机状态")
        command = bytes([0x03, 0x03])
        self.send_command(command)
        # 响应长度 = 2字节头 + 电机数量 + 1字节CRC
        response = self.receive_response(6)  # 假设3个电机
        if response:
            motor_count = len(response) - 2
            for i in range(motor_count):
                status = response[2 + i]
                rospy.loginfo(f"| ESP32 |：电机{i}: {'运行中' if status else '已停止'}")
        return response

    def switch_status(self):
        """
        查询4路开关GPIO状态（对应 blue_ctrl.ino 中 0x03 / 0x04 命令）
        返回值（去CRC后）格式：
        [0] = 0x03  主命令
        [1] = 0x04  子命令（GPIO 查询）
        [2..5] = 4 路 GPIO 状态，0x00/0x01
        """
        rospy.loginfo(f"| ESP32 |：查询开关GPIO状态")
        # 发送扩展命令 0x03, 子命令 0x04
        command = bytes([0x03, 0x04])
        self.send_command(command)
        # 期望收到 7 字节（含 CRC），其中前 6 字节为数据
        response = self.receive_response(7)
        if not response:
            return None

        if len(response) < 6 or response[0] != 0x03 or response[1] != 0x04:
            rospy.logwarn("| ESP32 |：返回数据格式错误")
            return None

        gpio_states = list(response[2:6])
        rospy.loginfo(f"| ESP32 |：开关GPIO状态: {gpio_states}")
        return gpio_states

    def query_all_gpio_status(self):
        """查询所有GPIO状态"""
        rospy.loginfo(f"| ESP32 |：查询所有GPIO状态")
        command = bytes([0x03, 0x05])
        self.send_command(command)
        response = self.receive_response(7)
        if not response:
            return None

        if len(response) < 6 or response[0] != 0x03 or response[1] != 0x05:
            rospy.logwarn("| ESP32 |：返回数据格式错误")
            return None

        gpio_states = list(response[2:6])
        rospy.loginfo(f"| ESP32 |：所有GPIO状态: {gpio_states}")
        return gpio_states

    def _spray_state_cb(self, msg):
        """/Duco_state 话题回调：取数组第一个元素"""
        if len(msg.data) > 0:
            if msg.data[15] < 5:
                _spray_state_value[0] = 0
            else:
                _spray_state_value[0] = 1
        self.last_spray_state_time = time.time()
    
    def get_spray_command(self):
            current = _spray_state_value[0]
            prev = _spray_state_prev[0]
            
            res = ""
            if current != prev:
                res = "autosprayON" if current == 1 else "autosprayOFF"
            
            _spray_state_prev[0] = current
            return res

    def _keyinput_cb(self, msg):
        """/keyinput 话题回调：取数组第一个元素"""
        if len(msg.keys) > 0:
            _keyinput_value[0] = msg.keys[0]
        self.last_key_time = time.time()

    def get_key_command(self):
        """
        从 rostopic /keyinput 数组第一个元素的指定位解析命令（上升沿触发）：
        第 8 位 -> 急停(e)，第 25~28 位 -> 1~4
        """
        if time.time() - self.last_key_time > 2:
            esp32.emergency_stop()
            return ""

        current = _keyinput_value[0]
        prev = _keyinput_prev[0]

        # 上升沿：当前为 1、上一次为 0 才返回对应命令
        if (current & (1 << 8)) and not (prev & (1 << 8)):
            _keyinput_prev[0] = current
            return "e"
        if (current & (1 << 25)) and not (prev & (1 << 25)):
            _keyinput_prev[0] = current
            return "1"
        if (current & (1 << 26)) and not (prev & (1 << 26)):
            _keyinput_prev[0] = current
            return "2"
        if (current & (1 << 27)) and not (prev & (1 << 27)):
            _keyinput_prev[0] = current
            return "3"
        if (current & (1 << 28)) and not (prev & (1 << 28)):
            _keyinput_prev[0] = current
            return "4"

        _keyinput_prev[0] = current
        return ""

def _handle_sigint(signum, frame):
    """确保 Ctrl+C 能退出（覆盖 rospy 对 SIGINT 的接管）"""
    rospy.loginfo("| ESP32 |：收到 Ctrl+C，正在退出...")
    try:
        rospy.signal_shutdown("Ctrl+C")
    except Exception:
        pass
    sys.exit(0)

if __name__ == "__main__":
    esp32 = ESP32Controller(ip, port, recv_timeout)
    signal.signal(signal.SIGINT, _handle_sigint)

    try:
        gpio_states = [0, 1, 0, 0]
        # 启动时带重试的连接逻辑：连接失败则每 10 秒重试一次
        connected = esp32.connect_with_retry(gpio_states=gpio_states, retry_interval=5)
        if not connected:
            # 若在重试过程中 ROS 被关闭，则直接结束主程序
            sys.exit(0)

        while True:
            cmd = esp32.get_key_command()
            spray_cmd = esp32.get_spray_command()

            if cmd == "" and spray_cmd == "":
                rospy.sleep(0.05)
                continue

            # 退出（无 ROS 时用键盘 q；有 ROS 时可用其他方式）
            if cmd == "q":
                rospy.loginfo("| ESP32 |：退出程序。")
                break

            # 急停
            elif cmd == "e":
                esp32.emergency_stop()
                gpio_states = [0, 1, 0, 0]

            # 查询 GPIO 开关状态
            elif cmd == "s":
                # states = esp32.switch_status()
                states = esp32.query_all_gpio_status()
                if states is not None:
                    rospy.loginfo(f"| ESP32 |：当前 GPIO 状态: {states}")

            # 切换某一路 GPIO
            elif cmd in ["1", "2", "3", "4"]:
                index = int(cmd) - 1
                # 本地状态取反
                gpio_states[index] = 0 if gpio_states[index] == 1 else 1
                rospy.loginfo(f"| ESP32 |：切换 GPIO{index} 为: {gpio_states[index]}")
                esp32.control_gpio(gpio_states)
            
            elif spray_cmd == "autosprayON":
                rospy.loginfo("| ESP32 |：自动开喷")
                gpio_states = [0, 1, 0, 1]
                esp32.control_gpio(gpio_states)
                rospy.sleep(4)
                gpio_states = [1, 1, 0, 1]
                esp32.control_gpio(gpio_states)

            elif spray_cmd == "autosprayOFF":
                rospy.loginfo("| ESP32 |：自动停喷")
                gpio_states = [0, 1, 0, 1]
                esp32.control_gpio(gpio_states)
                rospy.sleep(3)
                gpio_states = [0, 1, 0, 0]
                esp32.control_gpio(gpio_states)

            else:
                rospy.logwarn("无效命令，请重新输入。")

            rospy.sleep(0.05)

    except KeyboardInterrupt:
        rospy.loginfo("| ESP32 |：收到 Ctrl+C，正在退出...")
        rospy.signal_shutdown("Ctrl+C")
    except Exception as e:
        rospy.logwarn(f"| ESP32 |：错误: {e}")
        import traceback
        rospy.logwarn(f"| ESP32 |：错误: {traceback.format_exc()}")

    finally:
        esp32.disconnect()
