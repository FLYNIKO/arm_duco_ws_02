#!/usr/bin/env python3
import rospy
from key_input_pkg.msg import KeyInput
import sys
import select
import termios
import tty
import threading
import time

# 按键到位位置的映射 (根据接收端ManualControl.py的位解析规则)
key_mapping = {
    # 基础移动控制 (位0-5)
    'q': 0,   # x0 - X轴负方向
    'a': 1,   # x1 - X轴正方向
    'w': 2,   # y0 - Y轴负方向  
    's': 3,   # y1 - Y轴正方向
    'e': 4,   # z0 - Z轴负方向
    'd': 5,   # z1 - Z轴正方向
    
    # 功能控制 (位6-9)
    'r': 6,   # init - 初始化
    'f': 7,   # serv - 服务
    't': 8,   # multi - 多功能
    'g': 9,   # start - 开始
    
    # 旋转控制 (位10-15)
    'z': 10,  # rx0 - X轴旋转负方向
    'x': 11,  # rx1 - X轴旋转正方向
    'c': 12,  # ry0 - Y轴旋转负方向
    'v': 13,  # ry1 - Y轴旋转正方向
    'b': 14,  # rz0 - Z轴旋转负方向
    'n': 15,  # rz1 - Z轴旋转正方向
    
    # 扩展功能 (位16-22)
    'm': 16,  # clog
    'j': 17,  # find
    'k': 18,  # high
    'l': 19,  # center
    'u': 20,  # low
    'i': 21,  # top
    'o': 22,  # bottom
}

class KeyInputPublisher:
    def __init__(self):
        # key_input数组：[按键位状态, 数据1, 数据2, 数据3, ...]
        # 可以在这里修改要发送的固定数据
        self.fixed_data = [0, 0, 0, 0, 0, 0, 50, 50, 550, 0]  # 修改这里的数据！！！
        # self.fixed_data = [0, 0, 0, 0, 0, 0, 50, 50, 550, 1]  # 修改这里的数据！！！


        self.keys_state = [0] + self.fixed_data  # 第一个元素是按键位，后面是固定数据
        self.pressed_keys = set()  # 记录当前按下的按键
        self.lock = threading.Lock()
        self.running = True
        
        # 保存原始终端设置
        self.old_settings = None
        
    def setup_terminal(self):
        """设置终端为raw模式以捕获按键"""
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            print("✓ 终端设置为raw模式成功")
            return True
        except Exception as e:
            print(f"✗ 设置终端模式失败: {e}")
            return False
            
    def restore_terminal(self):
        """恢复终端设置"""
        if self.old_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                print("\n✓ 终端设置已恢复")
            except Exception as e:
                print(f"\n✗ 恢复终端设置失败: {e}")
    
    def get_char(self):
        """非阻塞获取字符"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None
    
    def process_key(self, key):
        """处理按键"""
        if key is None:
            return
            
        key = key.lower()
        
        # ESC键退出 (ASCII 27)
        if ord(key) == 27:
            print("\n检测到ESC键，退出...")
            self.running = False
            return
            
        # Ctrl+C退出 (ASCII 3)
        if ord(key) == 3:
            print("\n检测到Ctrl+C，退出...")
            self.running = False
            return
        
        with self.lock:
            if key in key_mapping:
                bit_position = key_mapping[key]
                if key not in self.pressed_keys:
                    # 按键刚按下
                    self.pressed_keys.add(key)
                    self.keys_state[0] |= (1 << bit_position)
                    # print(f"按下键: {key} -> 设置位 {bit_position}, 状态: {self.keys_state[0]:023b} (十进制: {self.keys_state[0]})")
            elif ord(key) >= 32 and ord(key) < 127:  # 可打印字符
                print(f"未映射的按键: {key}")
    
    def key_release_timer(self, key, delay=0.1):
        """延时释放按键的定时器"""
        def release():
            time.sleep(delay)
            with self.lock:
                if key in self.pressed_keys and key in key_mapping:
                    bit_position = key_mapping[key]
                    self.pressed_keys.discard(key)
                    self.keys_state[0] &= ~(1 << bit_position)
                    # print(f"释放键: {key} -> 清除位 {bit_position}, 状态: {self.keys_state[0]:023b} (十进制: {self.keys_state[0]})")
        
        threading.Timer(delay, release).start()
    
    def keyboard_listener(self):
        """键盘监听线程"""
        print("键盘监听线程启动...")
        last_key_time = {}
        
        while self.running:
            try:
                key = self.get_char()
                if key:
                    current_time = time.time()
                    key_lower = key.lower()
                    
                    # 防止按键重复触发
                    if key_lower in last_key_time:
                        if current_time - last_key_time[key_lower] < 0.05:  # 50ms防抖
                            continue
                    
                    last_key_time[key_lower] = current_time
                    self.process_key(key)
                    
                    # 如果是有效按键，设置自动释放定时器
                    if key_lower in key_mapping:
                        self.key_release_timer(key_lower, 0.05)
                
                time.sleep(0.01)  # 10ms循环间隔
                
            except Exception as e:
                print(f"键盘监听异常: {e}")
                time.sleep(0.1)
    
    def publish_keys(self, pub):
        """发布按键状态"""
        with self.lock:
            msg = KeyInput()
            msg.keys = self.keys_state
            pub.publish(msg)
            
            # 只在有按键时打印状态（减少输出）
            if self.keys_state[0] != 0:
                pass
                # print(f"发布状态: {self.keys_state[0]:023b} (十进制: {self.keys_state[0]})")
    
    def run(self):
        """主运行函数"""
        if not self.setup_terminal():
            print("终端设置失败，退出...")
            return
            
        try:
            rospy.init_node('key_input_publisher', anonymous=True)
            pub = rospy.Publisher('key_input', KeyInput, queue_size=10)
            rate = rospy.Rate(20)  # 20Hz
            
            print("启动ROS节点成功\n按键布局:")
            print("  移动: Q/A(X轴), W/S(Y轴), E/D(Z轴)")
            print("  功能: R(初始化), F(服务), T(多功能), G(开始)")
            print("  旋转: Z/X(RX轴), C/V(RY轴), B/N(RZ轴)")
            print("  扩展: M(clog), J(find), K(high), L(center), U(low), I(top), O(bottom)")
            print("\n控制说明:")
            print("  - 请确保终端窗口获得焦点")
            print("  - 按ESC或Ctrl+C退出")
            print("  - 按键会自动在100ms后释放")
            print("-" * 60)
            
            # 启动键盘监听线程
            keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
            keyboard_thread.start()
            
            # 主发布循环
            while not rospy.is_shutdown() and self.running:
                self.publish_keys(pub)
                rate.sleep()
                
        except rospy.ROSInterruptException:
            print("ROS中断")
        except KeyboardInterrupt:
            print("\n收到键盘中断信号")
        except Exception as e:
            print(f"运行时异常: {e}")
        finally:
            self.running = False
            self.restore_terminal()

def main():
    publisher = KeyInputPublisher()
    publisher.run()

if __name__ == '__main__':
    main()