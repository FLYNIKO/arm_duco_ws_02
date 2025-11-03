#!/usr/bin/env python3
import rospy
from key_input_pkg.msg import KeyInput
from pynput import keyboard

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

# 使用数组的第一个元素进行位操作存储按键状态
keys_state = [0] * 1  # 只需要一个元素

def publish_keys(pub):
    msg = KeyInput()
    msg.keys = keys_state
    pub.publish(msg)
    
    # 调试：打印当前按键状态
    if keys_state[0] != 0:
        print(f"发布按键状态: {keys_state[0]:023b} (十进制: {keys_state[0]})")

def on_press(key):
    try:
        k = key.char.lower()
        print(f"按下键: {k}")  # 调试输出
        if k in key_mapping:
            bit_position = key_mapping[k]
            # 使用位操作设置对应位为1
            keys_state[0] |= (1 << bit_position)
            print(f"设置位 {bit_position}, 当前状态: {keys_state[0]:023b}")  # 调试输出
        else:
            print(f"未映射的按键: {k}")  # 调试输出
    except AttributeError as e:
        print(f"按键检测异常: {e}, key: {key}")  # 调试输出

def on_release(key):
    try:
        k = key.char.lower()
        print(f"释放键: {k}")  # 调试输出
        if k in key_mapping:
            bit_position = key_mapping[k]
            # 使用位操作设置对应位为0
            keys_state[0] &= ~(1 << bit_position)
            print(f"清除位 {bit_position}, 当前状态: {keys_state[0]:023b}")  # 调试输出
    except AttributeError as e:
        print(f"按键释放异常: {e}, key: {key}")  # 调试输出

    if key == keyboard.Key.esc:
        print("检测到ESC键，退出...")
        return False

def publisher():
    rospy.init_node('key_input_publisher', anonymous=True)
    pub = rospy.Publisher('key_input', KeyInput, queue_size=10)
    rate = rospy.Rate(20)

    print("启动按键监听器...")
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    if listener.running:
        print("✓ 按键监听器启动成功")
    else:
        print("✗ 按键监听器启动失败")
        return

    print("按键监听中，ESC退出，正在发布按位编码的按键状态")
    print("按键布局:")
    print("  移动: Q/A(X轴), W/S(Y轴), E/D(Z轴)")
    print("  功能: R(初始化), F(服务), T(多功能), G(开始)")
    print("  旋转: Z/X(RX轴), C/V(RY轴), B/N(RZ轴)")
    print("  扩展: M(clog), J(find), K(high), L(center), U(low), I(top), O(bottom)")
    print("请在终端窗口中按键...")
    print("-" * 50)

    while not rospy.is_shutdown() and listener.running:
        publish_keys(pub)
        rate.sleep()

    listener.stop()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
