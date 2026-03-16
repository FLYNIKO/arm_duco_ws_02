#!/usr/bin/env python3
import sys
import time
import threading
import rospy
import os

base_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(base_dir, 'gen_py'))
sys.path.append(os.path.join(base_dir, 'lib'))
from DucoCobot import DucoCobot
from thrift import Thrift
from ManualControl import system_control
from std_msgs.msg import Float64MultiArray
from key_input_pkg.msg import KeyInput
from config import *

class DemoApp:
    def __init__(self):
        self.ip = IP
        self.stopheartthread = False
        self.duco_cobot = DucoCobot(IP, PORT)
        self.hearthread = threading.Thread(target=self.hearthread_fun)
        self.thread = threading.Thread(target=self.thread_fun)
        self.robot_state_thread = threading.Thread(target=self.robot_state_reader_fun)
        self.car_state_thread = threading.Thread(target=self.car_state_reader_fun)
        self.tcp_state = []  
        self.tcp_pub = rospy.Publisher('/Duco_state', Float64MultiArray, queue_size=20)
        self.adjust_pub = rospy.Publisher('/Duco_adjust', Float64MultiArray, queue_size=20) 
        self.sys_ctrl = None
        
        # 用于存储key_input和点云数据
        self.key_input_data = None
        self.cv2_H_points_data = None
        self.cv2_up_points_data = None
        self.data_lock = threading.Lock()
        
        # 用于存储机械臂状态（线程安全）
        self.robot_state_lock = threading.Lock()
        self.cached_tcp_pos = [0, 0, 0, 0, 0, 0]  # 默认值
        self.cached_tcp_state = [0, 0, 0, 0]  # 默认值
        
        # 用于存储车辆状态（线程安全）
        self.car_state_lock = threading.Lock()
        self.cached_car_state = [0, 0]  # 默认值
        self.cached_running_state = 0
        self.cached_distances = [0, 0, 0, 0]
        self.cached_spray_swinging = [0] * 10
        self.cached_lift_ctrl = -1
        self.cached_lift_height = 0
        self.cached_esp32_spray_state = 0
        
        # 共享的停止事件，供所有线程使用
        self._stop_event = threading.Event()
        
        # 订阅话题
        rospy.Subscriber('/key_input', KeyInput, self.key_input_callback)
        rospy.Subscriber('/cv2_H_points', Float64MultiArray, self.cv2_H_points_callback)
        rospy.Subscriber('/cv2_up_points', Float64MultiArray, self.cv2_up_points_callback) 

    def key_input_callback(self, msg):
        """处理/key_input话题的回调"""
        with self.data_lock:
            self.key_input_data = msg.keys
            # 根据keys[11]的值转发对应的点云数据
            self.forward_points_data()
    
    def cv2_H_points_callback(self, msg):
        """处理/cv2_H_points话题的回调"""
        with self.data_lock:
            self.cv2_H_points_data = msg
            # 如果当前选择的是cv2_H_points，则转发
            if self.key_input_data is not None and len(self.key_input_data) > 11:
                if self.key_input_data[11] == 0:
                    self.adjust_pub.publish(msg)
    
    def cv2_up_points_callback(self, msg):
        """处理/cv2_up_points话题的回调"""
        with self.data_lock:
            self.cv2_up_points_data = msg
            # 如果当前选择的是cv2_up_points，则转发
            if self.key_input_data is not None and len(self.key_input_data) > 11:
                if self.key_input_data[11] == 1:
                    self.adjust_pub.publish(msg)
    
    def forward_points_data(self):
        """根据key_input[11]的值转发对应的点云数据"""
        if self.key_input_data is None or len(self.key_input_data) <= 11:
            return
        
        if self.key_input_data[11] == 0:
            # 转发/cv2_H_points
            if self.cv2_H_points_data is not None:
                self.adjust_pub.publish(self.cv2_H_points_data)
        elif self.key_input_data[11] == 1:
            # 转发/cv2_up_points
            if self.cv2_up_points_data is not None:
                self.adjust_pub.publish(self.cv2_up_points_data)

    def robot_connect(self):
        rlt = self.duco_cobot.open()
        print("open:", rlt)
        rlt = self.duco_cobot.power_on(True)
        print("power_on:", rlt)
        rlt = self.duco_cobot.enable(True)
        print("enable:", rlt)
        self.duco_cobot.switch_mode(1)

    def hearthread_fun(self):
        self.duco_heartbeat = DucoCobot(self.ip, PORT)
        self.duco_heartbeat.open()
        while not self.stopheartthread:
            self.duco_heartbeat.rpc_heartbeat()
            self._stop_event.wait(1)
        self.duco_heartbeat.close()

    def robot_state_reader_fun(self):
        """专门用于读取机械臂状态的线程，避免阻塞发布线程"""
        self.duco_state_reader = DucoCobot(self.ip, PORT)
        self.duco_state_reader.open()
        
        while not self.stopheartthread:
            try:
                # 尝试获取机械臂姿态
                tcp_pos = self.duco_state_reader.get_tcp_pose()
            except Exception as e:
                # 若超时或通讯异常，保持旧值不变
                tcp_pos = None
                # rospy.logwarn_throttle(5, f"get_tcp_pose failed: {e}")

            try:
                tcp_state = self.duco_state_reader.get_robot_state()
            except Exception as e:
                # 若超时或通讯异常，保持旧值不变
                tcp_state = None
                # rospy.logwarn_throttle(5, f"get_robot_state failed: {e}")
            
            # 只有成功获取到新状态时才更新缓存
            with self.robot_state_lock:
                if tcp_pos is not None:
                    self.cached_tcp_pos = tcp_pos
                if tcp_state is not None:
                    self.cached_tcp_state = tcp_state
            
            # 状态读取可以稍微慢一点，比如每0.1秒读取一次
            self._stop_event.wait(0.1)
        
        self.duco_state_reader.close()

    def car_state_reader_fun(self):
        """专门用于读取车辆状态的线程，避免阻塞发布线程"""
        while not self.stopheartthread:
            # 等待 sys_ctrl 创建完成
            if self.sys_ctrl is not None:
                try:
                    car_state, running_state, distances, spray_swinging, lift_ctrl, lift_height, esp32_spray_state = self.sys_ctrl.get_car_state()
                    # 成功获取到新状态时才更新缓存（复制列表避免引用问题）
                    with self.car_state_lock:
                        self.cached_car_state = list(car_state) if isinstance(car_state, (list, tuple)) else car_state
                        self.cached_running_state = running_state
                        self.cached_distances = list(distances) if isinstance(distances, (list, tuple)) else distances
                        self.cached_spray_swinging = list(spray_swinging) if isinstance(spray_swinging, (list, tuple)) else spray_swinging
                        self.cached_lift_ctrl = lift_ctrl
                        self.cached_lift_height = lift_height
                        self.cached_esp32_spray_state = esp32_spray_state
                except Exception as e:
                    # 若获取失败，保持旧值不变
                    pass
                    # rospy.logwarn_throttle(5, f"get_car_state failed: {e}")
            
            # 车辆状态读取可以稍微慢一点，比如每0.1秒读取一次
            self._stop_event.wait(0.1)

    def thread_fun(self):
        """发布线程，使用已缓存的状态，不阻塞"""
        while not self.stopheartthread:
            # 从缓存中读取机械臂状态（线程安全）
            with self.robot_state_lock:
                tcp_pos = self.cached_tcp_pos.copy()
                tcp_state = self.cached_tcp_state.copy()
            
            self.tcp_state = tcp_state

            # 从缓存中读取车辆状态（线程安全）
            with self.car_state_lock:
                car_state = self.cached_car_state.copy()
                running_state = self.cached_running_state
                distances = self.cached_distances.copy()
                spray_swinging = self.cached_spray_swinging.copy()
                lift_ctrl = self.cached_lift_ctrl
                lift_height = self.cached_lift_height
                esp32_spray_state = self.cached_esp32_spray_state
            # 组装并发布 ROS 消息
            msg = Float64MultiArray()
            msg.data = (
                tcp_state
                + tcp_pos
                + distances
                + car_state
                + [running_state]
                + spray_swinging
                + [lift_ctrl]
                + [lift_height]
                + [esp32_spray_state]
            )

            self.tcp_pub.publish(msg)

            # 每0.05秒循环一次（不再被任何状态读取阻塞）
            self._stop_event.wait(0.05)

    def run(self):
        self.robot_connect()
        self.hearthread.start()
        self.robot_state_thread.start()  # 启动机械臂状态读取线程
        self.car_state_thread.start()  # 启动车辆状态读取线程
        self.thread.start()

        try:
            self.sys_ctrl = system_control(self.duco_cobot, self)
            self.sys_ctrl.run()
        finally:
            self.stopheartthread = True
            time.sleep(1)
            self.hearthread.join()
            self.robot_state_thread.join()  # 等待机械臂状态读取线程结束
            self.car_state_thread.join()  # 等待车辆状态读取线程结束
            self.thread.join()
            rlt = self.duco_cobot.close()
            print("close:", rlt)

    def main(self):
        try:
            self.run()
        except Thrift.TException as tx:
            print('%s' % tx.message)

if __name__ == '__main__':
    rospy.init_node('Duco_state_publisher', anonymous=True)
    app = DemoApp()
    try:
        app.main()
    except KeyboardInterrupt:
        print("主程序收到 KeyboardInterrupt，准备退出。")
        app.stopheartthread = True
        time.sleep(1)
        app.hearthread.join()
        app.robot_state_thread.join()  # 等待机械臂状态读取线程结束
        app.car_state_thread.join()  # 等待车辆状态读取线程结束
        app.thread.join()
        rlt = app.duco_cobot.close()
        print("close:", rlt)
