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
        self.tcp_state = []  
        self.tcp_pub = rospy.Publisher('/Duco_state', Float64MultiArray, queue_size=20)
        self.adjust_pub = rospy.Publisher('/Duco_adjust', Float64MultiArray, queue_size=20) 
        self.sys_ctrl = None
        
        # 用于存储key_input和点云数据
        self.key_input_data = None
        self.cv2_H_points_data = None
        self.cv2_up_points_data = None
        self.data_lock = threading.Lock()
        
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
        self._stop_event = threading.Event()
        self.duco_heartbeat = DucoCobot(self.ip, PORT)
        self.duco_heartbeat.open()
        while not self.stopheartthread:
            self.duco_heartbeat.rpc_heartbeat()
            self._stop_event.wait(1)
        self.duco_heartbeat.close()

    def thread_fun(self):
        self.duco_thread = DucoCobot(self.ip, PORT)
        self.duco_thread.open()

        while not self.stopheartthread:
            try:
                # 尝试获取机械臂姿态
                tcp_pos = self.duco_thread.get_tcp_pose()
            except Exception as e:
                # 若超时或通讯异常，则使用默认值
                tcp_pos = [0, 0, 0, 0, 0, 0]
                # 可选：打印一次警告（不要每次都打印，避免刷屏）
                # rospy.logwarn_throttle(5, f"get_tcp_pose failed: {e}")

            try:
                tcp_state = self.duco_thread.get_robot_state()
            except Exception as e:
                tcp_state = [0, 0, 0, 0]
                # rospy.logwarn_throttle(5, f"get_robot_state failed: {e}")

            # 从 system_control 获取车辆状态
            if self.sys_ctrl is not None:
                try:
                    car_state, running_state, distances, spray_swinging, lift_ctrl, lift_height = self.sys_ctrl.get_car_state()
                    # lift_ctrl, lift_height = self.sys_ctrl.get_lift_info()
                except Exception as e:
                    car_state = [0, 0]
                    running_state = 0
                    distances = [0, 0, 0, 0]
                    spray_swinging = [0] * 10
                    lift_ctrl = -8
                    lift_height = 0.5
                    # rospy.logwarn_throttle(5, f"get_car_state failed: {e}")
            else:
                tcp_pos = [0, 0, 0, 0, 0, 0]
                tcp_state = [0, 0, 0, 0]
                distances = [0, 0, 0, 0]
                running_state = 0
                car_state = [0, 0]
                spray_swinging = [0] * 10
                lift_ctrl = -1
                lift_height = 0
            self.tcp_state = tcp_state

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
            )

            self.tcp_pub.publish(msg)

            # 每0.05秒循环一次
            self._stop_event.wait(0.05)

        self.duco_thread.close()

    def run(self):
        self.robot_connect()
        self.hearthread.start()
        self.thread.start()

        try:
            self.sys_ctrl = system_control(self.duco_cobot, self)
            self.sys_ctrl.run()
        finally:
            self.stopheartthread = True
            time.sleep(1)
            self.hearthread.join()
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
        app.thread.join()
        rlt = app.duco_cobot.close()
        print("close:", rlt)
