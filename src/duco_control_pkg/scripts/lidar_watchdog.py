#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
雷达看门狗 - 监控三个雷达的/scan话题，超时3秒则自动重启对应节点
"""

import os
import rospy
import subprocess
import threading
import time
from sensor_msgs.msg import LaserScan

# 雷达配置: (话题, 节点名, launch文件)
RADAR_CONFIG = [
    ("/left_radar/scan", "/left_radar/rplidarNode", "rplidar_left.launch"),
    ("/main_radar/scan", "/main_radar/rplidarNode", "rplidar_mid.launch"),
    ("/right_radar/scan", "/right_radar/rplidarNode", "rplidar_right.launch"),
    ("/height_radar/scan", "/height_radar/rplidarNode", "rplidar_height.launch"),
]

TIMEOUT_SEC = 2.0
CHECK_INTERVAL = 1.0  # 检查间隔(秒)
RESTART_COOLDOWN = 5.0  # 同一雷达重启后冷却时间，避免频繁重启


class LidarWatchdog:
    def __init__(self):
        rospy.init_node("lidar_watchdog", anonymous=False)
        self.last_msg_time = {}  # topic -> last message time
        self.lock = threading.Lock()
        self.last_restart_time = {}  # topic -> last restart time
        self.launch_processes = {}  # topic -> Popen process

        # 订阅三个雷达的scan话题
        for topic, _, _ in RADAR_CONFIG:
            self.last_msg_time[topic] = time.time()
            self.last_restart_time[topic] = 0
            rospy.Subscriber(topic, LaserScan, self._make_callback(topic))

        rospy.loginfo("[LidarWatchdog] 已启动，监控话题: %s", [c[0] for c in RADAR_CONFIG])
        rospy.loginfo("[LidarWatchdog] 超时阈值: %.1f秒", TIMEOUT_SEC)

    def _make_callback(self, topic):
        def callback(msg):
            with self.lock:
                self.last_msg_time[topic] = time.time()
        return callback

    def _kill_node(self, node_name):
        """杀死指定ROS节点"""
        try:
            result = subprocess.run(
                ["rosnode", "kill", node_name],
                capture_output=True,
                text=True,
                timeout=5,
                env=dict(os.environ),
            )
            if result.returncode == 0:
                rospy.logwarn("[LidarWatchdog] 已终止节点: %s", node_name)
            else:
                # 节点可能已不存在
                rospy.logdebug("[LidarWatchdog] rosnode kill %s: %s", node_name, result.stderr)
        except subprocess.TimeoutExpired:
            rospy.logerr("[LidarWatchdog] 终止节点超时: %s", node_name)
        except Exception as e:
            rospy.logerr("[LidarWatchdog] 终止节点失败 %s: %s", node_name, e)

    def _relaunch_radar(self, launch_file):
        """后台启动roslaunch"""
        try:
            proc = subprocess.Popen(
                ["roslaunch", "rplidar_ros", launch_file],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=dict(os.environ),
            )
            rospy.loginfo("[LidarWatchdog] 已启动 roslaunch rplidar_ros %s (pid=%d)", launch_file, proc.pid)
            return proc
        except Exception as e:
            rospy.logerr("[LidarWatchdog] 启动失败 %s: %s", launch_file, e)
            return None

    def _restart_radar(self, topic, node_name, launch_file):
        """杀死节点并重新launch"""
        now = time.time()
        if now - self.last_restart_time[topic] < RESTART_COOLDOWN:
            rospy.logwarn("[LidarWatchdog] %s 冷却中，跳过重启", topic)
            return

        rospy.logwarn("[LidarWatchdog] %s 数据超时，正在重启...", topic)
        self.last_restart_time[topic] = now

        # 先终止旧节点
        self._kill_node(node_name)
        time.sleep(0.5)  # 等待节点完全退出

        # 后台启动新节点
        proc = self._relaunch_radar(launch_file)
        if proc:
            with self.lock:
                self.launch_processes[topic] = proc
                self.last_msg_time[topic] = time.time()  # 重置，避免刚启动就判定超时

    def _watchdog_loop(self):
        """定时检查各话题是否超时"""
        rate = rospy.Rate(1.0 / CHECK_INTERVAL)
        while not rospy.is_shutdown():
            now = time.time()
            to_restart = []
            with self.lock:
                for topic, node_name, launch_file in RADAR_CONFIG:
                    elapsed = now - self.last_msg_time[topic]
                    if elapsed > TIMEOUT_SEC:
                        rospy.logwarn("[LidarWatchdog] %s 超时 %.1f秒 (阈值%.1f秒)", topic, elapsed, TIMEOUT_SEC)
                        to_restart.append((topic, node_name, launch_file))
            for topic, node_name, launch_file in to_restart:
                self._restart_radar(topic, node_name, launch_file)
            rate.sleep()

    def run(self):
        """启动看门狗线程并保持主线程"""
        watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        watchdog_thread.start()
        rospy.spin()


if __name__ == "__main__":
    try:
        wd = LidarWatchdog()
        wd.run()
    except rospy.ROSInterruptException:
        pass
