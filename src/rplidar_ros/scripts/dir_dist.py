#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

import math

class DirectionalLaser:
    def __init__(self):
        rospy.init_node('directional_laser_reader')
        rospy.Subscriber('/left_radar/filtered_scan', LaserScan, self.scan_callback)
        rospy.spin()

    def scan_callback(self, scan):
        # 目标方向角（单位：弧度）
        angles = {
            "front": math.pi,
            "left": -math.pi / 2,
            "right": math.pi / 2,
            "back": 0
        }

        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        ranges = scan.ranges
        num_ranges = len(ranges)
        window_size = 3  # 左右各取3个，总共7个点

        for direction, target_angle in angles.items():
            # 把目标角度归一化到 [-π, π]
            angle = math.atan2(math.sin(target_angle), math.cos(target_angle))
            
            # 检查是否在扫描范围内
            if angle < scan.angle_min or angle > scan.angle_max:
                print(f"{direction}: target angle {math.degrees(angle):.1f}° out of range")
                continue

            index = int((angle - angle_min) / angle_increment)

            # 获取窗口内的索引范围
            start = max(0, index - window_size)
            end = min(num_ranges, index + window_size + 1)
            window_ranges = ranges[start:end]

            # 去掉 inf 和 0 的无效值
            valid_ranges = [r for r in window_ranges if not math.isinf(r) and r > 0.01]

            if valid_ranges:
                avg_dist = sum(valid_ranges) / len(valid_ranges)
                print(f"{direction}: {avg_dist:.2f} m")
            else:
                print(f"{direction}: -1")

        print("---")

if __name__ == "__main__":
    DirectionalLaser()