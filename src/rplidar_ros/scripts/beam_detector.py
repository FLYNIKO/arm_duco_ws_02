#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class WallFitter:
    def __init__(self):
        rospy.init_node("wall_fitter")
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.marker_pub = rospy.Publisher("/wall_marker", Marker, queue_size=1)

    def scan_callback(self, scan_msg):
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        ranges = scan_msg.ranges

        points = []

        for i in range(len(ranges)):
            angle = angle_min + i * angle_increment
            # 选取前方 ±10° 以内的点，距离在有效范围内
            if abs(angle - math.pi) < math.radians(10) and 0.1 < ranges[i] < 5.0:
                rx = ranges[i] * math.cos(angle)
                ry = ranges[i] * math.sin(angle)

                # 转换为前+X，右+Y
                x = -rx
                y = -ry
                points.append([x, y])

        if len(points) < 10:
            return

        points_np = np.array(points)
        xs, ys = points_np[:, 0], points_np[:, 1]

        # 最小二乘拟合 y = a * x + b
        A = np.vstack([xs, np.ones(len(xs))]).T
        a, b = np.linalg.lstsq(A, ys, rcond=None)[0]

        # 沿 x 的范围，计算拟合线两端点
        x_min, x_max = np.min(xs), np.max(xs)
        start = (x_min, a * x_min + b)
        end = (x_max, a * x_max + b)

        self.publish_line(start, end)

        # 输出结果
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        angle = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
        rospy.loginfo(f"Fitted line: start=({start[0]:.2f}, {start[1]:.2f}), "
                      f"end=({end[0]:.2f}, {end[1]:.2f}), "
                      f"length={length:.2f}m, angle={angle:.2f}°")

    def publish_line(self, start, end):
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "wall"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        p1 = Point(x=start[0], y=start[1], z=0)
        p2 = Point(x=end[0], y=end[1], z=0)
        marker.points = [p1, p2]

        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

if __name__ == "__main__":
    WallFitter()
    rospy.spin()
