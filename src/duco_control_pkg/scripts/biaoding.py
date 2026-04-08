#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from scipy.optimize import minimize

class LidarCalibrator:
    def __init__(self):
        rospy.init_node("lidar_calibrator")

        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.scans = []   # 存储多帧数据
        self.max_frames = 10

        self.current_scan = None

        print("按回车采集一帧，采集10帧后自动开始标定")

    def scan_callback(self, msg):
        self.current_scan = msg

    def get_points(self, scan):
        points = []
        angle = scan.angle_min

        for r in scan.ranges:
            if np.isfinite(r) and 0.1 < r < 5.0:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append([x, y])
            angle += scan.angle_increment

        return np.array(points)

    def fit_line_pca(self, points):
        # PCA拟合直线
        centroid = np.mean(points, axis=0)
        cov = np.cov(points.T)
        eigvals, eigvecs = np.linalg.eig(cov)

        direction = eigvecs[:, np.argmax(eigvals)]
        normal = np.array([-direction[1], direction[0]])

        a, b = normal
        c = -a * centroid[0] - b * centroid[1]

        return a, b, c

    def transform_points(self, pts, tx, ty, theta):
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])
        return (R @ pts.T).T + np.array([tx, ty])

    def collect_data(self):
        while len(self.scans) < self.max_frames:
            input("按回车采集一帧...")
            if self.current_scan is None:
                print("还没收到scan数据")
                continue

            pts = self.get_points(self.current_scan)
            self.scans.append(pts)
            print(f"已采集 {len(self.scans)} / {self.max_frames}")

    def build_reference_line(self):
        # 用第一帧作为参考
        pts = self.scans[0]
        return self.fit_line_pca(pts)

    def point_to_line_distance(self, pts, a, b, c):
        return np.abs(a * pts[:,0] + b * pts[:,1] + c) / np.sqrt(a*a + b*b)

    def optimize(self):
        ref_a, ref_b, ref_c = self.build_reference_line()

        def loss(params):
            tx, ty, theta = params
            total_error = 0

            for pts in self.scans:
                pts_world = self.transform_points(pts, tx, ty, theta)
                dist = self.point_to_line_distance(pts_world, ref_a, ref_b, ref_c)
                total_error += np.mean(dist**2)

            return total_error

        # 初值（你可以改成尺子量的）
        init = [0.0, 0.0, 0.0]

        result = minimize(loss, init, method='BFGS')

        tx, ty, theta = result.x

        print("\n====== 标定结果 ======")
        print(f"tx = {tx:.4f} m")
        print(f"ty = {ty:.4f} m")
        print(f"theta = {np.degrees(theta):.2f} deg")
        print(f"误差 = {result.fun:.6f}")

    def run(self):
        rospy.sleep(1)
        self.collect_data()
        self.optimize()


if __name__ == "__main__":
    calibrator = LidarCalibrator()
    calibrator.run()