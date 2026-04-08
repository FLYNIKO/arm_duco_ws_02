#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import math


class CylinderRANSAC:
    def __init__(self):
        rospy.init_node('cylinder_ransac_node', anonymous=True)
        
        # RANSAC 参数
        self.max_iterations = 1000  # 最大迭代次数
        self.distance_threshold = 0.05  # 内点距离阈值（米）
        self.min_inliers = 10  # 最小内点数量
        self.min_arc_angle = 20.0  # 最小圆弧角度（度）
        
        # 圆半径范围限制
        self.min_radius = 0.1  # 最小半径（米）
        self.max_radius = 1.0   # 最大半径（米）
        
        # 订阅激光雷达话题
        self.scan_sub = rospy.Subscriber(
            '/main_radar/filtered_scan',
            LaserScan,
            self.scan_callback
        )
        
        # 发布圆心位置
        self.center_pub = rospy.Publisher(
            '/cylinder_center',
            PointStamped,
            queue_size=10
        )
        
        # 发布可视化标记
        self.marker_pub = rospy.Publisher(
            '/cylinder_marker',
            Marker,
            queue_size=10
        )
        
        rospy.loginfo("圆柱RANSAC拟合节点已启动")
    
    def scan_callback(self, msg):
        """处理激光雷达扫描数据"""
        # 将激光扫描数据转换为笛卡尔坐标点
        points = self.scan_to_cartesian(msg)
        
        if len(points) < 3:
            rospy.logwarn("点数太少，无法拟合圆")
            return
        
        # 使用RANSAC拟合圆
        center, radius, inliers = self.ransac_fit_circle(points)
        
        if center is not None:
            rospy.loginfo_throttle(1.0, f"检测到圆柱 - 圆心: ({center[0]:.3f}, {center[1]:.3f}), 半径: {radius:.3f}m, 内点数: {len(inliers)}")
            
            # 发布圆心
            self.publish_center(center, radius, msg.header)
            
            # 发布可视化标记
            self.publish_marker(center, radius, msg.header)
        else:
            rospy.logdebug_throttle(1.0, "未检测到符合条件的圆柱")
    
    def scan_to_cartesian(self, scan_msg):
        """将激光扫描数据转换为笛卡尔坐标点"""
        points = []
        angle = scan_msg.angle_min
        
        for i, r in enumerate(scan_msg.ranges):
            # 过滤无效距离
            if scan_msg.range_min <= r <= scan_msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
            
            angle += scan_msg.angle_increment
        
        return np.array(points)
    
    def fit_circle_from_three_points(self, p1, p2, p3):
        """从三个点计算圆心和半径"""
        # 使用三点确定圆的解析解
        ax, ay = p1
        bx, by = p2
        cx, cy = p3
        
        # 计算中垂线
        d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        
        if abs(d) < 1e-10:
            return None, None
        
        # 圆心坐标
        ux = ((ax**2 + ay**2) * (by - cy) + 
              (bx**2 + by**2) * (cy - ay) + 
              (cx**2 + cy**2) * (ay - by)) / d
        
        uy = ((ax**2 + ay**2) * (cx - bx) + 
              (bx**2 + by**2) * (ax - cx) + 
              (cx**2 + cy**2) * (bx - ax)) / d
        
        center = np.array([ux, uy])
        radius = np.linalg.norm(p1 - center)
        
        return center, radius
    
    def calculate_inliers(self, points, center, radius, threshold):
        """计算内点"""
        distances = np.abs(np.linalg.norm(points - center, axis=1) - radius)
        inliers_idx = np.where(distances < threshold)[0]
        return inliers_idx
    
    def calculate_arc_angle(self, points, center):
        """计算圆弧的角度范围"""
        # 计算每个点相对于圆心的角度
        vectors = points - center
        angles = np.arctan2(vectors[:, 1], vectors[:, 0])
        
        # 计算角度范围
        angle_range = np.max(angles) - np.min(angles)
        
        # 处理跨越±π的情况
        if angle_range > np.pi:
            angle_range = 2 * np.pi - angle_range
        
        return np.degrees(angle_range)
    
    def ransac_fit_circle(self, points):
        """使用RANSAC算法拟合圆"""
        if len(points) < 3:
            return None, None, None
        
        best_center = None
        best_radius = None
        best_inliers = []
        max_inliers_count = 0
        best_model_center = None
        best_model_radius = None
        
        for _ in range(self.max_iterations):
            # 随机选择3个点
            idx = np.random.choice(len(points), 3, replace=False)
            sample_points = points[idx]
            
            # 拟合圆
            center, radius = self.fit_circle_from_three_points(
                sample_points[0],
                sample_points[1],
                sample_points[2]
            )
            
            if center is None or radius is None:
                continue
            
            # 检查半径是否在合理范围内
            if radius < self.min_radius or radius > self.max_radius:
                continue
            
            # 计算内点
            inliers_idx = self.calculate_inliers(
                points, center, radius, self.distance_threshold
            )
            
            # 检查内点数量
            if len(inliers_idx) < self.min_inliers:
                continue
            
            # 检查圆弧角度
            arc_angle = self.calculate_arc_angle(points[inliers_idx], center)
            if arc_angle < self.min_arc_angle:
                continue
            
            # 更新最佳模型
            if len(inliers_idx) > max_inliers_count:
                max_inliers_count = len(inliers_idx)
                best_model_center = center
                best_model_radius = radius
                best_inliers = inliers_idx
        
        # 如果找到了最佳模型，使用所有内点进行优化拟合
        if best_model_center is not None and len(best_inliers) > 0:
            # 使用更宽松的阈值重新选择内点（基于最佳模型）
            expanded_inliers = self.calculate_inliers(
                points, best_model_center, best_model_radius, 
                self.distance_threshold * 2.0  # 使用更宽松的阈值
            )
            
            # 如果扩展的内点更多，使用扩展的内点
            if len(expanded_inliers) > len(best_inliers):
                best_inliers = expanded_inliers
            
            # 使用所有内点进行精确拟合
            best_center, best_radius = self.refine_circle_fit(
                points[best_inliers]
            )
            
            return best_center, best_radius, best_inliers
        else:
            return None, None, None
    
    def refine_circle_fit(self, points):
        """使用代数圆拟合（Kasa方法）优化圆拟合"""
        # 使用代数圆拟合方法，对圆弧数据更准确
        n = len(points)
        if n < 3:
            # 如果点数太少，使用简单方法
            centroid = np.mean(points, axis=0)
            radius = np.mean(np.linalg.norm(points - centroid, axis=1))
            return centroid, radius
        
        # 提取x和y坐标
        x = points[:, 0]
        y = points[:, 1]
        
        # 计算质心
        x_mean = np.mean(x)
        y_mean = np.mean(y)
        
        # 中心化坐标
        u = x - x_mean
        v = y - y_mean
        
        # 计算辅助变量
        suu = np.sum(u * u)
        suv = np.sum(u * v)
        svv = np.sum(v * v)
        suuu = np.sum(u * u * u)
        svvv = np.sum(v * v * v)
        suuv = np.sum(u * u * v)
        suvv = np.sum(u * v * v)
        
        # 求解线性方程组
        # 使用Kasa方法（代数圆拟合）
        A = np.array([[suu, suv], [suv, svv]])
        b = np.array([0.5 * (suuu + suvv), 0.5 * (svvv + suuv)])
        
        # 检查矩阵是否可逆
        det = A[0, 0] * A[1, 1] - A[0, 1] * A[1, 0]
        if abs(det) < 1e-10:
            # 如果矩阵不可逆，使用简单方法
            centroid = np.array([x_mean, y_mean])
            radius = np.mean(np.linalg.norm(points - centroid, axis=1))
            return centroid, radius
        
        # 求解圆心偏移
        uc = (b[0] * A[1, 1] - b[1] * A[0, 1]) / det
        vc = (A[0, 0] * b[1] - A[1, 0] * b[0]) / det
        
        # 计算圆心（在原始坐标系中）
        center = np.array([x_mean + uc, y_mean + vc])
        
        # 计算半径（使用所有点到圆心的距离）
        distances = np.linalg.norm(points - center, axis=1)
        radius = np.mean(distances)
        
        # 使用几何圆拟合进行进一步优化（迭代优化）
        # 这可以处理圆弧数据，因为代数方法可能对圆弧有偏差
        for iteration in range(15):
            # 计算每个点到圆心的距离
            distances = np.linalg.norm(points - center, axis=1)
            
            # 更新半径（使用均值，对完整圆更准确）
            radius = np.mean(distances)
            
            # 计算梯度并更新圆心
            # 对于每个点，计算到圆的距离误差
            errors = distances - radius
            # 计算归一化方向向量（从圆心指向点）
            directions = (points - center) / (distances[:, np.newaxis] + 1e-10)
            # 更新圆心（沿着误差的梯度方向）
            gradient = np.mean(directions * errors[:, np.newaxis], axis=0)
            center += gradient * 0.3  # 使用较小的步长
            
            # 如果梯度很小，提前停止
            if np.linalg.norm(gradient) < 1e-6:
                break
        
        # 最终计算半径
        distances = np.linalg.norm(points - center, axis=1)
        radius = np.mean(distances)
        
        return center, radius
    
    def publish_center(self, center, radius, header):
        """发布圆心位置"""
        point_msg = PointStamped()
        point_msg.header = header
        point_msg.point.x = center[0]
        point_msg.point.y = center[1]
        point_msg.point.z = radius
        
        self.center_pub.publish(point_msg)
    
    def publish_marker(self, center, radius, header):
        """发布可视化标记"""
        marker = Marker()
        marker.header = header
        marker.ns = "cylinder"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # 设置位置
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # 设置大小
        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = 0.5
        
        # 设置颜色（半透明绿色）
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        
        marker.lifetime = rospy.Duration(0.5)
        
        self.marker_pub.publish(marker)
    
    def run(self):
        """运行节点"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = CylinderRANSAC()
        node.run()
    except rospy.ROSInterruptException:
        pass