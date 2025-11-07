#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import matplotlib.pyplot as plt
from collections import deque
from config import *
import threading

from duco_control_pkg.msg import LineInfo, LineDetectionArray 

class SeparateRadarLineDetector:
    def __init__(self):
        rospy.init_node('main_radar_line_detector')
        self.debug_mode = True
        # Parameters for image conversion
        self.image_size = 800  # 适中的图像尺寸
        self.max_range = 1.0      # 适中的范围
        self.resolution = self.max_range / (self.image_size / 2)
        
        # 圆形处理范围参数
        self.processing_radius_meters = 1  # 处理半径（米），只处理此范围内的数据
        self.processing_radius_pixels = int(self.processing_radius_meters / self.resolution)  # 转换为像素
        
        # Enhanced Hough line detection parameters (保持原有准确的参数)
        self.hough_threshold = 25
        self.min_line_length = 20
        self.max_line_gap = 50
        
        # Edge detection parameters
        self.canny_low = 10
        self.canny_high = 90
        self.gaussian_kernel = 3
        
        # Stability parameters
        self.temporal_buffer_size = 3  # 减少从5到3
        self.line_id_counter = 0
        
        # Line tracking and stability parameters
        self.position_threshold = 0.5
        self.angle_threshold = 20
        self.stability_requirement = 1  # 减少从3到2，更快达到稳定
        self.max_line_age = 5
        
        # Advanced filtering parameters
        self.min_line_length_meters = 0.05
        self.max_line_length_meters = 8.0
        self.angle_tolerance_deg = 75
        self.density_threshold = 0.6
        
        # 主雷达配置（使用雷达本地坐标系）
        self.radar_config = {
            'topic': '/main_radar/filtered_scan',
            'frame_id': 'main_radar',  # 使用雷达本地坐标系
            'data': None,
            'timestamp': None,
            'line_history': deque(maxlen=self.temporal_buffer_size),
            'stable_lines': [],
            'line_id_counter': 0
        }
        
        # 数据同步参数
        self.data_lock = threading.Lock()
        
        # Subscriber and publishers for main radar
        self.main_scan_sub = rospy.Subscriber(
            self.radar_config['topic'], 
            LaserScan, 
            self.radar_callback
        )
        
        # 为主雷达创建发布器
        self.marker_pub = rospy.Publisher('/main_radar/detected_lines', MarkerArray, queue_size=10)
        self.scan_points_pub = rospy.Publisher('/main_radar/scan_points', Marker, queue_size=10)
        self.debug_pub = rospy.Publisher('/main_radar/debug_lines', MarkerArray, queue_size=10)
        
        self.line_info_pub = rospy.Publisher('/main_radar/line_detection_info', LineDetectionArray, queue_size=10)
        
        # 打印初始化信息
        rospy.loginfo(f"Main Radar Line Detector initialized")

    def radar_callback(self, scan_msg):
        """雷达数据回调函数"""
        with self.data_lock:
            self.radar_config['data'] = scan_msg
            self.radar_config['timestamp'] = rospy.Time.now()
            
        # 处理雷达数据
        self.process_single_radar(scan_msg)

    def polar_to_cartesian(self, ranges, angles):
        """Convert polar coordinates to cartesian coordinates"""
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        return x, y

    def cartesian_to_image_coords(self, x, y):
        """Convert cartesian coordinates to image pixel coordinates"""
        center = self.image_size // 2
        img_x = center + (x / self.resolution).astype(int)
        img_y = center - (y / self.resolution).astype(int)
        img_x = np.clip(img_x, 0, self.image_size - 1)
        img_y = np.clip(img_y, 0, self.image_size - 1)
        return img_x, img_y

    def image_coords_to_cartesian(self, img_x, img_y):
        """Convert image pixel coordinates back to cartesian coordinates"""
        center = self.image_size // 2
        x = (img_x - center) * self.resolution
        y = (center - img_y) * self.resolution
        return x, y

    def create_enhanced_occupancy_image(self, scan_points):
        """从单个雷达的点云数据创建占用栅格图像"""
        if len(scan_points[0]) == 0:
            return np.zeros((self.image_size, self.image_size), dtype=np.uint8), ([], [])
        
        x_coords = scan_points[0]
        y_coords = scan_points[1]
        
        img_x, img_y = self.cartesian_to_image_coords(x_coords, y_coords)
        
        # 创建基础占用图像
        occupancy_img = np.zeros((self.image_size, self.image_size), dtype=np.uint8)
        
        # 只处理圆形范围内的点
        center = self.image_size // 2
        for i in range(len(img_x)):
            # 计算点到中心的距离（像素）
            dist_from_center = math.sqrt((img_x[i] - center)**2 + (img_y[i] - center)**2)
            
            # 只保留在指定半径内的点
            if dist_from_center <= self.processing_radius_pixels:
                occupancy_img[img_y[i], img_x[i]] = 255
        
        # 多尺度形态学操作提高线条连续性
        # 小核处理细节
        kernel_small = np.ones((2, 2), np.uint8)
        occupancy_img = cv2.morphologyEx(occupancy_img, cv2.MORPH_CLOSE, kernel_small)
        
        # 中等核连接临近点
        kernel_medium = np.ones((3, 3), np.uint8)
        occupancy_img = cv2.dilate(occupancy_img, kernel_medium, iterations=1)
        
        # 大核处理主要结构连续性
        kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        occupancy_img = cv2.morphologyEx(occupancy_img, cv2.MORPH_CLOSE, kernel_large)
        
        return occupancy_img, (x_coords, y_coords)

    def detect_lines_multi_scale(self, image):
        """Multi-scale line detection for better stability"""
        # Apply different scales of Gaussian blur
        blurred_fine = cv2.GaussianBlur(image, (3, 3), 0)
        blurred_coarse = cv2.GaussianBlur(image, (7, 7), 0)
        
        # Edge detection at multiple scales
        edges_fine = cv2.Canny(blurred_fine, self.canny_low, self.canny_high)
        edges_coarse = cv2.Canny(blurred_coarse, self.canny_low // 2, self.canny_high // 2)
        
        # Combine edges
        edges_combined = cv2.bitwise_or(edges_fine, edges_coarse)
        
        # Multiple Hough transforms with different parameters
        lines_strict = cv2.HoughLinesP(edges_combined, 
                                     rho=1, 
                                     theta=np.pi/180, 
                                     threshold=self.hough_threshold + 10,
                                     minLineLength=self.min_line_length + 10,
                                     maxLineGap=self.max_line_gap - 5)
        
        lines_loose = cv2.HoughLinesP(edges_combined, 
                                    rho=1, 
                                    theta=np.pi/180, 
                                    threshold=self.hough_threshold,
                                    minLineLength=self.min_line_length,
                                    maxLineGap=self.max_line_gap)
        
        # Combine results
        all_lines = []
        if lines_strict is not None:
            all_lines.extend(lines_strict)
        if lines_loose is not None:
            all_lines.extend(lines_loose)
        
        return all_lines, edges_combined

    def calculate_line_properties(self, x1, y1, x2, y2):
        """Calculate comprehensive line properties in radar coordinate system"""
        # 转换图像坐标到雷达本地笛卡尔坐标
        start_x, start_y = self.image_coords_to_cartesian(x1, y1)
        end_x, end_y = self.image_coords_to_cartesian(x2, y2)
        
        length = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        angle = math.atan2(end_y - start_y, end_x - start_x)
        
        # Normalize angle to [0, pi] for consistency
        angle_normalized = angle % math.pi
        
        mid_x = (start_x + end_x) / 2
        mid_y = (start_y + end_y) / 2
        distance = math.sqrt(mid_x**2 + mid_y**2)
        
        return {
            'start_point': (start_x, start_y),
            'end_point': (end_x, end_y),
            'midpoint': (mid_x, mid_y),
            'length': length,
            'angle_rad': angle_normalized,
            'angle_deg': math.degrees(angle_normalized),
            'distance_from_origin': distance,
            'radar_source': 'main'
        }

    def validate_line_with_scan_data(self, line_props, scan_points):
        """Validate line by checking point density along the line"""
        if len(scan_points[0]) == 0:
            return False
        
        start = np.array(line_props['start_point'])
        end = np.array(line_props['end_point'])
        line_vec = end - start
        line_length = np.linalg.norm(line_vec)
        
        if line_length < self.min_line_length_meters:
            return False
        
        # Check how many scan points are close to the line
        scan_points_array = np.column_stack(scan_points)
        
        # Calculate distance from each scan point to the line
        points_on_line = 0
        tolerance = 0.1  # 10cm tolerance
        
        for point in scan_points_array:
            point_vec = point - start
            projection_length = np.dot(point_vec, line_vec) / line_length
            
            # Check if projection is within line segment
            if 0 <= projection_length <= line_length:
                projection_point = start + (projection_length / line_length) * line_vec
                distance_to_line = np.linalg.norm(point - projection_point)
                
                if distance_to_line <= tolerance:
                    points_on_line += 1
        
        # Calculate density
        expected_points = max(1, int(line_length / 0.05))  # Expected points every 5cm
        density = points_on_line / expected_points
        
        return density >= self.density_threshold

    def cluster_similar_lines(self, lines):
        """Advanced clustering of similar lines"""
        if not lines:
            return []
        
        clusters = []
        used = [False] * len(lines)
        
        for i, line1 in enumerate(lines):
            if used[i]:
                continue
            
            cluster = [line1]
            used[i] = True
            
            for j, line2 in enumerate(lines[i+1:], i+1):
                if used[j]:
                    continue
                
                if self.are_lines_similar(line1['properties'], line2['properties']):
                    cluster.append(line2)
                    used[j] = True
            
            clusters.append(cluster)
        
        return clusters

    def are_lines_similar(self, props1, props2):
        """Check if two lines are similar enough to be merged"""
        # Angle similarity (considering 0/180 degree wrap-around)
        angle_diff = abs(props1['angle_deg'] - props2['angle_deg'])
        angle_diff = min(angle_diff, 180 - angle_diff)
        
        # Position similarity
        pos_diff = math.sqrt((props1['midpoint'][0] - props2['midpoint'][0])**2 + 
                           (props1['midpoint'][1] - props2['midpoint'][1])**2)
        
        # Parallel lines check (similar angle, different position)
        parallel_check = angle_diff < self.angle_threshold and pos_diff < self.position_threshold * 2
        
        # Collinear lines check (similar position and angle)
        collinear_check = angle_diff < self.angle_threshold and pos_diff < self.position_threshold
        
        return parallel_check or collinear_check

    def merge_line_cluster(self, cluster):
        """Merge a cluster of similar lines using the longest line as base"""
        if len(cluster) == 1:
            return cluster[0]
        
        # Find the longest line in the cluster as the base
        longest_line = max(cluster, key=lambda x: x['properties']['length'])
        
        if len(cluster) == 2:
            # For two lines, extend the longest one to cover both
            other_line = cluster[0] if cluster[1] == longest_line else cluster[1]
            return self.extend_line_to_cover(longest_line, other_line)
        
        # For more than 2 lines, collect all endpoints and fit
        all_endpoints = []
        for line in cluster:
            all_endpoints.extend([line['properties']['start_point'], 
                                line['properties']['end_point']])
        
        # Remove duplicates and find the extreme points along the main direction
        points = np.array(all_endpoints)
        
        # Use the longest line's direction as reference
        base_start = np.array(longest_line['properties']['start_point'])
        base_end = np.array(longest_line['properties']['end_point'])
        base_direction = base_end - base_start
        base_direction = base_direction / np.linalg.norm(base_direction)
        
        # Project all points onto the base line direction
        projections = []
        for point in points:
            point_vec = point - base_start
            projection_scalar = np.dot(point_vec, base_direction)
            projections.append(projection_scalar)
        
        # Find the extreme projections
        min_proj = min(projections)
        max_proj = max(projections)
        
        # Calculate new endpoints
        new_start = base_start + min_proj * base_direction
        new_end = base_start + max_proj * base_direction
        
        # Convert to image coordinates
        img_coords = self.cartesian_to_image_coords(
            np.array([new_start[0], new_end[0]]), 
            np.array([new_start[1], new_end[1]])
        )
        
        # Calculate properties
        props = self.calculate_line_properties(img_coords[0][0], img_coords[1][0], 
                                             img_coords[0][1], img_coords[1][1])
        
        return {
            'image_coords': (img_coords[0][0], img_coords[1][0], img_coords[0][1], img_coords[1][1]),
            'properties': props
        }
    
    def extend_line_to_cover(self, base_line, other_line):
        """Extend base line to cover the other line"""
        # Get all four endpoints
        points = np.array([
            base_line['properties']['start_point'],
            base_line['properties']['end_point'],
            other_line['properties']['start_point'],
            other_line['properties']['end_point']
        ])
        
        # Use base line direction
        base_start = np.array(base_line['properties']['start_point'])
        base_end = np.array(base_line['properties']['end_point'])
        direction = base_end - base_start
        direction = direction / np.linalg.norm(direction)
        
        # Project all points onto the line
        projections = []
        for point in points:
            point_vec = point - base_start
            projection_scalar = np.dot(point_vec, direction)
            projections.append(projection_scalar)
        
        # Find extreme points
        min_proj = min(projections)
        max_proj = max(projections)
        
        new_start = base_start + min_proj * direction
        new_end = base_start + max_proj * direction
        
        # Convert to image coordinates
        img_coords = self.cartesian_to_image_coords(
            np.array([new_start[0], new_end[0]]), 
            np.array([new_start[1], new_end[1]])
        )
        
        props = self.calculate_line_properties(img_coords[0][0], img_coords[1][0], 
                                             img_coords[0][1], img_coords[1][1])
        
        return {
            'image_coords': (img_coords[0][0], img_coords[1][0], img_coords[0][1], img_coords[1][1]),
            'properties': props
        }

    def track_lines_temporally(self, current_lines):
        """Track lines across multiple frames for stability"""
        # Add current detection to history
        self.radar_config['line_history'].append(current_lines)
        
        if len(self.radar_config['line_history']) < self.stability_requirement:
            rospy.loginfo(f"Main radar: Not enough history yet ({len(self.radar_config['line_history'])}/{self.stability_requirement})")
            return []  # Not enough history yet
        
        # Find consistently detected lines
        stable_candidates = []
        
        for current_line in current_lines:
            consistency_count = 1  # Current frame
            
            # Check consistency across history
            for historical_frame in list(self.radar_config['line_history'])[:-1]:
                for historical_line in historical_frame:
                    if self.are_lines_similar(current_line['properties'], 
                                            historical_line['properties']):
                        consistency_count += 1
                        break
            
            # If line appears consistently, it's stable
            if consistency_count >= self.stability_requirement:
                current_line['stability_score'] = consistency_count
                stable_candidates.append(current_line)
        
        return stable_candidates

    def update_stable_lines(self, new_stable_lines):
        """Update the stable line list with temporal filtering"""
        stable_lines = self.radar_config['stable_lines']
        
        # Age existing stable lines
        for line in stable_lines:
            line['age'] = line.get('age', 0) + 1
        
        # Remove old lines
        stable_lines[:] = [line for line in stable_lines 
                          if line['age'] < self.max_line_age]
        
        # Update or add new stable lines
        for new_line in new_stable_lines:
            # Find matching existing line
            matched = False
            for existing_line in stable_lines:
                if self.are_lines_similar(new_line['properties'], 
                                        existing_line['properties']):
                    # Update existing line with new detection
                    existing_line.update(new_line)
                    existing_line['age'] = 0
                    matched = True
                    break
            
            if not matched:
                # Add new stable line
                new_line['age'] = 0
                new_line['id'] = self.radar_config['line_id_counter']
                self.radar_config['line_id_counter'] += 1
                stable_lines.append(new_line)

    def publish_scan_points(self, scan_points):
        """发布雷达的扫描点"""
        marker = Marker()
        marker.header.frame_id = self.radar_config['frame_id']
        marker.header.stamp = rospy.Time.now()
        marker.ns = "main_scan_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        
        # 设置颜色：主雷达绿色
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.6
        
        # 添加点
        if len(scan_points[0]) > 0:
            for x, y in zip(scan_points[0], scan_points[1]):
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0
                marker.points.append(point)
        
        self.scan_points_pub.publish(marker)

    def publish_current_lines(self, current_lines):
        """发布当前检测到的线（即使不稳定）"""
        marker_array = MarkerArray()
        
        # Clear previous markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for i, line in enumerate(current_lines):
            # Main line marker
            line_marker = Marker()
            line_marker.header.frame_id = self.radar_config['frame_id']
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "main_current_lines"
            line_marker.id = i
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            line_marker.scale.x = 0.06  # 稍细的线条表示不稳定
            
            # 使用橙色表示不稳定状态
            line_marker.color.r = 1.0
            line_marker.color.g = 0.5
            line_marker.color.b = 0.0  # 橙色
            
            line_marker.color.a = 0.6  # 半透明
            
            # Add points
            start_point = Point()
            start_point.x = line['properties']['start_point'][0]
            start_point.y = line['properties']['start_point'][1]
            start_point.z = 0.0
            
            end_point = Point()
            end_point.x = line['properties']['end_point'][0]
            end_point.y = line['properties']['end_point'][1]
            end_point.z = 0.0
            
            line_marker.points = [start_point, end_point]
            marker_array.markers.append(line_marker)
        
        self.marker_pub.publish(marker_array)

    def publish_stable_lines(self):
        """Publish stable lines for main radar"""
        stable_lines = self.radar_config['stable_lines']
        
        marker_array = MarkerArray()
        
        # Clear previous markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for i, line in enumerate(stable_lines):
            # Main line marker with stability-based coloring
            line_marker = Marker()
            line_marker.header.frame_id = self.radar_config['frame_id']
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "main_stable_lines"
            line_marker.id = line.get('id', i)
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            # Color based on stability score
            stability_score = line.get('stability_score', 1)
            max_stability = self.temporal_buffer_size
            
            line_marker.scale.x = 0.08
            
            # 主雷达红色系
            line_marker.color.r = 1.0
            line_marker.color.g = min(1.0, stability_score / max_stability)
            line_marker.color.b = 0.0
            
            line_marker.color.a = 0.8 + 0.2 * (stability_score / max_stability)
            
            # Add points
            start_point = Point()
            start_point.x = line['properties']['start_point'][0]
            start_point.y = line['properties']['start_point'][1]
            start_point.z = 0.0
            
            end_point = Point()
            end_point.x = line['properties']['end_point'][0]
            end_point.y = line['properties']['end_point'][1]
            end_point.z = 0.0
            
            line_marker.points = [start_point, end_point]
            marker_array.markers.append(line_marker)
            
            # Text marker with enhanced information
            text_marker = Marker()
            text_marker.header = line_marker.header
            text_marker.ns = "main_line_info"
            text_marker.id = line.get('id', i)
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = line['properties']['midpoint'][0]
            text_marker.pose.position.y = line['properties']['midpoint'][1]
            text_marker.pose.position.z = 0.3
            
            text_marker.scale.z = 0.15
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            # Enhanced text with stability info
            text_marker.text = (f"M{line.get('id', i)}: "
                              f"{line['properties']['length']:.2f}m, "
                              f"{line['properties']['angle_deg']:.1f}°, "
                              f"S:{stability_score}")
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)

    def process_single_radar(self, scan_msg):
        """处理雷达的数据"""
        try:
            # 提取和预处理扫描数据
            ranges = np.array(scan_msg.ranges)
            angles = np.arange(scan_msg.angle_min, 
                              scan_msg.angle_max + scan_msg.angle_increment, 
                              scan_msg.angle_increment)
            
            # 确保ranges和angles长度一致
            min_len = min(len(ranges), len(angles))
            ranges = ranges[:min_len]
            angles = angles[:min_len]
            
            # 过滤有效数据
            valid_mask = ((ranges >= scan_msg.range_min) & 
                         (ranges <= scan_msg.range_max) & 
                         np.isfinite(ranges))
            ranges_valid = ranges[valid_mask]
            angles_valid = angles[valid_mask]
            
            if len(ranges_valid) == 0:
                print("No valid points for main radar")
                return
            
            # 转换为笛卡尔坐标（雷达本地坐标系）
            x_coords, y_coords = self.polar_to_cartesian(ranges_valid, angles_valid)
            scan_points = (x_coords, y_coords)
            
            # 创建占用栅格图像
            occupancy_img, _ = self.create_enhanced_occupancy_image(scan_points)
            
            # 多尺度线检测
            lines, edges = self.detect_lines_multi_scale(occupancy_img)
            
            if lines:
                # 计算所有检测到的线的属性
                current_lines = []
                for line in lines:
                    # 处理不同格式的线数据
                    if isinstance(line, np.ndarray) and line.ndim > 1:
                        x1, y1, x2, y2 = line[0]
                    else:
                        x1, y1, x2, y2 = line
                    
                    # 确保坐标是整数
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    
                    # 跳过无效线
                    if x1 == x2 and y1 == y2:
                        continue
                        
                    props = self.calculate_line_properties(x1, y1, x2, y2)
                    
                    # 基本长度检查
                    if (props['length'] >= self.min_line_length_meters and 
                        props['length'] <= self.max_line_length_meters):
                        
                        # 角度检查：过滤掉90度左右的线（垂直线）
                        angle = props['angle_deg']
                        current_lines.append({
                                'image_coords': (x1, y1, x2, y2),
                                'properties': props
                            })
                        '''
                        if not (angle >= (90 - self.angle_tolerance_deg) and
                                angle <= (90 + self.angle_tolerance_deg)):
                            current_lines.append({
                                'image_coords': (x1, y1, x2, y2),
                                'properties': props
                            })
                        '''
                
                
                # 聚类相似线条
                clusters = self.cluster_similar_lines(current_lines)
                merged_lines = []
                
                for cluster in clusters:
                    try:
                        merged_line = self.merge_line_cluster(cluster)
                        if merged_line is not None:
                            merged_lines.append(merged_line)
                    except Exception as e:
                        # 回退到簇中最长的线
                        longest = max(cluster, key=lambda x: x['properties']['length'])
                        merged_lines.append(longest)
                
                # 时间跟踪以获得稳定性
                stable_lines = self.track_lines_temporally(merged_lines)
                self.update_stable_lines(stable_lines)
                
                # 调试输出
                if self.debug_mode:
                    rospy.loginfo(f"Main radar: {len(merged_lines)} lines detected, {len(stable_lines)} stable")
                
                # 发布结果
                self.publish_scan_points(scan_points)
                self.publish_stable_lines()
                
                # 临时发布当前检测到的线（即使不稳定）
                if len(merged_lines) > 0:
                    self.publish_current_lines(merged_lines)
                
                # 发布稳定线信息
                if len(self.radar_config['stable_lines']) > 0:
                    self.publish_debug_line_info(self.radar_config['stable_lines'])
                
                if self.debug_mode: 
                    self.save_debug_images(occupancy_img, edges, merged_lines)

            else:
                if self.debug_mode:
                    print("No lines detected in main radar data")
                self.publish_scan_points(scan_points)
            
        except Exception as e:
            rospy.logerr(f"Error in main radar processing: {e}")
            import traceback
            traceback.print_exc()

    def save_debug_images(self, occupancy_img, edges, lines):
        """保存调试图像显示检测过程"""
        # 创建可视化图像
        vis_img = cv2.cvtColor(occupancy_img, cv2.COLOR_GRAY2BGR)
        
        # 绘制检测到的线条
        for line in lines:
            x1, y1, x2, y2 = line['image_coords']
            cv2.line(vis_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            
            # 绘制起点和终点
            cv2.circle(vis_img, (int(x1), int(y1)), 3, (0, 255, 0), -1)
            cv2.circle(vis_img, (int(x2), int(y2)), 3, (255, 0, 0), -1)
        
        # 添加雷达位置标记（在图像坐标系中显示）
        center = self.image_size // 2
        
        # 雷达原点（相对于雷达本身的位置）
        cv2.circle(vis_img, (center, center), 8, (255, 255, 255), -1)
        cv2.putText(vis_img, 'M', (center-8, center+8), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        # 添加坐标轴
        # X轴（红色）
        cv2.arrowedLine(vis_img, (center, center), (center + 50, center), (0, 0, 255), 2)
        cv2.putText(vis_img, 'X', (center + 55, center + 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # Y轴（绿色）
        cv2.arrowedLine(vis_img, (center, center), (center, center - 50), (0, 255, 0), 2)
        cv2.putText(vis_img, 'Y', (center + 5, center - 55), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # 添加标题和信息
        title = f"Main Radar - Lines: {len(lines)}"
        cv2.putText(vis_img, title, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # 显示稳定线数量
        stable_count = len(self.radar_config['stable_lines'])
        info_text = f"Stable: {stable_count}"
        cv2.putText(vis_img, info_text, (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 显示处理范围信息
        range_info = f"Processing radius: {self.processing_radius_meters:.1f}m ({self.processing_radius_pixels}px)"
        cv2.putText(vis_img, range_info, (10, 120), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # 在图像上绘制处理范围边界
        cv2.circle(vis_img, (center, center), self.processing_radius_pixels, (0, 255, 0), 2)
        
        # 可选：保存图像（如果不需要可以注释掉）
        # cv2.imwrite(f'/tmp/main_radar_occupancy.png', occupancy_img)
        # cv2.imwrite(f'/tmp/main_radar_edges.png', edges)
        # cv2.imwrite(f'/tmp/main_radar_lines.png', vis_img)
        
        # 显示图像
        window_name = "Main Radar Detected Lines"
        cv2.imshow(window_name, vis_img)
        cv2.waitKey(1)

    def get_radar_status(self):
        """获取雷达状态信息"""
        with self.data_lock:
            data_status = "OK" if self.radar_config['data'] is not None else "NO_DATA"
            timestamp = self.radar_config['timestamp']
            stable_lines = len(self.radar_config['stable_lines'])
            
            age_info = "FRESH"
            if timestamp is not None:
                age = (rospy.Time.now() - timestamp).to_sec()
                if age > 1.0:
                    age_info = f"OLD({age:.1f}s)"
            else:
                age_info = "NO_TIME"
            
            return {
                'data': data_status,
                'age': age_info,
                'stable_lines': stable_lines
            }

    def publish_debug_line_info(self, st_line):
        """/main_radar/line_detection_info"""
        line_detection_array_msg = LineDetectionArray()
        line_detection_array_msg.header.stamp = rospy.Time.now()
        line_detection_array_msg.header.frame_id = self.radar_config['frame_id']

        for i, line in enumerate(st_line):
            props = line['properties']
            start = props['start_point']
            end = props['end_point']

            # 创建 LineInfo 消息
            line_info = LineInfo()
            line_info.id = line.get('id', i)

            # 填充起点
            line_info.start_point.x = start[0]
            line_info.start_point.y = start[1]
            line_info.start_point.z = 0.0 # 假设是2D雷达，Z为0

            # 填充终点
            line_info.end_point.x = end[0]
            line_info.end_point.y = end[1]
            line_info.end_point.z = 0.0

            # 填充长度和角度
            line_info.length = props['length']
            line_info.angle_deg = props['angle_deg']
            line_info.distance = props['distance_from_origin']

            line_detection_array_msg.lines.append(line_info)

        
        self.line_info_pub.publish(line_detection_array_msg)

    def run(self):
        """主运行循环"""
        rospy.loginfo("Main Radar Line Detector started")
        rospy.loginfo("Processing main radar...")
        
        # 定期状态检查
        rate = rospy.Rate(0.5)  # 0.5Hz，每2秒检查一次
        status_counter = 0
        while not rospy.is_shutdown():
            status_counter += 1
            rate.sleep()
            
if __name__ == '__main__':
    try:
        detector = SeparateRadarLineDetector()
        detector.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass