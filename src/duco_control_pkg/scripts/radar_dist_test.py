#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from triple_radar_front_distance import TripleRadarFrontDistance


def query_distance(triple_radar, radar, direction):
    """
    调用 TripleRadarFrontDistance 的 get_distance，
    读取指定雷达、指定方向的距离数据。

    Args:
        triple_radar: TripleRadarFrontDistance 实例
        radar (str): 雷达名称，支持 "left" / "right" / "main" / "height"
        direction (str): 方向名称，支持 "front" / "up" / "down"

    Returns:
        float: 距离值（米），-1 表示无效或超时
    """
    return triple_radar.get_distance(radar, direction)


def main():
    rospy.init_node("radar_dist_test")

    # 创建 TripleRadarFrontDistance 实例
    triple_radar = TripleRadarFrontDistance()

    rate = rospy.Rate(2)  # 2Hz

    # 你可以在这里根据需要修改要测试的雷达和方向
    test_radars = ["left", "right"]
    test_directions = ["front", "up", "down"]

    rospy.loginfo("radar_dist_test 节点启动，开始周期性读取距离数据")

    while not rospy.is_shutdown():
        rospy.loginfo("===========")
        for radar in test_radars:
            for direction in test_directions:
                dist = query_distance(triple_radar, radar, direction)
                if dist > 0:
                    rospy.loginfo(f"radar={radar}, direction={direction}, distance={dist:.3f} m")
                else:
                    rospy.loginfo(f"radar={radar}, direction={direction}, distance=无效")
        rospy.loginfo("===========")
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
