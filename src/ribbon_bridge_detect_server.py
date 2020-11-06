#! /usr/bin/env python
# coding:utf-8
import rospy, rosparam

from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from multi_tracker_ros_msgs.msg import RegionOfInterest

"""
浮体の個数を送ると、YOLOの検出結果を返すservice
このノードの裏で、darknet_ros ribbon_bridge_sim.launchを動かす必要があります。
"""

class 


if __name__ == "__main__":
    rospy.init_node("ribbon_bridge_detect_service", anonymous=False)