#!/usr/bin/env python
# coding: utf-8
import rospy
import math
from ribbon_bridge_measurement.msg import RibbonBridge, RibbonBridges
from geometry_msgs.msg import *
from armarker_ros.msg import MarkerInfo, MarkersInfo
import numpy as np
import cv2

measured_pose_0 = []
measured_pose_1 = []
measured_pose_2 = []
measured_pose_3 = []

true_pose_0 = []
true_pose_1 = []
true_pose_2 = []
true_pose_3 = []

true_angle_0 = []
true_angle_1 = []
true_angle_2 = []
true_angle_3 = []

pixel_error_0 = []
pixel_error_1 = []
pixel_error_2 = []
pixel_error_3 = []

max_error_0 = 0.0
max_error_1 = 0.0
max_error_2 = 0.0
max_error_3 = 0.0

def get_distance(center1, center2):
    x1 = center1.x
    y1 = center1.y
    x2 = center2.x
    y2 = center2.y
    return math.sqrt(pow(x1-x2,2)+pow(y1-y2,2))


def callback_marker_info(msg):
    for i in range(len(msg.markers_info)):
        if(msg.markers_info[i].ID==0):
            true_pose_0.append(msg.markers_info[i].center)
            true_angle_0.append(msg.markers_info[i].theta)

        elif(msg.markers_info[i].ID==1):
            true_pose_1.append(msg.markers_info[i].center)
            true_angle_1.append(msg.markers_info[i].theta)

        elif(msg.markers_info[i].ID==2):
            true_pose_2.append(msg.markers_info[i].center)
            true_angle_2.append(msg.markers_info[i].theta)

        elif(msg.markers_info[i].ID==3):
            true_pose_3.append(msg.markers_info[i].center)
            true_angle_3.append(msg.markers_info[i].theta)


def callback_measured_result(msg):
    global max_error_0, max_error_1, max_error_2, max_error_3
    #if len(msg.RibbonBridges) != 0:
    if len(msg.RibbonBridges) == 4:
        for i in range(len(msg.RibbonBridges)):
            if msg.RibbonBridges[i].boat_id == 0:
                print msg.RibbonBridges[i].boat_id
                true_pose = true_pose_0[len(true_pose_0)-1]
                measured_pose = msg.RibbonBridges[i].center
                distance = get_distance(measured_pose, true_pose)
                #print "真値[位置]："+str(true_pose.x)+str(true_pose.y)
                #print "計測値[位置]："+str(measured_pose.x)+str(measured_pose.y)
                #print "誤差[位置]:"+str(distance)
                #print "---"

                pixel_error_0.append(distance)
                if max_error_0 < distance:
                    max_error_0 = distance

            elif msg.RibbonBridges[i].boat_id == 1:
                print msg.RibbonBridges[i].boat_id
                true_pose = true_pose_1[len(true_pose_1)-1]
                measured_pose = msg.RibbonBridges[i].center
                distance = get_distance(measured_pose, true_pose)
                #print "真値[位置]："+str(true_pose.x)+str(true_pose.y)
                #print "計測値[位置]："+str(measured_pose.x)+str(measured_pose.y)
                #print "誤差[位置]:"+str(distance)
                #print "---"
                pixel_error_1.append(distance)
                if max_error_1 < distance:
                    max_error_1 = distance

            elif msg.RibbonBridges[i].boat_id == 2:
                print msg.RibbonBridges[i].boat_id
                true_pose = true_pose_2[len(true_pose_2)-1]
                measured_pose = msg.RibbonBridges[i].center
                distance = get_distance(measured_pose, true_pose)
                #print "真値[位置]："+str(true_pose.x)+str(true_pose.y)
                #print "計測値[位置]："+str(measured_pose.x)+str(measured_pose.y)
                #print "誤差[位置]:"+str(distance)
                #print "---"
                pixel_error_2.append(distance)
                if max_error_2 < distance:
                    max_error_2 = distance


            elif msg.RibbonBridges[i].boat_id == 3:
                print msg.RibbonBridges[i].boat_id
                true_pose = true_pose_3[len(true_pose_3)-1]
                measured_pose = msg.RibbonBridges[i].center
                distance = get_distance(measured_pose, true_pose)
                #print "真値[位置]："+str(true_pose.x)+str(true_pose.y)
                #print "計測値[位置]："+str(measured_pose.x)+str(measured_pose.y)
                #print "誤差[位置]:"+str(distance)
                #print "---"
                pixel_error_3.append(distance)
                if max_error_3 < distance:
                    max_error_3 = distance

            #print "==========================================="


if __name__ == "__main__":
    rospy.init_node("collect_data")

    marker_info_sub = rospy.Subscriber("/armarker_ros/markers_info", MarkersInfo, callback_marker_info)

    measured_result_sub = rospy.Subscriber("/ribbon_bridge_measurement/measure_result", RibbonBridges, callback_measured_result)

    while not rospy.is_shutdown():
        if len(pixel_error_0) != 0:
            print "平均誤差[0]:"+str(sum(pixel_error_0)/len(pixel_error_0))
            print "最大誤差[0]:"+str(max_error_0)
            print "---"

        if len(pixel_error_1) != 0:
            print "平均誤差[1]:"+str(sum(pixel_error_1)/len(pixel_error_1))
            print "最大誤差[1]:"+str(max_error_1)
            print "---"

        if len(pixel_error_2) != 0:
            print "平均誤差[2]:"+str(sum(pixel_error_2)/len(pixel_error_2))
            print "最大誤差[2]:"+str(max_error_2)
            print "---"

        if len(pixel_error_3) != 0:
            print "平均誤差[3]:"+str(sum(pixel_error_3)/len(pixel_error_3))
            print "最大誤差[3]:"+str(max_error_3)
            print "---"

        print "=================="


    #rospy.spin()
    #
