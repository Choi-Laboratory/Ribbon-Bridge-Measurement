#!/usr/bin/env python
# coding: utf-8
import rospy
import math
from ribbon_bridge_measurement.msg import RibbonBridge, RibbonBridges
from geometry_msgs.msg import *
from armarker_ros.msg import MarkerInfo, MarkersInfo
import numpy as np
import cv2

first_unit_data_list = []
second_unit_data_list = []
third_unit_data_list = []

angle_error_list_base = []
angle_error_list_first = []
angle_error_list_second = []
angle_error_list_third = []

data_counter = 0.0000000

pixel_length = 0.12985948

def get_distance(center1, center2):
    x1 = center1.x
    y1 = center1.y
    x2 = center2.x
    y2 = center2.y
    return math.sqrt(pow(x1-x2,2)+pow(y1-y2,2))


def measurement_data_callback_(msg):
    global first_unit_data_list
    global second_unit_data_list
    global third_unit_data_list
    global data_counter

    if len(msg.RibbonBridges) == 4:
        data_counter += 1.0000000
        distance_0_to_1 = get_distance(msg.RibbonBridges[0].center, msg.RibbonBridges[1].center)
        distance_0_to_2 = get_distance(msg.RibbonBridges[0].center, msg.RibbonBridges[2].center)
        distance_0_to_3 = get_distance(msg.RibbonBridges[0].center, msg.RibbonBridges[3].center)

        length_0_to_1 = distance_0_to_1 * pixel_length
        length_0_to_2 = distance_0_to_2 * pixel_length
        length_0_to_3 = distance_0_to_3 * pixel_length

        error_0_to_1 = abs(length_0_to_1 - 12.2000)
        error_0_to_2 = abs(length_0_to_2 - 12.2000*2.00)
        error_0_to_3 = abs(length_0_to_3 - 12.2000*3.00)

        error_rate_0_to_1 = error_0_to_1 / 12.2000
        error_rate_0_to_2 = error_0_to_2 / 12.2000 * 2.00
        error_rate_0_to_3 = error_0_to_3 / 12.2000 * 3.00

        first_unit_data_list.append(error_rate_0_to_1)
        second_unit_data_list.append(error_rate_0_to_2)
        third_unit_data_list.append(error_rate_0_to_3)

        average_1 = sum(first_unit_data_list)/data_counter
        average_2 = sum(second_unit_data_list)/data_counter
        average_3 = sum(third_unit_data_list)/data_counter

        hull_average = (average_1+average_2+average_3)/3.00000

        angle_error_base = 90.000000 - msg.RibbonBridges[0].center.theta
        angle_error_first = 90.000000 - msg.RibbonBridges[1].center.theta
        angle_error_second = 90.000000 - msg.RibbonBridges[2].center.theta
        angle_error_third = 90.000000 - msg.RibbonBridges[3].center.theta

        angle_error_list_base.append(angle_error_base)
        angle_error_list_first.append(angle_error_first)
        angle_error_list_second.append(angle_error_second)
        angle_error_list_third.append(angle_error_third)

        average_angle_error_0 = sum(angle_error_list_base)/data_counter
        average_angle_error_1 = sum(angle_error_list_first)/data_counter
        average_angle_error_2 = sum(angle_error_list_second)/data_counter
        average_angle_error_3 = sum(angle_error_list_third)/data_counter

        hull_angle_average = (average_angle_error_0+average_angle_error_1+average_angle_error_2+average_angle_error_3)/4.000000000


        print "データ数："+str(int(data_counter))
        print "ユニット1の誤差率:" + str(average_1)
        print "ユニット2の誤差率:" + str(average_2)
        print "ユニット3の誤差率:" + str(average_3)
        print "全体の誤差率：" + str(hull_average)
        print "---"
        print "ユニット0の角度誤差:" + str(average_angle_error_0)
        print "ユニット1の角度誤差:" + str(average_angle_error_1)
        print "ユニット2の角度誤差:" + str(average_angle_error_2)
        print "ユニット3の角度誤差:" + str(average_angle_error_3)
        print "全体の角度誤差：" + str(hull_angle_average)
        print "------------------"


bridge_0_position = []
bridge_1_position = []
bridge_2_position = []
bridge_3_position = []

"""str_ = '\n'.join(list_)
    with open("text_write_str.txt", 'wt') as f:
        f.write(str_)
"""

img_width = 960
img_height = 720
blank = np.zeros((img_height, img_width, 3))
blank += [255,255,255]

def measurement_data_callback(msg):
    if len(msg.RibbonBridges) != 0:
    #if len(msg.RibbonBridges) == 4:
        for i in range(len(msg.RibbonBridges)):
            boat_id = msg.RibbonBridges[i].boat_id
            if(boat_id==0):
                bridge_0_position.append(msg.RibbonBridges[i].center)
                cv2.circle(blank,(int(msg.RibbonBridges[i].center.x), int(msg.RibbonBridges[i].center.y)), 5, (255,0,0), -1)

            elif(boat_id==1):
                bridge_1_position.append(msg.RibbonBridges[i].center)
                cv2.circle(blank,(int(msg.RibbonBridges[i].center.x), int(msg.RibbonBridges[i].center.y)), 5, (0,255,0), -1)

            elif(boat_id==2):
                bridge_2_position.append(msg.RibbonBridges[i].center)
                cv2.circle(blank,(int(msg.RibbonBridges[i].center.x), int(msg.RibbonBridges[i].center.y)), 5, (0,0,255), -1)

            elif(boat_id==3):
                bridge_3_position.append(msg.RibbonBridges[i].center)
                cv2.circle(blank,(int(msg.RibbonBridges[i].center.x), int(msg.RibbonBridges[i].center.y)), 5, (255,0,255), -1)


def armarker_data_callback(msg):
    for i in range(len(msg.markers_info)):
        if(msg.markers_info[i].ID==0):
            bridge_0_position.append(msg.markers_info[i].center)
            cv2.circle(blank,(int(msg.markers_info[i].center.x), int(msg.markers_info[i].center.y)), 5, (255,0,0), -1)

        elif(msg.markers_info[i].ID==1):
            bridge_1_position.append(msg.markers_info[i].center)
            cv2.circle(blank,(int(msg.markers_info[i].center.x), int(msg.markers_info[i].center.y)), 5, (0,255,0), -1)

        elif(msg.markers_info[i].ID==2):
            bridge_2_position.append(msg.markers_info[i].center)
            cv2.circle(blank,(int(msg.markers_info[i].center.x), int(msg.markers_info[i].center.y)), 5, (0,0,255), -1)

        elif(msg.markers_info[i].ID==3):
            bridge_3_position.append(msg.markers_info[i].center)
            cv2.circle(blank,(int(msg.markers_info[i].center.x), int(msg.markers_info[i].center.y)), 5, (255,0,255), -1)








if __name__ == "__main__":
    rospy.init_node("collect_data_node")

    measurement_data_subscriber = rospy.Subscriber("/ribbon_bridge_measurement/measure_result", RibbonBridges, measurement_data_callback)

    #armarker_data_subscriber = rospy.Subscriber("/armarker_ros/markers_info", MarkersInfo, armarker_data_callback)

    while not rospy.is_shutdown():
        #print len(bridge_0_position),len(bridge_1_position),len(bridge_2_position),len(bridge_3_position)
        cv2.imshow("result", blank)
        cv2.imwrite("/home/rg26/Desktop/measured_result2_.jpg", blank)
        #cv2.imwrite("/home/rg26/Desktop/marker_result3.jpg", blank)
        cv2.waitKey(1)

    rospy.spin()
