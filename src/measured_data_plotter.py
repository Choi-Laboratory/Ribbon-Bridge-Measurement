#!/usr/bin/env python
# coding: utf-8
"""
マーカーの真値と計測値を描画する関数
"""
import rospy, rospkg
import cv2
import numpy as np
import math

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from arcode_ros.msg import MarkerInfo, MarkersInfo
from ribbon_bridge_measurement.msg import RibbonBridge, RibbonBridges

PACKAGE_PATH = rospkg.RosPack().get_path("ribbon_bridge_measurement") + "/"

class MeasuredDataPlotter():
    def __init__(self):
        rospy.init_node("measured_data_plotter", anonymous=False)

        self.ribbon_bridge_num = 7

        self.marker_subscriber = rospy.Subscriber("/arcode_ros/detect_result", MarkersInfo, self.marker_subscriber_callback)
        self.measured_data_subscriber = rospy.Subscriber("/ribbon_bridge_measurement/measure_result", RibbonBridges, self.measured_data_subscriber_callback)
        self.img_subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_subscriber_callback)

        self.img_width = -1 
        self.img_height = -1
        self.is_get_img_size = False

        self.markers_info_list = []
        self.measure_data_list = []

    def marker_subscriber_callback(self, msg):
        markers_info = [MarkerInfo()]*self.ribbon_bridge_num
        
        for marker_info in msg.markers_info:
            markers_info[marker_info.value] = marker_info
        
        self.markers_info_list.append(markers_info)
    
    def measured_data_subscriber_callback(self, msg):
        self.measure_data_list.append(msg.RibbonBridges)

        

    def img_subscriber_callback(self, msg):
        if self.is_get_img_size == True:
            pass
        
        else:
            try:
                cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
                self.img_height, self.img_width, _ = cv_img.shape[:3]
                self.is_get_img_size = True

            except CvBridgeError, e:
                rospy.logerr(e)

    def create_plot_img(self):
        data_list = self.markers_info_list

        white_img = np.ones((self.img_height, self.img_width, 3),np.uint8)*255
        
        for i, markers_data in enumerate(data_list):
            for id, marker_data in enumerate(markers_data):
                if marker_data != MarkerInfo(): #マーカの検出に成功した場合
                    #マーカーの傾きを計算
                    radian = math.atan2(marker_data.right_top.y - marker_data.left_top.y, marker_data.right_top.x - marker_data.left_top.x)

                    #中心点の描画
                    cv2.circle(white_img, \
                            (int(marker_data.center.x), int(marker_data.center.y)),\
                            radius=1, \
                            color=(0,id*20,255-id*20), \
                            thickness=-1, \
                            lineType=cv2.LINE_8, \
                            shift=0)
                    
                    #一つ前のフレームのマーカーと現在のマーカーを結ぶ線を描画
                    if i != 0:
                        counter = 1
                        while True:
                            last_marker_data = data_list[i-counter][id]
                            if last_marker_data != MarkerInfo():
                                break
                            else:
                                counter += 1

                        cv2.arrowedLine(white_img, \
                                (int(last_marker_data.center.x), int(last_marker_data.center.y)),\
                                (int(marker_data.center.x), int(marker_data.center.y)),\
                                color=(0,id*20,255-id*20),\
                                thickness=1,\
                                tipLength=0.25,\
                                shift=0)
                    
                    
                    #一番新しい点は、角度の軸も描画する
                    if i == len(data_list)-1:
                        cv2.line(white_img, \
                                (int(marker_data.center.x), int(marker_data.center.y)),\
                                (int((marker_data.right_top.x+marker_data.right_bottom.x)/2), int((marker_data.right_top.y+marker_data.right_bottom.y)/2)),\
                                color=(0,0,255),\
                                thickness=3,\
                                lineType=cv2.LINE_8,\
                                shift=0)
                        
                        cv2.line(white_img, \
                                (int(marker_data.center.x), int(marker_data.center.y)),\
                                (int((marker_data.right_top.x+marker_data.left_top.x)/2), int((marker_data.right_top.y+marker_data.left_top.y)/2)),\
                                color=(255,0,0),\
                                thickness=3,\
                                lineType=cv2.LINE_8,\
                                shift=0)


        cv2.imshow("Marker plot", white_img)
        cv2.waitKey(1) 

    def main(self):
        while not rospy.is_shutdown():
            try:
                if self.is_get_img_size == True:
                    #self.create_plot_img()
                    pass
            
            except KeyboardInterrupt:
                break


if __name__ == "__main__":
    mdp = MeasuredDataPlotter()
    mdp.main()