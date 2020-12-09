#!/usr/bin/env python
# coding: utf-8
"""
マーカーの真値と計測値を描画する関数
"""
import rospy, rospkg
import cv2
import numpy as np

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

        self.data_list = []

    def marker_subscriber_callback(self, msg):
        markers_info = [MarkerInfo()]*self.ribbon_bridge_num
        
        for marker_info in msg.markers_info:
            markers_info[marker_info.value] = marker_info
        
        self.data_list.append(markers_info)
    
    def measured_data_subscriber_callback(self, msg):
        pass
    
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
        data_list = self.data_list

        white_img = np.ones((self.img_height, self.img_width, 3),np.uint8)*255
        
        for markers_data in data_list:
            for i, marker_data in enumerate(markers_data):
                cv2.circle(white_img, \
                        (int(marker_data.center.x), int(marker_data.center.y)),\
                        radius=3, \
                        color=(0,i*20,255-i*20), \
                        thickness=-1, \
                        lineType=cv2.LINE_8, \
                        shift=0)
                
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
                    self.create_plot_img()
            
            except KeyboardInterrupt:
                break


if __name__ == "__main__":
    mdp = MeasuredDataPlotter()
    mdp.main()