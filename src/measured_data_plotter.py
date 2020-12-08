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

from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from ribbon_bridge_measurement.msg import RibbonBridge, RibbonBridges

PACKAGE_PATH = rospkg.RosPack().get_path("ribbon_bridge_measurement") + "/"

class MeasuredDataPlotter():
    def __init__(self):
        rospy.init_node("measured_data_plotter", anonymous=False)

        self.ribbon_bridge_num = 7

        self.marker_subscriber = rospy.Subscriber("/arcode_ros/detect_result", BoundingBoxArray, self.marker_subscriber_callback)
        self.measured_data_subscriber = rospy.Subscriber("/ribbon_bridge_measurement/measure_result", RibbonBridges, self.measured_data_subscriber_callback)
        self.img_subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_subscriber_callback)

        self.img_width = -1 
        self.img_height = -1
        self.is_get_img_size = False

        self.markers_data = []

    def marker_subscriber_callback(self, msg):
        bounding_boxes = [BoundingBox()]*self.ribbon_bridge_num
        
        for bounding_box in msg.boxes:
            bounding_boxes[bounding_box.label] = bounding_box
        
        self.markers_data.append(bounding_boxes)
    
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
        markers_data = self.markers_data

        white_img = np.ones((self.img_height, self.img_width, 3),np.uint8)*255
        
        for marker_data in markers_data:
            for i, data in enumerate(marker_data):
                cv2.circle(white_img, \
                        (int(data.pose.position.x), int(data.pose.position.y)),\
                        radius=3, \
                        color=(0,i*20,255-i*20), \
                        thickness=-1, \
                        lineType=cv2.LINE_8, \
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