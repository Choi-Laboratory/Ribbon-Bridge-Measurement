#!/usr/bin/env python
# coding: utf-8
import rospy
from sensor_msgs.msg import *
from darknet_ros_msgs.msg import *


class dummy_bbox_maker:
	def __init__(self):

		self.pub_yolo_topic_name = "/darknet_ros/bounding_boxes"
		self.sub_image_topic_name = "/aerial_camera/camera1/image_raw"

		self.pub_bbox = rospy.Publisher(self.pub_yolo_topic_name, BoundingBoxes, queue_size=1)
		self.sub_image = rospy.Subscriber(self.sub_image_topic_name, Image, self.image_cb)

	def image_cb(self,msg):

		bbox = BoundingBox()
		bbox.probability = 1.0
		bbox.xmin = 0
		bbox.ymin = 0
		bbox.xmax = msg.width
		bbox.ymax = msg.height

		send_msg = BoundingBoxes()
		send_msg.bounding_boxes.append(bbox)

		self.pub_bbox.publish(send_msg)

if __name__ == '__main__':

	rospy.init_node('dummy_bbox_maker_node')

	obc = dummy_bbox_maker()
	rospy.spin()
