#! /usr/bin/env python
# coding:utf-8
import rospy, rosparam

from std_msgs.msg import String
from jsk_recognition_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes as darknet_ros_BoundingBoxes
from multi_tracker_ros_msgs.msg import RegionOfInterest, RegionOfInterests
from ribbon_bridge_measurement.srv import DetectRibbonBridges, DetectRibbonBridgesResponse 


"""
浮体の個数を送ると、YOLOの検出結果を返すservice
このノードの裏で、darknet_ros ribbon_bridge_sim.launchを動かす必要があります。
検出結果をそのまま追跡領域としてpublishします

input: 
    - 検出したい浮き橋の数

output:
    - 検出したい浮き橋の数と実際の検出数が一致しているかどうか
    - 追跡領域の配列
"""

class RibbonBridgeDetectServer():
    def __init__(self):
        rospy.loginfo("[RibbonBridgeDetectServer] Server is initialized")

        #Publishers
        self.object_detection_control_publisher = rospy.Publisher("/darknet_ros/detection_control", String, queue_size=1, latch=True)
        self.region_of_interest_publisher = rospy.Publisher("/multi_tracker_ros/region_of_interest", RegionOfInterest, queue_size=10, latch=False)
        self.region_of_interests_publisher = rospy.Publisher("/multi_tracker_ros/region_of_interests", RegionOfInterests, queue_size=10, latch=False)

        #Subscribers
        self.object_detection_subscriber = rospy.Subscriber("/darknet_ros/bounding_boxes", darknet_ros_BoundingBoxes, self.object_detection_subscriber_callback)
        
        #検出した浮体のbounding_boxesを格納する
        self.bounding_boxes = []
        self.is_update_bounding_boxes = False #検出したらTrue

        #ノード起動直後にdarknet_rosの推論を止める
        self.object_detection_control_publisher.publish("False")

        #serverの起動
        rospy.Service("/ribbon_bridge_detect_server", DetectRibbonBridges, self.service_callback)

        #YOLOの結果より広めの領域を設定したい場合のマージン
        self.margin = 30 #pixel



    def service_callback(self, srv):
        rospy.loginfo("[RibbonBridgeDetectServer] Server is called")

        res = DetectRibbonBridgesResponse()

        if self.object_detection_subscriber.get_num_connections() == 0:
            res.is_find_all_ribbon_bridges = False
            res.region_of_interests = []
            rospy.logwarn("[RibbonBridgeDetectServer] Maybe darknet_ros node is not running")

        else:
            self.bounding_boxes = []
            self.is_update_bounding_boxes = False

            self.object_detection_control_publisher.publish("True") #推論開始

            #物体検出の結果が返ってくるまで待機
            while self.is_update_bounding_boxes != True:
                rospy.logdebug("[RibbonBridgeDetectServer] Waiting for darknet_ros BoundingBoxes")
                rospy.sleep(0.5) #sleep for 0.5 seconds
            
            self.object_detection_control_publisher.publish("False") #推論停止

            bounding_boxes = self.bounding_boxes

            #指定した領域の個数と検出された領域の個数が一致しているか確認
            if len(bounding_boxes) == srv.ribbon_bridges_num:
                res.is_find_all_ribbon_bridges = True
            else:
                res.is_find_all_ribbon_bridges = False
                rospy.logwarn("[RibbonBridgeDetectServer] The specified number and the detected number do not match.")
            
            #各bounding_boxをregion_of_interst型に変換
            for i, bounding_box in enumerate(bounding_boxes):
                region_of_interest = RegionOfInterest()
                region_of_interest.ID = i
                region_of_interest.boundingBox.header.frame_id = str(i)
                region_of_interest.boundingBox.label = i
                region_of_interest.boundingBox.value = bounding_box.probability
                region_of_interest.boundingBox.pose.position.x = bounding_box.xmin - self.margin
                region_of_interest.boundingBox.pose.position.y = bounding_box.ymin - self.margin
                region_of_interest.boundingBox.dimensions.x = bounding_box.xmax - bounding_box.xmin + self.margin*2
                region_of_interest.boundingBox.dimensions.y = bounding_box.ymax - bounding_box.ymin + self.margin*2

                res.region_of_interests.append(region_of_interest)

                #検出した領域を追跡ノードにpublish
                self.region_of_interest_publisher.publish(region_of_interest)

        #region_of_interests = RegionOfInterests()
        #region_of_interests.region_of_interests = res.region_of_interests
        #self.region_of_interests_publisher.publish(region_of_interests)

        rospy.loginfo("[RibbonBridgeDetectServer] Server responses client")
        return res

    def object_detection_subscriber_callback(self, msg):
        self.bounding_boxes = msg.bounding_boxes
        self.is_update_bounding_boxes = True


if __name__ == "__main__":
    rospy.init_node("ribbon_bridge_detect_service", anonymous=False)

    ribbon_bridge_detect_server = RibbonBridgeDetectServer()

    rospy.spin()