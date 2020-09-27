#include "iostream"
#include "time.h"
#include "stdio.h"
#include "unistd.h"

#include "ros/ros.h"
#include "darknet_dnn/BoundingBox.h"
#include "darknet_dnn/BoundingBoxes.h"
#include "multi_tracker_ros/RegionOfInterests.h"
#include "multi_tracker_ros/TrackingResult.h"
#include "multi_tracker_ros/TrackingResults.h"
#include "ribbon_bridge_measurement/RibbonBridge.h"
#include "ribbon_bridge_measurement/RibbonBridges.h"
#include "geometry_msgs/Pose2D.h"

ros::Publisher regions_of_interests_publisher;

darknet_dnn::BoundingBoxes g_tracking_results;

void callbackTrackingResults(const darknet_dnn::BoundingBoxes& msg){
  g_tracking_results = msg;
}//callbackTrackingResults

void callbackMeasurementResults(const ribbon_bridge_measurement::RibbonBridges& msg){
  if( msg.RibbonBridges.size() != 0 ){
    darknet_dnn::BoundingBoxes tracking_results = g_tracking_results;
    for( int i = 0; i < msg.RibbonBridges.size(); i++ ){
        //std::cout << msg.RibbonBridges[i] << std::endl;
        //for( int j = 0; j < tracking_results.boundingBoxes.size(); j++ ){
          //std::cout << tracking_results.boundingBoxes[j].Class << std::endl;
        //}
        int boat_id = msg.RibbonBridges[i].boat_id;
        geometry_msgs::Pose2D roi_center;
        roi_center.x = tracking_results.boundingBoxes[boat_id].x + (tracking_results.boundingBoxes[boat_id].width/2.0);
        roi_center.y = tracking_results.boundingBoxes[boat_id].y + (tracking_results.boundingBoxes[boat_id].height/2.0);
        float distance = sqrt(pow(msg.RibbonBridges[i].center.x-roi_center.x, 2) + pow(msg.RibbonBridges[i].center.y-roi_center.y, 2));
        //std::cout << distance << std::endl;

        if( 5.0 < distance && distance < 20.0 ){
          multi_tracker_ros::RegionOfInterests roi;
          roi.ID = boat_id;
          /*roi.boundingBox.x = int(msg.RibbonBridges[i].center.x - 75);
          roi.boundingBox.y = int(msg.RibbonBridges[i].center.y - 75);
          roi.boundingBox.width = 150;
          roi.boundingBox.height = 150;
          */
          roi.boundingBox.x = int(msg.RibbonBridges[i].center.x - 100);
          roi.boundingBox.y = int(msg.RibbonBridges[i].center.y - 100);
          roi.boundingBox.width = 200;
          roi.boundingBox.height = 200;
          regions_of_interests_publisher.publish(roi);
          std::cout << "Update ROI of " << boat_id << std::endl;
        }

    }//for
    std::cout << "-----\n";
  }
}//callbackMeasurementResults


int main(int argc, char** argv){
  ros::init(argc, argv, "update_regions_of_interest_node");
  ROS_INFO("update_regions_of_interest START");

  ros::NodeHandle nh;

  regions_of_interests_publisher = nh.advertise<multi_tracker_ros::RegionOfInterests>("multi_tracker_ros/RegionOfInterests", 1);

  ros::Subscriber tracking_results_subscriber = nh.subscribe("/multi_tracker_ros/tracking_results", 10, callbackTrackingResults);

  ros::Subscriber measuement_results_subscriber = nh.subscribe("/ribbon_bridge_measurement/measure_result", 10, callbackMeasurementResults);

  ros::spin();

}//main
