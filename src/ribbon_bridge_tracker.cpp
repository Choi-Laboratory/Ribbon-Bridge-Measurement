#include <iostream>
#include <time.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <jsk_rviz_plugins/OverlayText.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <ribbon_bridge_measurement/RibbonBridge.h>
#include <ribbon_bridge_measurement/RibbonBridges.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#define PI 3.141592653589793
typedef ribbon_bridge_measurement::RibbonBridge RibbonBridge;
typedef ribbon_bridge_measurement::RibbonBridges RibbonBridges;
typedef std::pair<std_msgs::Header, RibbonBridge> RibbonBridgeStamped;


class Ribbon_Bridge_Tracker{

public:

    Ribbon_Bridge_Tracker(){

        ros::param::get("sub_ctrl_topic_name", this->contrl_topic_name);
        ros::param::get("sub_image_topic_name", this->image_topic_name);
        ros::param::get("sub_ribbon_bridges_topic_name", this->ribbon_bridges_topic_name);

        this->counter = 0;
        this->sub_camera_img = nh.subscribe(image_topic_name, 1, &Ribbon_Bridge_Tracker::rgbImageCallback, this);
        this->sub_ribbon_bridges = nh.subscribe(this->ribbon_bridges_topic_name, 1, &Ribbon_Bridge_Tracker::ribbonBridgesCallback, this);

    }//Ribbon_Bridge_Tracker()


    void rgbImageCallback(const sensor_msgs::Image msg){
        this->image_msg = msg;
    }


    void ribbonBridgesCallback(const ribbon_bridge_measurement::RibbonBridges msg){
        this->ribbon_bridges_data = msg;
    }


    void initialize_id(){

        if(this->ribbon_bridges_data.RibbonBridges.empty() == true){return;}

        //浮橋のx座標が小さい順にIDを割り当てる．
        std::vector<RibbonBridge> sorted_ribbon_bridges;

        for(int i = 0; i < this->ribbon_bridges_data.RibbonBridges.size(); i++){
            int insert_point = this->calc_insert_point(sorted_ribbon_bridges, this->ribbon_bridges_data.RibbonBridges[i]);
            sorted_ribbon_bridges.insert(sorted_ribbon_bridges.begin()+insert_point, this->ribbon_bridges_data.RibbonBridges[i]);
        }//for

        this->tracking_ribbon_bridge.clear();
        for(int id = 0; id < sorted_ribbon_bridges.size(); id++){
            sorted_ribbon_bridges[id].boat_id = id;
            RibbonBridgeStamped temp_data = std::make_pair(this->ribbon_bridges_data.header, sorted_ribbon_bridges[id]);
            this->tracking_ribbon_bridge.push_back(temp_data);
        }
    }//initialize_id


	void update(){
        if(this->ribbon_bridges_data.RibbonBridges.empty() == true){return;}

        //浮橋のx座標が小さい順にIDを割り当てる．
        std::vector<RibbonBridge> sorted_ribbon_bridges;

        for(int i = 0; i < this->ribbon_bridges_data.RibbonBridges.size(); i++){
            int insert_point = this->calc_insert_point(sorted_ribbon_bridges, this->ribbon_bridges_data.RibbonBridges[i]);
            sorted_ribbon_bridges.insert(sorted_ribbon_bridges.begin()+insert_point, this->ribbon_bridges_data.RibbonBridges[i]);
        }//for

	}//update


    int calc_insert_point(std::vector<RibbonBridge> target_vec, RibbonBridge source){
        int insert_point = 0;
        for(int pt = 0; pt < target_vec.size(); pt++){
            if(target_vec[pt].center.x > source.center.x){ break; }
            insert_point++;
        }
        return insert_point;
    }//calc_insert_point

private:

    ros::NodeHandle nh;
    ros::Subscriber sub_camera_img;
    ros::Subscriber sub_ctrl_flag;
    ros::Subscriber sub_ribbon_bridges;
    ros::Publisher pub_result_img;
    ros::Publisher pub_overlay_text;
    ros::Publisher pub_ribbon_bridges_msg;

    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::Image image_msg;
    cv::Mat image;

    std::string image_topic_name;
    std::string contrl_topic_name;
    std::string ribbon_bridges_topic_name;
    int counter;

    ribbon_bridge_measurement::RibbonBridges ribbon_bridges_data;
    std::vector<std::pair<std_msgs::Header, RibbonBridge>> tracking_ribbon_bridge;

};//Ribbon_Bridge_Tracker


int main(int argc, char** argv) {

    ros::init(argc, argv, "ribbon_bridges_tracker_node");
    Ribbon_Bridge_Tracker rbt;

    while(true){
        rbt.update();
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return 0;
}//main
