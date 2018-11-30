#include <iostream>
#include <time.h>
#include <cstdlib>
#include <sstream>
#include <string>

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


class RibbonBridgeMeasurement{

public:

    RibbonBridgeMeasurement(){

        ros::param::get("show_result_img_flag", show_result_flag);
        ros::param::get("pub_result_img_flag", pub_result_flag);
        ros::param::get("save_result_img_flag", save_result_flag);

        ros::param::get("sub_image_topic_name", image_topic_name);
        ros::param::get("sub_yolo_topic_name", yolo_topic_name);

        ros::param::get("this_package_name", this_package_name);
        ros::param::get("yolo_rect_margin_px", rect_margin);

        this->counter                = 0;
        this->sub_camera_img         = nh.subscribe(image_topic_name, 1, &RibbonBridgeMeasurement::rgbImageCallback, this);
        this->sub_yolo_bbox          = nh.subscribe(yolo_topic_name, 1, &RibbonBridgeMeasurement::yolobboxCallback, this);
        this->pub_result_img         = nh.advertise<sensor_msgs::Image>("result_img", 1);
        this->pub_overlay_text       = nh.advertise<jsk_rviz_plugins::OverlayText>("result_info", 1);
        this->pub_ribbon_bridges_msg = nh.advertise<ribbon_bridge_measurement::RibbonBridges>("ribbon_bridges", 1);

    }//RibbonBridgeMeasurement()


    void rgbImageCallback(const sensor_msgs::Image::ConstPtr& msg){
        try{ this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); }
        catch (cv_bridge::Exception& e){ return; }
        this->color_img = this->cv_ptr->image.clone();
    }//rgbImageCallback


    void yolobboxCallback(const darknet_ros_msgs::BoundingBoxes& msg){

        if(this->color_img.empty() == true){ return; }

		this->target_img = this->color_img.clone();
		ribbon_bridge_measurement::RibbonBridges measuredBridges;//計測結果の格納用

        for(int bbox_count = 0; bbox_count < msg.bounding_boxes.size(); bbox_count++){
            ribbon_bridge_measurement::RibbonBridge bridge_data;
            cv::Rect bridge_rect = this->make_margin_rect(msg.bounding_boxes[bbox_count]);
            bool detect_flag = this->measure_ribbon_bridge(target_img, bridge_rect, bridge_data);
			if(detect_flag == true){ measuredBridges.RibbonBridges.push_back(bridge_data); }
        }//bbox_count

        ROS_INFO_STREAM("--- Detect " << measuredBridges.RibbonBridges.size() << "Bridges. ---");

        if(this->show_result_flag || this->save_result_flag || this->pub_result_flag){ this->make_result_image(measuredBridges); }
        if(this->show_result_flag == true){ this->show_result_image(); }
        if(this->save_result_flag == true){ this->save_result_img(); }
        if(this->pub_result_flag  == true){ this->publish_result_img(); }
        this->pub_ribbon_bridges_msg.publish(measuredBridges);

    }//yolobboxCallback


	cv::Rect make_margin_rect(darknet_ros_msgs::BoundingBox yolo_bbox){
        cv::Rect boat_rect;
        boat_rect.x = yolo_bbox.xmin - this->rect_margin;
        boat_rect.y = yolo_bbox.ymin - this->rect_margin;
        boat_rect.width = (yolo_bbox.xmax - boat_rect.x) + (this->rect_margin * 2);
        boat_rect.height = (yolo_bbox.ymax - boat_rect.y) + (this->rect_margin * 2);

        if(boat_rect.x < 0){boat_rect.x = 0;}
        if(boat_rect.y < 0){boat_rect.y = 0;}
        if((boat_rect.x + boat_rect.width) > this->color_img.cols){ boat_rect.width = (this->color_img.cols - boat_rect.x);}
        if((boat_rect.y + boat_rect.height) > this->color_img.rows){ boat_rect.height = (this->color_img.rows - boat_rect.y);}

		return boat_rect;
	}//make_margin_rect


	bool measure_ribbon_bridge(cv::Mat& target_img, cv::Rect& bridge_rect, ribbon_bridge_measurement::RibbonBridge& measured_bridge){

		cv::Mat bridge_img(target_img, bridge_rect);//画像のトリミング
        cv::Point2i center_pt = cv::Point2i(bridge_img.rows*0.5, bridge_img.cols*0.5);
		cv::Mat gray_img;
		//cv::Mat bin_img;
		cv::cvtColor(bridge_img, gray_img, CV_BGR2GRAY);
		cv::normalize(gray_img, gray_img, 0, 255, cv::NORM_MINMAX);
		//cv::threshold(gray_img, bin_img, 200, 255, cv::THRESH_BINARY);//220で2値化処理
		//cv::erode(bin_img, bin_img, cv::Mat(), cv::Point(-1, -1), 1);//拡大縮小によるノイズ除去
		//cv::dilate(bin_img, bin_img, cv::Mat(), cv::Point(-1, -1), 1);

		std::vector<cv::Point2f> corners;
		cv::goodFeaturesToTrack(gray_img, corners, 32, 0.01, 3, cv::Mat(), 3, true);
		if (corners.size() < 4) { return false; }

        /*
        //cv::Mat descriptor;
        std::vector<cv::KeyPoint> keypoints;
        //cv::KeyPoint::convert(corners, keypoints);
        auto orb = cv::ORB::create();
        //orb->setMaxFeatures(32);
        orb->detect(gray_img, keypoints);
        //orb->compute(gray_img, keypoints, descriptor);


        std::vector<cv::KeyPoint> keypoints_next;
        for(int i = 0; i < corners.size(); i++){
            double length = DBL_MAX;
            int temp_index = -1;
            for(int key = 0; key < keypoints.size(); key++){
                double temp_distance = sqrt(pow(corners[i].x-keypoints[key].pt.x,2)+pow(corners[i].y-keypoints[key].pt.y,2));
                if(temp_distance < length){
                    length = temp_distance;
                    temp_index = key;
                }
            }
            keypoints_next.push_back(keypoints[temp_index]);
        }//for

        cv::Mat dstImg;
        cv::drawKeypoints(target_img, keypoints_next, dstImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow("Keypoints", dstImg);
        */

        std::vector<std::pair<float, int>> distance_and_index;
        for(int i = 0; i < corners.size(); i++){
            float distance = sqrt(pow(corners[i].x-center_pt.x,2)+pow(corners[i].y-center_pt.y,2));
            int index = i;
            std::pair<float, int> temp_data = std::make_pair(distance, index);
            distance_and_index.push_back(temp_data);
        }
        sort(distance_and_index.begin(), distance_and_index.end());

        std::cout << "***************************" << std::endl;
        for(int i = 0; i < distance_and_index.size(); i ++){
            std::cout << "distance = " << distance_and_index[i].first << " index = " << distance_and_index[i].second << std::endl;
        }

        //ここから近い物順に選択＆組み合わせを試していく

        for(int num = 4; num < distance_and_index.size(); num++){
            std::vector<cv::Point2i> selected_corners;
            for(int i = 0; i < num; i++){
                selected_corners.push_back(corners[distance_and_index[i].second]);
            }
        }


		for (int i = 0; i < corners.size(); i++) {
			cv::circle(gray_img, cv::Point(corners[i].x, corners[i].y), 1, cv::Scalar(0, 255, 0), -1);
			cv::circle(gray_img, cv::Point(corners[i].x, corners[i].y), 8, cv::Scalar(0, 255, 0));
		}
        cv::imshow("gray_img", gray_img);
        //cv::imshow("bin_image", bin_img);
        cv::waitKey(27);

		cv::RotatedRect rc = cv::minAreaRect(corners);
		cv::Point2f vertexes[4];
		rc.points(vertexes);
		std::vector<cv::Point2f> points(vertexes, vertexes + 4);

		//minAreaRectで推定されたコーナーを基に浮橋の位置を推定
		std::vector<cv::Point2f> subpix_corners;
        this->find_most_closest_pt(corners, points, subpix_corners);
		cv::cornerSubPix(gray_img, subpix_corners, cv::Size(11, 11), cv::Size(-1, -1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		//中心位置を推定
		cv::Point2f center;
		double a1 = (subpix_corners[2].y - subpix_corners[0].y) / (subpix_corners[2].x - subpix_corners[0].x);
		double a2 = (subpix_corners[3].y - subpix_corners[1].y) / (subpix_corners[3].x - subpix_corners[1].x);
		center.x = (a1 * subpix_corners[0].x - subpix_corners[0].y - a2 * subpix_corners[1].x + subpix_corners[1].y) / (a1 - a2);
		center.y = (subpix_corners[2].y - subpix_corners[0].y) / (subpix_corners[2].x - subpix_corners[0].x)*(center.x - subpix_corners[0].x) + subpix_corners[0].y;

		//angleの計測
		//double len_03 = sqrt(pow((subpix_corners[3]- subpix_corners[0]).x, 2) + pow((subpix_corners[3] - subpix_corners[0]).y, 2));
		//double len_01 = sqrt(pow((subpix_corners[1] - subpix_corners[0]).x, 2) + pow((subpix_corners[1] - subpix_corners[0]).y, 2));

		//計測結果の格納
		measured_bridge.center.x = center.x + bridge_rect.x;
		measured_bridge.center.y = center.y + bridge_rect.y;
		for(int i = 0; i < 4; i++){
			geometry_msgs::Pose2D corner_pt;
			corner_pt.x = subpix_corners[i].x + bridge_rect.x;
			corner_pt.y = subpix_corners[i].y + bridge_rect.y;
			measured_bridge.corners.push_back(corner_pt);
		}//for

		return true;
	}//measure_ribbon_bridge


    void find_most_closest_pt(std::vector<cv::Point2f> &source, std::vector<cv::Point2f> &target, std::vector<cv::Point2f> &result){
        //minAreaRectで推定されたコーナーに最も近いコーナーを算出
        for (int i = 0; i < target.size(); i++) {
            double min_distance = DBL_MAX;
            cv::Point2f min_point;
            for (int j = 0; j < source.size(); j++) {
                cv::Point2f sub_point = target[i] - source[j];
                double distance = sqrt(pow(sub_point.x, 2) + pow(sub_point.y, 2));
                if (distance < min_distance) { min_distance = distance; min_point = source[j];	}//最短距離のコーナーを更新
            }
            result.push_back(min_point);
        }
    }


	void save_result_img(){
		time_t now = time(NULL);
		struct tm *pnow = localtime(&now);
		std::stringstream file_name;
		file_name << ros::package::getPath(this->this_package_name) << "/Result/" << pnow->tm_mon + 1 <<"_"<< pnow->tm_mday <<"_"<< pnow->tm_hour <<"_"<< pnow->tm_min <<"_"<< pnow->tm_sec <<".png";
		cv::imwrite(file_name.str(), this->result_img);
	}//save_result_img


	void publish_result_img(){
		std_msgs::Header header; // empty header
		header.seq = this->counter; // user defined counter
		header.stamp = ros::Time::now();

		cv_bridge::CvImage result_img;
		sensor_msgs::Image result_msg;
		result_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, this->result_img);
		result_img.toImageMsg(result_msg); // from cv_bridge to sensor_msgs::Image
		this->pub_result_img.publish(result_msg);
		this->counter++;
	}//publish_result_img


    void show_result_image(){
        cv::imshow("Ribbon_Bridge_Corner_Result", this->result_img);
        cv::waitKey(27);
    }//show_result_image


	void publish_overlay_text(std::string& word){
		jsk_rviz_plugins::OverlayText send_msg;
		send_msg.text = word;
		this->pub_overlay_text.publish(send_msg);
	}//publish_overlay_text


	void make_result_image(ribbon_bridge_measurement::RibbonBridges data){

		this->result_img = this->target_img.clone();//結果の描画用画像の確保

		for(int i = 0; i < data.RibbonBridges.size(); i++){

			cv::Point2f center = cv::Point2f(data.RibbonBridges[i].center.x, data.RibbonBridges[i].center.y);
			cv::circle(this->result_img, center, 8, cv::Scalar(255, 0, 0));

			std::vector<cv::Point2f> corners;
			for(int j = 0; j < data.RibbonBridges[i].corners.size(); j++){
				cv::Point2f corner = cv::Point2f(data.RibbonBridges[i].corners[j].x, data.RibbonBridges[i].corners[j].y);
				corners.push_back(corner);
			}//for
			cv::line(this->result_img, corners[0], corners[2], cv::Scalar(255, 0, 0), 1, CV_AA);//クロスラインの描画
			cv::line(this->result_img, corners[1], corners[3], cv::Scalar(255, 0, 0), 1, CV_AA);
			cv::line(this->result_img, corners[0], corners[1], cv::Scalar(0, 255, 0), 3, CV_AA);//矩形の描画
			cv::line(this->result_img, corners[1], corners[2], cv::Scalar(0, 255, 0), 3, CV_AA);
			cv::line(this->result_img, corners[2], corners[3], cv::Scalar(0, 255, 0), 3, CV_AA);
			cv::line(this->result_img, corners[3], corners[0], cv::Scalar(0, 255, 0), 3, CV_AA);
		}//for
	}//make_result_image


private:

    ros::NodeHandle nh;
    ros::Subscriber sub_camera_img;
    ros::Subscriber sub_ctrl_flag;
    ros::Subscriber sub_yolo_bbox;
    ros::Publisher pub_result_img;
    ros::Publisher pub_overlay_text;
    ros::Publisher pub_ribbon_bridges_msg;

    cv_bridge::CvImagePtr cv_ptr;

    std::string image_topic_name;
    std::string yolo_topic_name;
    std::string this_package_name;
    int counter;
    bool save_result_flag;
    bool show_result_flag;
    bool pub_result_flag;
    double boat_aspect_ratio;
    cv::Mat color_img;
    cv::Mat target_img;
    cv::Mat result_img;

    int rect_margin;

};//RibbonBridgeMeasurement


int main(int argc, char** argv) {

    ros::init(argc, argv, "ribbon_bridge_corner_node");

    RibbonBridgeMeasurement rbm;
    ros::spin();
    return 0;

}//main
