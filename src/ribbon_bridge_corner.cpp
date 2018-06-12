#include <iostream>
#include <fstream>
#include <time.h>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <jsk_rviz_plugins/OverlayText.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <Ribbon-Bridge-Measurement/RibbonBridge.h>
#include <Ribbon-Bridge-Measurement/RibbonBridges.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#define PI 3.141592653589793


class Ribbon_Bridge_Measurement{

public:

    Ribbon_Bridge_Measurement(){

        ros::param::get("/boat_image_topic_name", image_topic_name);
        ros::param::get("/boat_contrl_topic_name", contrl_topic_name);
        ros::param::get("/boat_bbox_topic_name", boat_bbox_topic_name);
        ros::param::get("/boat_execute_default", exe_flag);
        ros::param::get("/boat_package_name", this_node_name);
        ros::param::get("/boat_base_frame_name", base_frame_name);
        ros::param::get("/boat_save_result_image", save_result);
        ros::param::get("/boat_show_result", show_result);
        ros::param::get("/boat_height", boat_height);
        ros::param::get("/boat_width", boat_width);
        ros::param::get("/yolo_detect_threshold", yolo_detect_threshold);
        ros::param::get("/boat_rect_margin", rect_margin);

        this->counter = 0;
        this->boat_aspect_ratio = (boat_width / boat_height);

        this->sub_rgb = nh.subscribe(image_topic_name, 1, &Boat_Measurement::rgbImageCallback, this);
        this->sub_contrl = nh.subscribe(contrl_topic_name, 1, &Boat_Measurement::contrlCallback, this);
        this->sub_yolobbox = nh.subscribe(boat_bbox_topic_name, 1, &Boat_Measurement::yolobboxCallback, this);
        this->pub_result = nh.advertise<sensor_msgs::Image>("boat_result", 1);
        this->pub_overlay_text = nh.advertise<jsk_rviz_plugins::OverlayText>("overlay_text", 1);
        this->pub_polygon_array = nh.advertise<jsk_recognition_msgs::PolygonArray>("detect_boat_polygon", 1);

    }//Boat_Measurement()


    void contrlCallback(const std_msgs::Bool& msg){
        this->exe_flag = msg.data;
    }//contrlCallback

    void rgbImageCallback(const sensor_msgs::Image::ConstPtr& msg){

        try{
            this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge error %s",e.what());
            return;
        }
        cv::Mat &image = this->cv_ptr->image;
        this->color_img = image.clone();
    }//rgbImageCallback

	void publish_overlay_text(std::string& word){
		jsk_rviz_plugins::OverlayText text;
		text.text = word;
		this->pub_overlay_text.publish(text);
	}//publish_overlay_text

	void save_result_img(cv::Mat& image){
		time_t now = time(NULL);
		struct tm *pnow = localtime(&now);
		std::stringstream file_name;
		file_name << ros::package::getPath(this->this_node_name) << "/Result/" << pnow->tm_mon + 1 <<"_"<< pnow->tm_mday <<"_"<< pnow->tm_hour <<"_"<< pnow->tm_min <<"_"<< pnow->tm_sec <<".png";
		cv::imwrite(file_name.str(), image);
	}//save_result_img

	cv::Rect make_margin_rect(darknet_ros_msgs::BoundingBox& yolo_bbox){
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

	void publish_result_img(cv::Mat& image){
		std_msgs::Header header; // empty header
		header.seq = this->counter; // user defined counter
		header.stamp = ros::Time::now();

		cv_bridge::CvImage result_img;
		sensor_msgs::Image result_msg;
		result_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
		result_img.toImageMsg(result_msg); // from cv_bridge to sensor_msgs::Image
		this->pub_result.publish(result_msg);
		this->counter++;
	}//publish_result_img

	void make_result_image(Ribbon-Bridge-Measurement::RibbonBridges data){

		this->result_img = this->target_img.clone();//結果の描画用画像の確保

		for(int i = 0; i < data.RibbonBridges.size(); i++){

			cv::Point2f center = cv::Point2f(data.RibbonBridges[i].center.x, data.RibbonBridges[i].center.y);
			cv::circle(this->result_img, center, 8, cv::Scalar(255, 0, 0));

			std::vector<cv::point2f> corners;
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

	Ribbon-Bridge-Measurement::RibbonBridge measure(cv::Mat& target_img, cv::Rect& bridge_rect, bool& detect_flag){

		Ribbon-Bridge-Measurement::RibbonBridge measured_bridge;//返り値用
		cv::Mat bridge_img(target_img, bridge_rect);//画像のトリミング

		cv::Mat gray_img;
		cv::Mat bin_img;
		cv::cvtColor(bridge_img, gray_img, CV_BGR2GRAY);
		cv::normalize(gray_img, gray_img, 0, 255, cv::NORM_MINMAX);
		cv::threshold(gray_img, bin_img, 220, 255, cv::THRESH_BINARY);//220で2値化処理
		cv::erode(bin_img, bin_img, cv::Mat(), cv::Point(-1, -1), 1);//拡大縮小によるノイズ除去
		cv::dilate(bin_img, bin_img, cv::Mat(), cv::Point(-1, -1), 1);

		std::vector<cv::Point2f> corners;
		cv::goodFeaturesToTrack(bin_img, corners, 32, 0.01, 3, cv::Mat(), 3, true);
		if (corners.size() < 4) { return; }

		//for (int i = 0; i < corners.size(); i++) {
			//cv::circle(image, cv::Point(corners[i].x, corners[i].y), 1, cv::Scalar(0, 255, 0), -1);
			//cv::circle(image, cv::Point(corners[i].x, corners[i].y), 8, cv::Scalar(0, 255, 0));
		//}

		cv::RotatedRect rc = cv::minAreaRect(corners);
		cv::Point2f vertexes[4];
		rc.points(vertexes);
		std::vector<cv::Point2f> points(vertexes, vertexes + 4);

		//minAreaRectで推定されたコーナーに最も近いコーナーを算出
		std::vector<cv::Point2f> subpix_corners;
		for (int i = 0; i < points.size(); i++) {
			double min_distance = DBL_MAX;
			cv::Point2f min_point;
			for (int j = 0; j < corners.size(); j++) {
				cv::Point2f sub_point = points[i] - corners[j];
				double distance = sqrt(pow(sub_point.x, 2) + pow(sub_point.y, 2));
				if (distance < min_distance) { min_distance = distance; min_point = corners[j];	}//最短距離のコーナーを更新
			}//for_j
			subpix_corners.push_back(min_point);
		}//for_i

		//サブピクセル推定
		cv::cornerSubPix(gray_img, subpix_corners, cv::Size(11, 11), cv::Size(-1, -1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		//中心位置を推定
		cv::Point2f center;
		double a1 = (subpix_corners[2].y - subpix_corners[0].y) / (subpix_corners[2].x - subpix_corners[0].x);
		double a2 = (subpix_corners[3].y - subpix_corners[1].y) / (subpix_corners[3].x - subpix_corners[1].x);
		center.x = (a1 * subpix_corners[0].x - subpix_corners[0].y - a2 * subpix_corners[1].x + subpix_corners[1].y) / (a1 - a2);
		center.y = (subpix_corners[2].y - subpix_corners[0].y) / (subpix_corners[2].x - subpix_corners[0].x)*(center.x - subpix_corners[0].x) + subpix_corners[0].y;

		//angle�̎Z�o
		//double len_03 = sqrt(pow((subpix_corners[3]- subpix_corners[0]).x, 2) + pow((subpix_corners[3] - subpix_corners[0]).y, 2));
		//double len_01 = sqrt(pow((subpix_corners[1] - subpix_corners[0]).x, 2) + pow((subpix_corners[1] - subpix_corners[0]).y, 2));

		//計測結果の格納
		measured_bridge.center.x = center.x + bridge_rect.x;
		measured_bridge.center.x = center.y + bridge_rect.y;
		for(int i = 0; i < 4; i++){
			geometry_msgs::Pose2D corner_pt;
			corner_pt.x = subpix_corners[i].x + bridge_rect.x;
			corner_pt.y = subpix_corners[i].y + bridge_rect.y;
			measured_bridge.corners.push_back(corner_pt);
		}//for
		detect_flag = true;
		return measured_bridge;

	}//measure

    void yolobboxCallback(const darknet_ros_msgs::BoundingBoxes& msg){

        if(this->exe_flag == false){ return; }
        if(this->color_img.empty() == true){
			ROS_ERROR_STREAM("yolobboxCallback -> image empty");
			return;
		}//if

		//ここから計測
		this->target_img = this->color_img.clone();
		Ribbon-Bridge-Measurement::RibbonBridges measured_bridges;//計測結果の格納用

        for(int bbox_count = 0; bbox_count < msg.bounding_boxes.size(); bbox_count++){

			//確率が低い場合の例外処理
            if(msg.bounding_boxes[bbox_count].probability < this->yolo_detect_threshold){continue;}

            cv::Rect bridge_rect = this->make_margin_rect(msg.bounding_boxes[bbox_count]);//余白を付与したrectの作成

			bool detect_flag = false;
			Ribbon-Bridge-Measurement::RibbonBridge bridge_data = this->measure(target_img, bridge_rect, detect_flag);

			if(detect_flag == true){
				measured_bridges.push_back(bridge_data);
			}

        }//bbox_count

        ROS_INFO_STREAM("--- Detect " << measured_bridges.RibbonBridges.size() << "Bridges ---");

		if(this->show_result || this->save_result){ this->make_result_image(measured_bridges); }//計測結果画像の作成
        if(this->show_result == true){ this->publish_result_img(this->result_image); }//結果画像のpublish
        if(this->save_result == true){ this->save_result_img(this->result_image); }//結果画像の保存

    }//yolobboxCallback

private:

    ros::NodeHandle nh;
    ros::Subscriber sub_rgb;
    ros::Subscriber sub_contrl;
    ros::Subscriber sub_yolobbox;
    ros::Publisher pub_result;
    ros::Publisher pub_overlay_text;
    ros::Publisher pub_polygon_array;

    tf::TransformBroadcaster br;
    cv_bridge::CvImagePtr cv_ptr;

    std::string image_topic_name;
    std::string contrl_topic_name;
    std::string boat_bbox_topic_name;
    std::string this_node_name;
    std::string base_frame_name;
    int counter;
    bool save_result;
    bool show_result;
    bool exe_flag;
    double boat_height;
    double boat_width;
    double boat_aspect_ratio;
    double yolo_detect_threshold;
    cv::Mat color_img;
	cv::Mat target_img;
	cv::Mat result_img;

    int rect_margin;
    std::vector<cv::Point2d> detect_boat_pt;
    std::vector<double> detect_boat_angle;

};//boat_measurement

/*
int main(int argc, char** argv){

    ros::init(argc, argv, "ribbon_bridge_lsd_node");

    Boat_Measurement bm;
    ros::spin();
    return 0;

}//main
*/


int main() {


	cv::Mat image;
	cv::VideoCapture cap = cv::VideoCapture(0);

	while (true) {

		cap >> image;

		cv::Mat gray_img;
		cv::Mat bin_img;
		cv::cvtColor(image, gray_img, CV_BGR2GRAY);
		cv::normalize(gray_img, gray_img, 0, 255, cv::NORM_MINMAX);
		cv::threshold(gray_img, bin_img, 220, 255, cv::THRESH_BINARY);//220で2値化処理
		cv::erode(bin_img, bin_img, cv::Mat(), cv::Point(-1, -1), 1);//拡大縮小によるノイズ除去
		cv::dilate(bin_img, bin_img, cv::Mat(), cv::Point(-1, -1), 1);

		std::vector<cv::Point2f> corners;
		cv::goodFeaturesToTrack(bin_img, corners, 32, 0.01, 3, cv::Mat(), 3, true);

		for (int i = 0; i < corners.size(); i++) {
			cv::circle(image, cv::Point(corners[i].x, corners[i].y), 1, cv::Scalar(0, 255, 0), -1);
			cv::circle(image, cv::Point(corners[i].x, corners[i].y), 8, cv::Scalar(0, 255, 0));
		}

		if (corners.size() >= 4) {

			cv::Mat result_img = image.clone();
			cv::RotatedRect rc = cv::minAreaRect(corners);
			cv::Point2f vertexes[4];
			rc.points(vertexes);
			std::vector<cv::Point2f> points(vertexes, vertexes + 4);

			//minAreaRectで推定されたコーナーに最も近いコーナーを算出
			std::vector<cv::Point2f> subpix_corners;
			for (int i = 0; i < points.size(); i++) {
				double min_distance = DBL_MAX;
				cv::Point2f min_point;
				for (int j = 0; j < corners.size(); j++) {
					cv::Point2f sub_point = points[i] - corners[j];
					double distance = sqrt(pow(sub_point.x, 2) + pow(sub_point.y, 2));
					if (distance < min_distance) {
						min_distance = distance;
						min_point = corners[j];
					}//if
				}//for_j
				subpix_corners.push_back(min_point);
			}//for_i

			//サブピクセル推定
			cv::cornerSubPix(gray_img, subpix_corners, cv::Size(11, 11), cv::Size(-1, -1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			//中心位置を推定
			cv::Point2f center;
			double a1 = (subpix_corners[2].y - subpix_corners[0].y) / (subpix_corners[2].x - subpix_corners[0].x);
			double a2 = (subpix_corners[3].y - subpix_corners[1].y) / (subpix_corners[3].x - subpix_corners[1].x);
			center.x = (a1 * subpix_corners[0].x - subpix_corners[0].y - a2 * subpix_corners[1].x + subpix_corners[1].y) / (a1 - a2);
			center.y = (subpix_corners[2].y - subpix_corners[0].y) / (subpix_corners[2].x - subpix_corners[0].x)*(center.x - subpix_corners[0].x) + subpix_corners[0].y;
			cv::circle(result_img, center, 8, cv::Scalar(255, 0, 0));

			//angle�̎Z�o
			//double len_03 = sqrt(pow((subpix_corners[3]- subpix_corners[0]).x, 2) + pow((subpix_corners[3] - subpix_corners[0]).y, 2));
			//double len_01 = sqrt(pow((subpix_corners[1] - subpix_corners[0]).x, 2) + pow((subpix_corners[1] - subpix_corners[0]).y, 2));


			//結果の描画
			cv::line(result_img, subpix_corners[0], subpix_corners[2], cv::Scalar(255, 0, 0), 1, CV_AA);
			cv::line(result_img, subpix_corners[1], subpix_corners[3], cv::Scalar(255, 0, 0), 1, CV_AA);

			cv::line(result_img, subpix_corners[0], subpix_corners[1], cv::Scalar(0, 255, 0), 3, CV_AA);
			cv::line(result_img, subpix_corners[1], subpix_corners[2], cv::Scalar(0, 255, 0), 3, CV_AA);
			cv::line(result_img, subpix_corners[2], subpix_corners[3], cv::Scalar(0, 255, 0), 3, CV_AA);
			cv::line(result_img, subpix_corners[3], subpix_corners[0], cv::Scalar(0, 255, 0), 3, CV_AA);
			cv::imshow("result", result_img);
			cv::waitKey(27);
		}

		//cv::imshow("bin_image", bin_img);
		//cv::imshow("image", image);
		//cv::waitKey(27);

	}//while


}//main
