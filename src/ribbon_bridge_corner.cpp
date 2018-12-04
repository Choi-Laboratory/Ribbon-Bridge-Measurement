#include <iostream>
#include <time.h>
#include <cstdlib>
#include <stdlib.h>
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

		this->boat_aspect_ratio = 1.45454545;
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
		measuredBridges.header.stamp = ros::Time::now();
		measuredBridges.header.frame_id = "camera_image";

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
		cv::Point2f center_pt = cv::Point2i(bridge_img.cols*0.5, bridge_img.rows*0.5);
		cv::Mat gray_img;
		cv::cvtColor(bridge_img, gray_img, CV_BGR2GRAY);
		cv::normalize(gray_img, gray_img, 0, 255, cv::NORM_MINMAX);

		std::vector<cv::Point2f> corners;
		cv::goodFeaturesToTrack(gray_img, corners, 32, 0.01, 3, cv::Mat(), 3, true);
		this->show_corner_img(bridge_img.clone(), corners);
		if (corners.size() < 4) { return false; }

		//orb特徴の抽出
		std::vector<cv::KeyPoint> keypoints;
		this->detect_orb_from_gray_img(gray_img, corners, keypoints);

		//ここから特徴点の角度の対応を探査していく
		std::vector<cv::KeyPoint> threshold_keypoints;
		for(int i = 0; i < keypoints.size(); i++){
			cv::Point2f center2key_pt = keypoints[i].pt - center_pt;
			float center2key_angle = atan2(center2key_pt.y,center2key_pt.x) * 180.0 / PI;

			float keypoint_angle = keypoints[i].angle;
			if(keypoint_angle > 180){keypoint_angle -= 360;}
			//std::cout << "keypoints.angle = " << keypoint_angle << " center2key_angle = " << center2key_angle_deg << std::endl;

			if(std::abs(keypoint_angle-center2key_angle) < 20){
					threshold_keypoints.push_back(keypoints[i]);
			}
		}//for

		cv::Mat dstImg;
		cv::drawKeypoints(bridge_img, threshold_keypoints, dstImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		//cv::imshow("Keypoints", dstImg);
		//cv::waitKey(27);

		//ここから組み合わせを探査していく
		std::vector<std::vector<int>> evalating_combi;

		for(int i1 = 0; i1 < threshold_keypoints.size(); i1++){
			float angle1 = threshold_keypoints[i1].angle;

			for(int i2 = 0; i2 < threshold_keypoints.size(); i2++){
				float angle2 = threshold_keypoints[i2].angle;
				bool do_next_search3 = this->is_target_angle(angle1, angle2, -90, 10);

				for(int i3 = 0; i3 < threshold_keypoints.size() && do_next_search3 == true; i3++){
					float angle3 = threshold_keypoints[i3].angle;
					bool do_next_search4 = this->is_target_angle(angle1, angle3, 90, 10);

					for(int i4 = 0; i4 < threshold_keypoints.size() && do_next_search4 == true; i4++){
						float angle4 = threshold_keypoints[i4].angle;
						bool is_save_combi = this->is_target_angle(angle1, angle4, 180, 10);
						if(is_save_combi == true){
								std::vector<int> temp_data;
								temp_data.push_back(i1);
								temp_data.push_back(i2);
								temp_data.push_back(i3);
								temp_data.push_back(i4);
								std::sort(temp_data.begin(),temp_data.end());
								bool is_new_combi = true;
								for(int i = 0; i < evalating_combi.size(); i++){//check
									if(evalating_combi[i][0]==temp_data[0] && evalating_combi[i][1]==temp_data[1] && evalating_combi[i][2]==temp_data[2] && evalating_combi[i][3]==temp_data[3]){
										is_new_combi = false;
									}
								}
								if(is_new_combi == true){ evalating_combi.push_back(temp_data); }
							}//if
					}//for_i4
				}//for_i3
			}//for_i2
		}//for_i1

		//ここから組み合わせを試していく
		double best_aspect_ratio_gap = DBL_MAX;
		std::vector<cv::Point2f> best_subpix_corners;
		for(int num = 0; num < evalating_combi.size(); num++){

			std::vector<cv::Point2i> selected_corners;
			for(int i = 0; i < evalating_combi[num].size(); i++){
				selected_corners.push_back(threshold_keypoints[evalating_combi[num][i]].pt);
			}

			cv::RotatedRect rc = cv::minAreaRect(selected_corners);
			cv::Point2f vertexes[4];
			rc.points(vertexes);
			std::vector<cv::Point2f> points(vertexes, vertexes + 4);

			//コーナー角度の計算
			bool is_corner_correct = this->check_is_corner_correct(points);
			if(is_corner_correct == false){ continue; }

			//minAreaRectで推定されたコーナーを基に浮橋の位置を推定
			std::vector<cv::Point2f> subpix_corners;
			this->find_most_closest_pt(corners, points, subpix_corners);
			cv::cornerSubPix(gray_img, subpix_corners, cv::Size(11, 11), cv::Size(-1, -1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			//アスペクト比の計算
			double aspect_ratio = this->calc_aspect_ratio(subpix_corners);
			double aspect_ratio_gap = std::abs(this->boat_aspect_ratio - aspect_ratio);

			if(aspect_ratio_gap < best_aspect_ratio_gap){
				best_aspect_ratio_gap = aspect_ratio_gap;
				best_subpix_corners.clear();
				copy(subpix_corners.begin(), subpix_corners.end(), back_inserter(best_subpix_corners));
			}

		}//for_corner_num

		if(best_aspect_ratio_gap > 0.15){ return false; }

		//中心位置を推定
		cv::Point2f bridge_center_pt = this->calc_bridge_center_pt(best_subpix_corners);
		double bridge_angle = this->calc_bridge_angle(best_subpix_corners);

		//計測結果の格納
		measured_bridge.center.x = bridge_center_pt.x + bridge_rect.x;
		measured_bridge.center.y = bridge_center_pt.y + bridge_rect.y;
		for(int i = 0; i < 4; i++){
			geometry_msgs::Pose2D corner_pt;
			corner_pt.x = best_subpix_corners[i].x + bridge_rect.x;
			corner_pt.y = best_subpix_corners[i].y + bridge_rect.y;
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


	cv::Point2f calc_bridge_center_pt(std::vector<cv::Point2f> &corners){
		double a1 = (corners[2].y - corners[0].y) / (corners[2].x - corners[0].x);
		double a2 = (corners[3].y - corners[1].y) / (corners[3].x - corners[1].x);
		cv::Point2f center;
		center.x = (a1 * corners[0].x - corners[0].y - a2 * corners[1].x + corners[1].y) / (a1 - a2);
		center.y = (corners[2].y - corners[0].y) / (corners[2].x - corners[0].x)*(center.x - corners[0].x) + corners[0].y;
		return center;
	}


	double calc_bridge_angle(std::vector<cv::Point2f> &corners){
		double angle1 = this->calc_angle(corners[0],corners[1]);
		double angle2 = this->calc_angle(corners[3],corners[2]);

		//std::cout << "bridge_angle = " << angle1 << ",  " << angle2 << std::endl;
		return (angle1 + angle2) / 2;
	}


	double calc_aspect_ratio(std::vector<cv::Point2f> &corners){
		double len_0 = sqrt(pow((corners[1]-corners[0]).x,2) + pow((corners[1]-corners[0]).y,2));
		double len_1 = sqrt(pow((corners[2]-corners[1]).x,2) + pow((corners[2]-corners[1]).y,2));
		double len_2 = sqrt(pow((corners[3]-corners[2]).x,2) + pow((corners[3]-corners[2]).y,2));
		double len_3 = sqrt(pow((corners[0]-corners[3]).x,2) + pow((corners[0]-corners[3]).y,2));
		double ratio0 = std::max(len_0,len_1)/std::min(len_0,len_1);
		double ratio1 = std::max(len_1,len_2)/std::min(len_1,len_2);
		double ratio2 = std::max(len_2,len_3)/std::min(len_2,len_3);
		double ratio3 = std::max(len_3,len_0)/std::min(len_3,len_0);
		return (ratio0+ratio1+ratio2+ratio3)/4;
	}


	bool check_is_corner_correct(std::vector<cv::Point2f> &corners){
		bool corner_0 = this->is_angle_90deg(this->calc_angle(corners[0],corners[1]),this->calc_angle(corners[0],corners[3]),3);
		bool corner_1 = this->is_angle_90deg(this->calc_angle(corners[1],corners[0]),this->calc_angle(corners[1],corners[2]),3);
		bool corner_2 = this->is_angle_90deg(this->calc_angle(corners[2],corners[1]),this->calc_angle(corners[2],corners[3]),3);
		bool corner_3 = this->is_angle_90deg(this->calc_angle(corners[3],corners[0]),this->calc_angle(corners[3],corners[2]),3);

		if(corner_0 && corner_1 && corner_2 && corner_3){ return true; }
		else{ return false; }
	}//check_is_corner_correct


	double calc_angle(cv::Point2f source_point, cv::Point2f target_point){
		cv::Point2f sub_point = (target_point - source_point);
		float angle = atan2(sub_point.y,sub_point.x) * 180.0 / PI;
		if(angle < 0){ angle += 360; }
		return angle;
	}


	bool is_angle_90deg(float angle1, float angle2, float threshold_deg){
		float gap_deg = abs(angle1 - angle2);
		if(gap_deg > 180){ gap_deg -= 180; }
		if((90-threshold_deg) < gap_deg && gap_deg < (90+threshold_deg)){ return true; }
		else{ return false; }
	}


	void detect_orb_from_gray_img(cv::Mat image, std::vector<cv::Point2f> &corners, std::vector<cv::KeyPoint> &dst_keypoints){
		std::vector<cv::KeyPoint> keypoints;
		auto orb = cv::ORB::create();
		orb->detect(image, keypoints);

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
			dst_keypoints.push_back(keypoints[temp_index]);
		}//for
	}


	void show_corner_img(cv::Mat image, std::vector<cv::Point2f> corners){
		for (int i = 0; i < corners.size(); i++) {
			cv::circle(image, cv::Point(corners[i].x, corners[i].y), 1, cv::Scalar(0, 255, 0), -1);
			cv::circle(image, cv::Point(corners[i].x, corners[i].y), 8, cv::Scalar(0, 255, 0));
		}
		cv::Point2i center_pt = cv::Point2i(image.cols*0.5, image.rows*0.5);
		cv::circle(image, cv::Point(center_pt.x, center_pt.y), 8, cv::Scalar(0, 0, 255));
		//cv::imshow("corners_image", image);
		//cv::waitKey(27);
	}


	bool is_target_angle(float source_deg, float evalate_deg, int gap_deg, float threshold_deg){
		int target_deg = source_deg + gap_deg;
		if(target_deg > 360){target_deg -= 360;}
		else if(target_deg < 0){target_deg += 360;}

		int target_max = target_deg + threshold_deg;
		int target_min = target_deg - threshold_deg;

		if((0 < target_max && target_max < 360) && (0 < target_min && target_min < 360)){
			if((target_min < evalate_deg) && (evalate_deg < target_max)){ return true; }
			else{ return false;}
		}
		else if((target_max > 360) && (0 < target_min && target_min < 360)){
			if((target_min < evalate_deg) || (evalate_deg < (target_max-360))){ return true; }
			else{ return false;}
		}
		else if((0 < target_max && target_max < 360) && (target_min < 0)){
			if((evalate_deg > (target_min+360)) || (evalate_deg < target_max)){ return true; }
			else{ return false;}
		}else{
			std::cout << "is_target_angleで想定しないelse source_deg = "
			 << source_deg << " evalate_deg = " << evalate_deg << " gap_deg = " << gap_deg << std::endl;
		}
	}//is_target_angle


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
		//cv::imshow("Ribbon_Bridge_Corner_Result", this->result_img);
		//cv::waitKey(27);
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
