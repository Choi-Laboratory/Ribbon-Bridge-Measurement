#include <iostream>
#include <fstream>
#include <time.h>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <std_msgs/UInt8.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/line_descriptor.hpp>

#define PI 3.141592653589793

class Boat_Measurement{

public:

    Boat_Measurement(){

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
        this->descriptor = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();

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


    void yolobboxCallback(const darknet_ros_msgs::BoundingBoxes& msg){

        if(this->exe_flag == false){ return; }
        if(color_img.empty() == true){
          ROS_ERROR_STREAM("yolobboxCallback -> image empty");
          return;
        }

        std::vector<cv::Rect> yolo_boat_rect;
        std::vector<std::vector<cv::Point2f> > boat_corner_array;//検出された浮橋の矩形
        std::vector<double> boat_prob;//検出された浮橋の確率
        std::vector<cv::Point2d> boat_center_px_array;//検出された浮き橋の中心座標
        std::vector<double> boat_angle_array;//検出された浮き橋の角度
        std::vector<std::vector<cv::line_descriptor::KeyLine> > detected_lines;//検出された線分特徴
        std::vector<cv::Point2f> detected_lines_startpt;

        for(int bbox_count = 0; bbox_count < msg.bounding_boxes.size(); bbox_count++){

            if(msg.bounding_boxes[bbox_count].probability < this->yolo_detect_threshold){continue;}

            cv::Rect boat_rect;
            boat_rect.x = msg.bounding_boxes[bbox_count].xmin - rect_margin;
            boat_rect.y = msg.bounding_boxes[bbox_count].ymin - rect_margin;
            boat_rect.width = (msg.bounding_boxes[bbox_count].xmax - boat_rect.x) + (rect_margin * 2);
            boat_rect.height = (msg.bounding_boxes[bbox_count].ymax - boat_rect.y) + (rect_margin * 2);

            if(boat_rect.x < 0){boat_rect.x = 0;}
            if(boat_rect.y < 0){boat_rect.y = 0;}
            if((boat_rect.x + boat_rect.width) > this->color_img.cols){ boat_rect.width = (this->color_img.cols - boat_rect.x);}
            if((boat_rect.y + boat_rect.height) > this->color_img.rows){ boat_rect.height = (this->color_img.rows - boat_rect.y);}

            if(boat_rect.x < 0 || boat_rect.y < 0 || boat_rect.width <= 0 || boat_rect.height <= 0 ||
            (boat_rect.width + boat_rect.x) > this->color_img.cols || (boat_rect.height + boat_rect.y) > this->color_img.rows){
                ROS_ERROR_STREAM("--- boat_rect error ---");
                ROS_ERROR_STREAM("boat_rect.x = " << boat_rect.x);
                ROS_ERROR_STREAM("boat_rect.y = " << boat_rect.y);
                ROS_ERROR_STREAM("boat_rect.width = " << boat_rect.width);
                ROS_ERROR_STREAM("boat_rect.height = " << boat_rect.height);
                continue;
            }

            yolo_boat_rect.push_back(boat_rect);
            cv::Point2f start_pt = cv::Point2f(boat_rect.x, boat_rect.y);

            cv::Mat boat_image(this->color_img, boat_rect);//画像トリミング
            cv::Mat input_mask;
            std::vector<cv::line_descriptor::KeyLine> input_lines;
            descriptor->detect(boat_image, input_lines, input_mask);
            detected_lines_startpt.push_back(start_pt);
            detected_lines.push_back(input_lines);

            //線分のペア、レーティング結果の格納用配列
            std::vector<std::vector<int> > pair_lines;
            std::vector<double> pair_rate;
            std::vector<std::vector<cv::Point2f> > vertex_array;

            for (int i = 0; i < input_lines.size(); i++){
                for (int j = 0; j < input_lines.size(); j++){

                    //評価対象かチェック
                    if (i == j){continue;}
                    bool continue_flag = false;
                    for(int num = 0; num < pair_lines.size(); num++){
                        if( (pair_lines[num][0] == i & pair_lines[num][1] == j) || (pair_lines[num][0] == j & pair_lines[num][1] == i)){
                            continue_flag = true;
                            break;
                    }}
                    if(continue_flag == true){ continue; }

                    //新しいペアを格納
                    std::vector<int> new_pair = {i,j};
                    pair_lines.push_back(new_pair);

                    //線の長さの比による閾値処理
                    double length_max = std::max(input_lines[i].lineLength, input_lines[j].lineLength);
                    double length_min = std::min(input_lines[i].lineLength, input_lines[j].lineLength);
                    if ((length_min / length_max) < 0.85){continue;}//線分の長さの差が2割以上

                    //線の角度による閾値処理
                    double angle_lines_rad = abs(input_lines[i].angle - input_lines[j].angle);
                    double angle_lines_deg = 180 * angle_lines_rad / PI;
                    if (abs(angle_lines_deg) > 5.0 || (abs(angle_lines_deg) - 180) > 5.0){continue;}//指定角度外
                    //-----------------------ここからレーティング処理-----------------------------

                    //コーナー座標を配列に格納
                    std::vector<cv::Point2i> corner;
                    corner.push_back(input_lines[i].getStartPoint());
                    corner.push_back(input_lines[i].getEndPoint());
                    corner.push_back(input_lines[j].getStartPoint());
                    corner.push_back(input_lines[j].getEndPoint());

                    cv::RotatedRect rc = cv::minAreaRect(corner);
                    cv::Point2f vertexes[4];
                    rc.points(vertexes);
                    std::vector<cv::Point2f> temp_points(vertexes, vertexes + 4);

                    //ここに角度でしきい値処理を加える
                    double line_angle = std::atan2(input_lines[i].endPointY - input_lines[i].startPointY, input_lines[i].endPointX - input_lines[i].startPointX);
                    double rect_angle = std::atan2(temp_points[1].y - temp_points[0].y, temp_points[1].x - temp_points[0].x);
                    double line_angle_deg = std::fmod((180 * line_angle / PI), 90);
                    double rect_angle_deg = std::fmod((180 * rect_angle / PI), 90);

                    if(line_angle_deg < 0){line_angle_deg += 90;}
                    if(rect_angle_deg < 0){rect_angle_deg += 90;}
                    double sub_line2rect_angle = abs(line_angle_deg - rect_angle_deg);
                    if(sub_line2rect_angle > 10){
                        //ROS_ERROR_STREAM("line2rect_angle error [angle = " << sub_line2rect_angle << "]");
                        continue;
                    }

                    double len_vertex_01 = sqrt(pow(vertexes[0].x - vertexes[1].x, 2) + pow(vertexes[0].y - vertexes[1].y, 2));
                    double len_vertex_12 = sqrt(pow(vertexes[1].x - vertexes[2].x, 2) + pow(vertexes[1].y - vertexes[2].y, 2));
                    double aspect_vertex_bbox = std::max(len_vertex_01, len_vertex_12) / std::min(len_vertex_01, len_vertex_12);//検出された矩形のアスペクト比
                    double rate_aspect = std::min(boat_aspect_ratio, aspect_vertex_bbox) / std::max(boat_aspect_ratio, aspect_vertex_bbox);

                    //
                    printf("aspect_vertex_bbox:[%f]\n", aspect_vertex_bbox);
                    //
                    vertex_array.push_back(temp_points);
                    pair_rate.push_back(rate_aspect);

                }//for_j
            }//for_i

            if(pair_rate.size() == 0){ continue; }
            double max_prob = *std::max_element(pair_rate.begin(), pair_rate.end());
            if(max_prob < 0.4){ continue; }

            std::vector<double>::iterator iter = std::max_element(pair_rate.begin(), pair_rate.end());
            size_t index = std::distance(pair_rate.begin(), iter);

            std::vector<cv::Point2f> img_corner;
            img_corner.push_back(vertex_array[index][0] + start_pt);
            img_corner.push_back(vertex_array[index][1] + start_pt);
            img_corner.push_back(vertex_array[index][2] + start_pt);
            img_corner.push_back(vertex_array[index][3] + start_pt);
            boat_corner_array.push_back(img_corner);//矩形領域を格納
            boat_prob.push_back(max_prob);//確率を格納

            cv::Point2d boat_center = (img_corner[0] + img_corner[1] + img_corner[2] + img_corner[3]) / (double)img_corner.size();
            double boat_angle = std::atan2(img_corner[1].y - img_corner[0].y, img_corner[1].x - img_corner[0].x);
            boat_center_px_array.push_back(boat_center);
            boat_angle_array.push_back(boat_angle);

        }//bbox_count

        ROS_INFO_STREAM("--- Detect " << boat_corner_array.size() << "Boat ---");

        //OverlayTextで浮き橋の検出結果の表示
        std::stringstream pub_word;
        pub_word << "--- Detect " << boat_corner_array.size() << " Boat ---\n";
        for(int i = 0; i < boat_prob.size(); i++){ pub_word << "boat " << i << " prob = " << boat_prob[i] * 100 << "%\n";}
        jsk_rviz_plugins::OverlayText text;
        text.text = pub_word.str();
        this->pub_overlay_text.publish(text);


        std::vector<double> boat_len_vec;
        //px2mを求めるため、浮橋の長さを計算。浮き橋のコーナー座標から浮き橋の中心座標、角度の算出
        for(int bbox_count = 0; bbox_count < boat_corner_array.size(); bbox_count++){
            auto temp_pt = boat_corner_array[bbox_count][1] - boat_corner_array[bbox_count][0];
            float temp_len = sqrt(pow(temp_pt.x,2) + pow(temp_pt.y, 2));
            boat_len_vec.push_back(temp_len);
            //std::cout << "boat_len_px = " << sqrt(pow(temp_pt.x,2) + pow(temp_pt.y, 2)) << std::endl;
        }//make_result_image

        double boat_len_ave = 131;//std::accumulate(boat_len_vec.begin(), boat_len_vec.end(), 0.0) / (double)boat_len_vec.size();
        double px2m = boat_width / boat_len_ave;
        std::cout << "boat_len_ave = " << boat_len_ave << std::endl;
        std::cout << "px2m = " << px2m << std::endl;


        //浮き橋のコーナーの位置情報から浮き橋中心の位置を算出
        //jsk_recognition_msgs::PolygonArray polygon_array;//多分使わない

        for(int i = 0; i < boat_center_px_array.size(); i++){
            boat_center_px_array[i] *= px2m;
        }

        //if(this->detect_boat_pt.empty() == true){
        //    std::copy(v1.begin(), v1.end(), back_inserter(v2) );
        //}

        for(int i = 0; i < boat_center_px_array.size(); i++){

            //std::cout << "boat_center_array = " << boat_center_px_array[i] << "  angle = " << boat_angle_array[i] << std::endl;
            std::stringstream ss;
            ss << "boat_" << i;

            //トラッキング処理が必要




            tf::Transform transform;
            tf::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, boat_angle_array[i]);
            transform.setOrigin( tf::Vector3(boat_center_px_array[i].x, boat_center_px_array[i].y, 0) );
            transform.setRotation(quaternion);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), this->base_frame_name, ss.str() ));

            //浮橋の中心座標と角度を元にpolygon_arrayを作成
            /*
            geometry_msgs::PolygonStamped temp_polygon;
            temp_polygon.header.frame_id = this->base_frame_name;
            temp_polygon.header.stamp = ros::Time::now();

            geometry_msgs::Point32 point1;
            point1.x = 1;

            geometry_msgs::Point32 point2;
            geometry_msgs::Point32 point3;
            geometry_msgs::Point32 point4;
            temp_polygon.polygon.points.push_back(point1);
            temp_polygon.polygon.points.push_back(point2);
            temp_polygon.polygon.points.push_back(point3);
            temp_polygon.polygon.points.push_back(point4);

            //this->pub_polygon_array.publish(temp_polygon);
            */
        }



        //make_result_image
        cv::Mat result_image = this->color_img.clone();
        for(int bbox_count = 0; bbox_count < boat_corner_array.size(); bbox_count++){
            //検出した線分をすべて表示
            for(int i = 0; i < detected_lines.size(); i++){
                for(int j = 0; j < detected_lines[i].size(); j++){
                    cv::line(result_image, detected_lines[i][j].getStartPoint() + detected_lines_startpt[i], detected_lines[i][j].getEndPoint() + detected_lines_startpt[i], cv::Scalar(255,0,0), 1, cv::LINE_AA);
                }//for
            }//for

            for (int i = 0; i < 4; i++){
                cv::line(result_image, boat_corner_array[bbox_count][i], boat_corner_array[bbox_count][i + 1 < 4 ? i + 1 : 0], cv::Scalar(0,255,0), 2, cv::LINE_AA);
            }//for_i

            for(int i = 0; i < yolo_boat_rect.size(); i++){
                //cv::rectangle(result_image, yolo_boat_rect[i], cv::Scalar(0,0,255), 2);
            }//for

        }//make_result_image
        std::cout << "---------------------------------------" << std::endl;

        if(this->show_result == true){
            cv_bridge::CvImage img_bridge;
            sensor_msgs::Image result_img_msg;
            std_msgs::Header header; // empty header
            header.seq = this->counter; // user defined counter
            header.stamp = ros::Time::now(); // time
            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, result_image);
            img_bridge.toImageMsg(result_img_msg); // from cv_bridge to sensor_msgs::Image
            pub_result.publish(result_img_msg);
            this->counter++;
        }//show_result

        if(this->save_result == true){//Save Result Image
            time_t now = time(NULL);
            struct tm *pnow = localtime(&now);
            std::stringstream file_name;
            file_name << ros::package::getPath(this_node_name) << "/Result/" << pnow->tm_mon + 1 <<"_"<< pnow->tm_mday <<"_"<< pnow->tm_hour <<"_"<< pnow->tm_min <<"_"<< pnow->tm_sec <<".png";
            cv::imwrite(file_name.str(), result_image);
        }//save_result

    }//rgbImageCallback

private:

    ros::NodeHandle nh;
    ros::Subscriber sub_rgb;
    ros::Subscriber sub_contrl;
    ros::Subscriber sub_yolobbox;
    ros::Publisher pub_result;
    ros::Publisher pub_overlay_text;
    ros::Publisher pub_polygon_array;

    tf::TransformBroadcaster br;
    cv::Ptr<cv::line_descriptor::BinaryDescriptor> descriptor;
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

    int rect_margin;
    std::vector<cv::Point2d> detect_boat_pt;
    std::vector<double> detect_boat_angle;

};//boat_measurement


int main(int argc, char** argv){

    ros::init(argc, argv, "ribbon_bridge_lsd_node");

    Boat_Measurement bm;
    ros::spin();
    return 0;

}//main
