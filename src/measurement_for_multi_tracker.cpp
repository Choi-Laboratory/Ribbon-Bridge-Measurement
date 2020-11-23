#include "iostream"
#include "time.h"
#include "stdio.h"
#include "unistd.h"

#include "opencv2/opencv.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/opencv_modules.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/line_descriptor.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "image_transport/image_transport.h"
#include "jsk_recognition_msgs/BoundingBox.h"
#include "multi_tracker_ros_msgs/RegionOfInterest.h"
#include "multi_tracker_ros_msgs/RegionOfInterests.h"
#include "multi_tracker_ros_msgs/TrackingResult.h"
#include "multi_tracker_ros_msgs/TrackingResults.h"
#include "ribbon_bridge_measurement/RibbonBridge.h"
#include "ribbon_bridge_measurement/RibbonBridges.h"
#include "geometry_msgs/Pose2D.h"

#define PI 3.141592653589793

class MeasurementForMultiTracker{
  private:
    ros::NodeHandle nh_;

    //parameters
    std::string sub_img_topic_name_;
    bool show_result_img_flag_;
    bool pub_result_img_flag_;

    //subscribers
    ros::Subscriber sub_img_;
    ros::Subscriber sub_tracking_result_;

    //publishers
    ros::Publisher measure_result_publisher_;
    ros::Publisher result_img_publisher_;
    //image_transport::Publisher result_img_publisher_;

    //subscribeしたtracking_results
    multi_tracker_ros_msgs::TrackingResults tracking_results_;

    //
    cv_bridge::CvImagePtr cv_ptr_;
    cv::Mat color_img_;

    //lsd特徴検出器
    cv::Ptr<cv::line_descriptor::BinaryDescriptor> descriptor_ = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();

    // dictionary生成
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name_ = cv::aruco::DICT_4X4_50;
    cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_name_);

    double boat_width_ = 11.1;//12.00;
    double boat_height_ = 7.40;//8.00;
    double boat_aspect_ratio_ = boat_width_ / boat_height_; //



  public:
    MeasurementForMultiTracker(){
      //load parameters
      ros::param::get("sub_img_topic_name", sub_img_topic_name_);
      ros::param::get("show_result_image", show_result_img_flag_);
      ros::param::get("pub_result_image", pub_result_img_flag_);
      ROS_INFO("sub_img_topic_name:[%s]", sub_img_topic_name_.c_str());

      //setup subscribers
      sub_img_ = nh_.subscribe(sub_img_topic_name_, 10, &MeasurementForMultiTracker::sub_img_Callback, this);
      sub_tracking_result_ = nh_.subscribe("/multi_tracker_ros/tracking_results", 10, &MeasurementForMultiTracker::sub_tracking_result_Callback, this);

      //setup publishers
      measure_result_publisher_ = nh_.advertise<ribbon_bridge_measurement::RibbonBridges>("measure_result", 1);
      result_img_publisher_ = nh_.advertise<sensor_msgs::Image>("result_img", 10);


    }//measurement_for_multi_tracker

    void sub_img_Callback(const sensor_msgs::Image::ConstPtr& msg)
    {
      try{
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        color_img_ = cv_ptr_->image.clone();
      }//try

      catch( cv_bridge::Exception& e ){
        ROS_WARN("[cv_bridge Error] %s", e);
      }//catch
    }//sub_img_Callback

    void sub_tracking_result_Callback(const multi_tracker_ros_msgs::TrackingResults& msg)
    {
      tracking_results_ = msg;
      contours_detector();
    }//sub_tracking_result_Callback


    void contours_detector(){
      int bridge_num = tracking_results_.tracking_results.size(); //浮橋の数
      
      ribbon_bridge_measurement::RibbonBridges measure_results; //計測結果の配列

      std::cout << "==============\n";

      for( int i = 0; i < bridge_num; i++ ){
        multi_tracker_ros_msgs::TrackingResult tracking_result = tracking_results_.tracking_results[i];
        ribbon_bridge_measurement::RibbonBridge measure_result;
        
        std::string bridge_ID = tracking_result.boundingBox.header.frame_id;//浮橋のIDを取得
        
        measure_result.boat_id = std::stoi(bridge_ID);

        if( tracking_result.boundingBox.dimensions.x == 0 && tracking_result.boundingBox.dimensions.y == 0){
          continue;
        }

        try{
          //画像から浮橋の領域をトリミング
          cv::Mat trim_img(color_img_, cv::Rect(tracking_result.boundingBox.pose.position.x, tracking_result.boundingBox.pose.position.y, tracking_result.boundingBox.dimensions.x, tracking_result.boundingBox.dimensions.y));

          try{//find contours
            cv::Mat gray_img, bin_img;
            cv::cvtColor(trim_img, gray_img, CV_BGR2GRAY);

            cv::threshold(gray_img, bin_img, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
            //cv::threshold(gray_img, bin_img, 220, 255, cv::THRESH_BINARY);
            cv::erode(bin_img, bin_img, cv::Mat(), cv::Point(-1, -1), 1);//拡大縮小によるノイズ除去
        		cv::dilate(bin_img, bin_img, cv::Mat(), cv::Point(-1, -1), 1);
            bin_img = ~bin_img;

            //二値化した画像の描画（デバッグ用）
            std::string window_name = bridge_ID + "_bin";
            cv::imshow(window_name, bin_img);
            cv::waitKey(1);

            //輪郭検出
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(bin_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
            //std::cout << contours.size() << "\n"; //輪郭の候補の数

            std::vector<cv::Point2f> bridge_contours;
            for( int i = 0; i < contours.size(); i++ ){
              if( i == 0 ){
                //0番目の輪郭は画像全体になるのでパスする
                continue;
              }
              else{
                //std::cout << contours[i].size() << "\n"; //輪郭を構成する点の数
                if( contours[i].size() < 0 ){
                  //矩形を構成するには最低4つの点が必要
                  continue;
                }
                else{
                  //ある程度の面積があるものに絞る
                  double area = cv::contourArea(contours[i], false);
                  //if( area < 4500 ){
                  //  std::cout << "area_size : " << area << std::endl;
                  //  continue;
                  //}



                  //輪郭を直線近似する
                  std::vector<cv::Point> approx;
                  cv::approxPolyDP(cv::Mat(contours[i]), approx, 0.01 * cv::arcLength(contours[i], true), true);

                  //矩形以外の形状はcontinue
                  if( approx.size() != 4 ){
                    continue;
                  }

                  for( int j = 0; j < contours[i].size(); j++ ){
                    //cv::circle(trim_img, contours[i][j], 3, cv::Scalar(0, 0, 255), -1);
                    bridge_contours.push_back(contours[i][j]);
                  }

                  //cornersの点をを全て含むような矩形を推定する
                  cv::RotatedRect rc = cv::minAreaRect(bridge_contours);
                  cv::Point2f vertexes[4];
                  rc.points(vertexes);
                  std::vector<cv::Point2f> points(vertexes, vertexes + 4);

                  std::string window_name;
                  //トリムした画像の描画
                  //window_name = bridge_ID + "_trimmed_rect";
                  //cv::imshow(window_name, trim_img);
                  //cv::waitKey(1);

                  //推定した矩形の描画
                  cv::Mat rect_img = trim_img.clone();
                  for (int i = 0; i < 4; i++)
                  {
                     cv::line(rect_img,
                        vertexes[i],
                        vertexes[i + 1 < 4 ? i + 1 : 0],
                        cv::Scalar(0,255,0),
                        3,
                        cv::LINE_8
                        );
                  }
                  //window_name = bridge_ID + "_estimated_rect";
                  //cv::imshow(window_name, rect_img);
                  //cv::waitKey(1);




                  //minAreaRectで推定された矩形のコーナーに最も近いコーナーを算出
                  std::vector<cv::Point2f> subpix_corners;
                  for (int i = 0; i < points.size(); i++) {
                    double min_distance = DBL_MAX;
                    cv::Point2f min_point;
                    for (int j = 0; j < bridge_contours.size(); j++) {
                      cv::Point2f sub_point = points[i] - bridge_contours[j];
                      double distance = sqrt(pow(sub_point.x, 2) + pow(sub_point.y, 2));
                      if (distance < min_distance) { min_distance = distance; min_point = bridge_contours[j];	}//最短距離のコーナーを更新
                    }//for_j
                    subpix_corners.push_back(min_point);
                  }//for_i

                  //サブピクセル推定
                  cv::cornerSubPix(gray_img, subpix_corners, cv::Size(11, 11), cv::Size(-1, -1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

                  /*
                    検出した角の座標を左上から時計回りに並び替える
                  */
                  cv::Point2f result_corners[4];//並び替えた角の座標
                  //４つの角を左上から時計回りに並び替える
                  cv::Point2f first_top_corner, second_top_corner;
                  float first_top = 9999999.999;
                  float second_top = 9999999.999;
                  for( int i = 0; i < 4; i++ ){
                    if( subpix_corners[i].y < first_top ){
                      second_top = first_top;
                      first_top = subpix_corners[i].y;
                      second_top_corner = first_top_corner;
                      first_top_corner = subpix_corners[i];
                    }
                    else if( subpix_corners[i].y < second_top ){
                      second_top = subpix_corners[i].y;
                      second_top_corner = subpix_corners[i];
                    }
                  }

                  cv::Point2f first_bottom_corner, second_bottom_corner;
                  float first_bottom = 0.000;
                  float second_bottom = 0.000;
                  for( int i = 0; i < 4; i++ ){
                    if( subpix_corners[i].y > first_bottom ){
                      second_bottom = first_bottom;
                      first_bottom = subpix_corners[i].y;
                      second_bottom_corner = first_bottom_corner;
                      first_bottom_corner = subpix_corners[i];
                    }
                    else if( subpix_corners[i].y > second_bottom ){
                      second_bottom = subpix_corners[i].y;
                      second_bottom_corner = subpix_corners[i];
                    }
                  }

                  if( first_top_corner.x < second_top_corner.x ){
                    result_corners[0] = first_top_corner;
                    result_corners[1] = second_top_corner;
                  }
                  else{
                    result_corners[0] = second_top_corner;
                    result_corners[1] = first_top_corner;
                  }

                  if( first_bottom_corner.x < second_bottom_corner.x ){
                    result_corners[2] = second_bottom_corner;
                    result_corners[3] = first_bottom_corner;
                  }
                  else{
                    result_corners[2] = first_bottom_corner;
                    result_corners[3] = second_bottom_corner;
                  }


                  //並び替え終了


                  /* result_corners
                  0 -- 1
                  |    |
                  |    |
                  3 -- 2
                  */

                  //コーナーの描画
                  //cv::Mat corner_img = trim_img.clone();
                  //cv::circle(corner_img, result_corners[0], 5, cv::Scalar(255, 0, 255), -1);
                  //cv::circle(corner_img, result_corners[1], 5, cv::Scalar(0, 0, 255), -1);
                  //cv::circle(corner_img, result_corners[2], 5, cv::Scalar(0, 255, 255), -1);
                  //cv::circle(corner_img, result_corners[3], 5, cv::Scalar(255, 255, 0), -1);
                  //window_name = bridge_ID + "_added_corner";
                  //cv::imshow(window_name, corner_img);
                  //cv::waitKey(1);



                  //角度による閾値処理
                  cv::Point2f vector_a, vector_b;
                  double theta, degree;
                  double angle_threshold = 40.0; //±20°を許容範囲とする
                  vector_a.x = result_corners[1].x - result_corners[0].x;
                  vector_a.y = result_corners[1].y - result_corners[0].y;
                  vector_b.x = result_corners[3].x - result_corners[0].x;
                  vector_b.y = result_corners[3].y - result_corners[0].y;
                  theta = std::acos((vector_a.x*vector_a.y+vector_b.x*vector_b.y)/(sqrt(pow(vector_a.x,2)+pow(vector_a.y,2))*sqrt(pow(vector_b.x,2)+pow(vector_b.y,2))));
                  degree = theta * 180.0 / PI;
                  if( abs(90.0-degree) > angle_threshold ){
                    //ROS_WARN("ID:[%d] The angle exceeds the threshold.", std::stoi(bridge_ID));
                    //std::cout << "theta:" << theta << std::endl;
                    //std::cout << "degree:" << degree << std::endl;
                    //continue;

                    //逆の角で再検証
                    vector_a.x = result_corners[1].x - result_corners[2].x;
                    vector_a.y = result_corners[1].y - result_corners[2].y;
                    vector_b.x = result_corners[3].x - result_corners[2].x;
                    vector_b.y = result_corners[3].y - result_corners[2].y;
                    theta = std::acos((vector_a.x*vector_a.y+vector_b.x*vector_b.y)/(sqrt(pow(vector_a.x,2)+pow(vector_a.y,2))*sqrt(pow(vector_b.x,2)+pow(vector_b.y,2))));
                    degree = theta * 180.0 / PI;
                    if( abs(90.0-degree) > angle_threshold ){
                      //ROS_WARN("ID:[%d] The angle exceeds the threshold.", std::stoi(bridge_ID));
                      ROS_ERROR("ID:[%d] The angle exceeds the threshold.", std::stoi(bridge_ID));
                      std::cout << "degree:" << degree << std::endl;
                      //continue;
                    }

                  }//end if


                  /*vector_a.x = result_corners[1].x - result_corners[3].x;
                  vector_a.y = result_corners[1].y - result_corners[3].y;
                  vector_b.x = result_corners[2].x - result_corners[3].x;
                  vector_b.y = result_corners[2].y - result_corners[3].y;
                  theta = std::acos((vector_a.x*vector_a.y+vector_b.x*vector_b.y)/(sqrt(pow(vector_a.x,2)+pow(vector_a.y,2))*sqrt(pow(vector_b.x,2)+pow(vector_b.y,2))));
                  degree = theta * 180.0 / PI;
                  if( 65.0 > abs(degree) || abs(degree) > 115.0 ){
                    ROS_WARN("ID:[%d] The angle exceeds the threshold.", std::stoi(bridge_ID));
                    std::cout << "degree:" << degree << std::endl;
                    continue;
                  }*/



                  /*
                    浮橋のアスペクト比による閾値処理
                  */
                  //double result_len_1 = sqrt(pow(result_corners[2].x-result_corners[0].x,2) + pow(result_corners[2].y-result_corners[0].y,2));
                  //double result_len_2 = sqrt(pow(result_corners[1].x-result_corners[0].x,2) + pow(result_corners[1].y-result_corners[0].y,2));
                  double result_len_1 = sqrt(pow(std::max(abs(result_corners[0].x),abs(result_corners[1].x))-std::min(abs(result_corners[0].x),abs(result_corners[1].x)),2)+pow(std::max(abs(result_corners[0].y),abs(result_corners[1].y))-std::min(abs(result_corners[0].y),abs(result_corners[1].x)),2));
                  double result_len_2 = sqrt(pow(std::max(abs(result_corners[0].x),abs(result_corners[3].x))-std::min(abs(result_corners[0].x),abs(result_corners[3].x)),2)+pow(std::max(abs(result_corners[0].y),abs(result_corners[3].y))-std::min(abs(result_corners[0].y),abs(result_corners[1].x)),2));

                  double result_width = std::max(result_len_1,result_len_2);
                  double result_height = std::min(result_len_1,result_len_2);
                  double result_aspect_ratio = result_width / result_height;

                  double judge = boat_aspect_ratio_ / result_aspect_ratio;
                  if( judge < 0.85 || 1.15 < judge ){
                    std::cout << "result_aspect_ratio:" << result_aspect_ratio << std::endl;
                    std::cout << "correct_aspect_ratio:" << boat_aspect_ratio_ << std::endl;
                    ROS_ERROR("ID:[%d] The aspect_ratio exceeds the threshold.", std::stoi(bridge_ID));
                    continue;
                  }


                  //中心位置を推定
                  cv::Point2f center;
                  center.x = (result_corners[0].x + result_corners[2].x)/2.00;
                  center.y = (result_corners[0].y + result_corners[2].y)/2.00;

                  //中心の描画
                  cv::circle(trim_img, center, 5, cv::Scalar(255, 0, 255), -1);

                  //浮体の傾きを描画
                  double len_0_to_1 = sqrt(pow(result_corners[1].x-result_corners[0].x,2) + pow(result_corners[1].y-result_corners[0].y,2));
                  double len_0_to_3 = sqrt(pow(result_corners[3].x-result_corners[0].x,2) + pow(result_corners[3].y-result_corners[0].y,2));
                  double result_rad;

                  if( len_0_to_1 > len_0_to_3 ){
                    cv::line(trim_img, center, cv::Point((result_corners[0].x+result_corners[1].x)/2.00,(result_corners[0].y+result_corners[1].y)/2.00), cv::Scalar(255,0,0), 3, 8, 0);
                    cv::line(trim_img, center, cv::Point((result_corners[1].x+result_corners[2].x)/2.00,(result_corners[1].y+result_corners[2].y)/2.00), cv::Scalar(0,0,255), 3, 8, 0);
                    result_rad = std::atan2(result_corners[1].y-result_corners[0].y, result_corners[1].x-result_corners[0].x);
                  }
                  else{
                    cv::line(trim_img, center, cv::Point((result_corners[0].x+result_corners[1].x)/2.00,(result_corners[0].y+result_corners[1].y)/2.00), cv::Scalar(0,0,255), 3, 8, 0);
                    cv::line(trim_img, center, cv::Point((result_corners[0].x+result_corners[3].x)/2.00,(result_corners[0].y+result_corners[3].y)/2.00), cv::Scalar(255,0,0), 3, 8, 0);
                    result_rad = std::atan2(result_corners[3].y-result_corners[0].y, result_corners[3].x-result_corners[0].x);
                  }

                  //浮橋の角度を計算
                  double result_deg = result_rad * 180.0 / PI;

                  //画像全体における浮橋の中心位置を求める
                  center.x = center.x + tracking_result.boundingBox.pose.position.x;
                  center.y = center.y + tracking_result.boundingBox.pose.position.y;

                  std::cout << "ID:" << bridge_ID << "\n";
                  std::cout << "center:" << center << "\n";
                  std::cout << "degree:" << 90.0+result_deg << "\n";
                  //std::cout << "radian:" << result_rad << "\n";
                  std::cout << "---\n";

                  //計測結果を格納
                  measure_result.center.x = center.x;
                  measure_result.center.y = center.y;
                  measure_result.center.theta = degree;
                  for( int i = 0; i < 4; i++ ){
                    geometry_msgs::Pose2D corner_pose;
                    corner_pose.x = result_corners[i].x;
                    corner_pose.y = result_corners[i].y;
                    measure_result.corners.push_back(corner_pose);
                  }
                  measure_results.RibbonBridges.push_back(measure_result);

                  //尤もらしい浮橋が見つかったのでループを抜ける
                  break;
                }//end else
              }// end else
            }// end for i

            //計測結果をpublish
            if( measure_results.RibbonBridges.size() != 0 ){
              measure_result_publisher_.publish(measure_results);
            }

          }//end try find contours

          catch(...){
            ROS_WARN("error");
          }//end catch

          //trimした浮橋の画像の描画
          std::string window_name = bridge_ID;
          cv::imshow(window_name, trim_img);
          cv::waitKey(1);

        }//end try triming

        catch(...){
          ROS_WARN("Faild to trim");
          return;
        }// end catch triming

      }//end for

      if( show_result_img_flag_ == true ){
        cv::Mat show_image;
        cv::resize(color_img_, show_image, cv::Size(), 1.0, 1.0);
        cv::imshow("result", show_image);
        cv::waitKey(1);
      }//endif
    }//end contours_detector
//-----------------------------------

};//class MeasurementForMultiTracker


int main(int argc, char** argv)
{
  ros::init(argc, argv, "measurement_for_multi_tracker_node");
  ROS_INFO("measurement_for_multi_tracker START");

  MeasurementForMultiTracker mmt;

  ros::spin();
}
