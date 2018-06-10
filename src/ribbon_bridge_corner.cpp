#include "UseOpenCV.h"
#include <iostream>
#include <vector>

class Combination {

private:

	std::vector<std::vector<int>> combination;

	void sub_func(int n, int r, int last_num, std::vector<int> input_vec) {

		/*if (input_vec.empty() == false){
			if (input_vec[input_vec.size() - 1] + (r - input_vec.size()) > n) {
				return;
			}
		}*/

		for (int i = last_num; i < n; i++) {

			std::vector<int> temp_vec = input_vec;
			temp_vec.push_back(i);

			if (temp_vec.size() == r) {
				this->combination.push_back(temp_vec);
			}
			else {
				this->sub_func(n, r, (i + 1), temp_vec);
			}
		}//for

	}//sub_func

public:

	std::vector<std::vector<int>> calc_combination(int n, int r) {

		this->combination.clear();
		this->sub_func(n, r, 0, std::vector<int>());//組み合わせの算出
		return this->combination;

	}//make_combi

};


int main() {


	cv::Mat image;
	cv::VideoCapture cap = cv::VideoCapture(0);


	//cv::Ptr<cv::Feature2D> akaze = cv::AKAZE::create();
	//std::vector<cv::KeyPoint> keypoint;
	//cv::Mat desk;
	//akaze->detectAndCompute(bin_img, cv::Mat(), keypoint, desk, false);
	//cv::drawKeypoints(image, keypoint, image, cv::Scalar(0, 255, 0), 4);


	while (true) {

		cap >> image;
		
		cv::Mat gray_img;
		cv::Mat bin_img;
		cv::cvtColor(image, gray_img, CV_BGR2GRAY);
		cv::normalize(gray_img, gray_img, 0, 255, cv::NORM_MINMAX);
		
		cv::threshold(gray_img, bin_img, 220, 255, cv::THRESH_BINARY);//2値価
		cv::erode(bin_img, bin_img, cv::Mat(), cv::Point(-1, -1), 1);//ノイズ除去
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

			//pointsに一番近いコーナーを検出してサブピクセル推定、描画
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
					}
				}//for_j
				subpix_corners.push_back(min_point);
			}//for_i
			cv::cornerSubPix(gray_img, subpix_corners, cv::Size(11, 11), cv::Size(-1, -1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			
			//中心座標の推定
			cv::Point2f center;
			double a1 = (subpix_corners[2].y - subpix_corners[0].y) / (subpix_corners[2].x - subpix_corners[0].x);
			double a2 = (subpix_corners[3].y - subpix_corners[1].y) / (subpix_corners[3].x - subpix_corners[1].x);
			center.x = (a1 * subpix_corners[0].x - subpix_corners[0].y - a2 * subpix_corners[1].x + subpix_corners[1].y) / (a1 - a2);
			center.y = (subpix_corners[2].y - subpix_corners[0].y) / (subpix_corners[2].x - subpix_corners[0].x)*(center.x - subpix_corners[0].x) + subpix_corners[0].y;
			cv::circle(result_img, center, 8, cv::Scalar(255, 0, 0));

			//angleの算出
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