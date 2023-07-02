#pragma once
/*
Creation Date: 2023/5/22
Latest Update: 2023/5/22
Developer(s): 21-WZY 21-WTR
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- power rune identifier
*/

//#pragma warning(disable : 4996)

#include <optional>
#include <string>

#include <opencv2/highgui/highgui_c.h>

#include "Core/Identifier/BuffIdentifierInterface.h"
#include "Core/Identifier/Color/ColorIdentifier_V1.h"
#include "Util/Debug/DebugCanvas.h"
#include "Util/UtilFunctions.h"


class BuffIdentifier_V2 {
private:
	TimeStamp _latest_time;
	BuffAngle _angle, _last_angle;
	BuffAngularSpeed _angularSpeed;
	cv::Point2f _leftPoint;
	cv::Point2f _rightPoint;
	cv::Point2f _rCenter;
	double _radius;
	
	bool _data_valid;

	cv::VideoWriter _videoWriter;
	int _coder;

	int getlayers(std::vector<std::vector<cv::Point>> contours,
				  std::vector<cv::Vec4i> hierarchy, int i) 
	{
		// get the number of sub contours
		int num = 1;
		if (hierarchy[i][2] == -1) return 0;
		else {
			i = hierarchy[i][2];
			for (; hierarchy[i][0] != -1;) {
				num++;
				i = hierarchy[i][0];
			}
			return num;
		}
	}
	void drawRotatedRect(cv::RotatedRect rr, cv::Scalar color,
						 int line_thickness = 2, bool draw_center = 0)
	{
		if constexpr (debugCanvas.buff) {
			cv::Point2f points[4];
			rr.points(points);
			if (draw_center)
				cv::circle(debugCanvas.buff.GetMat(), cv::Point(rr.center.x, rr.center.y), 2, color, -1, 8);
			for (int j = 0; j < 4; j++)
				cv::line(debugCanvas.buff.GetMat(), points[j], points[(j + 1) % 4], color, line_thickness, 8);
		}
	}

public:
	BuffIdentifier_V2() {
		//std::string _vwFilename = getTime() + ".mp4";
		//double _vwFps = 30.0;

		//_coder = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
		//_videoWriter.open(_vwFilename, _coder, _vwFps, cv::Size(120, 720), true);
		//if (!_videoWriter.isOpened()) {
		//	throw_with_trace(std::runtime_error, "Video writer open failed!")
		//	//std::cout << "Video writer open failed" << std::endl;
		//}
	}

	BuffIdentifier_V2(const BuffIdentifier_V2&) = delete;
	BuffIdentifier_V2(BuffIdentifier_V2&&) = delete;

	~BuffIdentifier_V2() {
		//_videoWriter.release();
	}

	std::optional <BuffIdentifyData> Identify(const cv::Mat& img, const TimeStamp& timeStamp) {
		// length-width ratio
		const double buff_R_Ratio = 1.3;
		const double buff_Upper_MinRatio = 2.2;
		const double buff_Upper_MaxRatio = 2.8;
		const double buff_Bed_MinRatio = 1.2;
		const double buff_Bed_MaxRatio = 1.4;
		// area multiplier (compared to the preset area dis * dis)
		const double buff_Bed_MinAreaMultiple = 0.2;
		const double buff_Bed_MaxAreaMultiple = 0.6;
		// angle difference
		const double buff_UB_AngleDif = 5.0;
		// distance multiplier (compared to the preset radius pixel length dis)
		const double buff_UB_MinDisMultiple = 0.4;
		const double buff_UB_MaxDisMultiple = 0.7;
		const double buff_R_DisMultiple = 1.175;
		const double buff_R_ErrMultiple = 0.2;
		const double buff_Radius_Upper = 2.095; // radius / long side of the "Upper" = 2.095
		static int dis = 260; // history: 200, 260

		static TimeStamp invalid_t = 0;
		static double valid_a = 0;
		static double valid_v = 0;

		//_videoWriter.write(img);
		_latest_time = timeStamp;
		double v = 0, angle = 0;
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::RotatedRect> rects;
		std::vector<cv::Vec4i> hierarchy;
		std::vector<int> R_Index;
		std::vector<int> Upper_Index;
		std::vector<int> Bed_Index;

		// pretreat
		cv::Mat imgThre;
		cvtColor(img, imgThre, cv::COLOR_BGR2HSV);
		if (true) { // color = [red] / blue
			inRange(imgThre, cv::Scalar(30, 0, 220),
							 cv::Scalar(40, 255, 255), imgThre);
		}
		else { // color = red / [blue]
			inRange(imgThre, cv::Scalar(80, 40, 220),
							 cv::Scalar(100, 255, 255), imgThre);
		}
		cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
		cv::dilate(imgThre, imgThre, dilate_kernel);
		cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
		cv::morphologyEx(imgThre, imgThre, cv::MORPH_CLOSE, close_kernel);
		cv::imshow("imgThre", imgThre);
		cv::waitKey(100);

		// filter upper light, bed light, and R light
		findContours(imgThre, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
		//std::cout << std::endl;
		for (int i = 0; i < contours.size(); i++) {
			rects.push_back(minAreaRect(contours[i]));
			if (hierarchy[i][2] == -1) {
				float ratio = (float)rects.back().size.width / rects.back().size.height;
				if (ratio < 1.0) ratio = 1 / ratio;
				
				//std::cout << rects.back().size.width * rects.back().size.height << " - " << ratio << std::endl;

				if (ratio > buff_Upper_MinRatio && ratio < buff_Upper_MaxRatio) {
					if constexpr (debugCanvas.buff) {
						cv::circle(debugCanvas.buff.GetMat(), rects[i].center, 3, COLOR_LIGHTBLUE, -1);
					}
					Upper_Index.push_back(i);
				}
				else if (ratio > buff_Bed_MinRatio && ratio < buff_Bed_MaxRatio) {
					int area = rects.back().size.width * rects.back().size.height;
					if (area > buff_Bed_MinAreaMultiple * dis * dis &&
						area < buff_Bed_MaxAreaMultiple * dis * dis) {
						if constexpr (debugCanvas.buff) {
							cv::circle(debugCanvas.buff.GetMat(), rects[i].center, 3, COLOR_BLUE, -1);
						}
						Bed_Index.push_back(i);
					}
					//if constexpr (debugCanvas.buff) {
					//cv::circle(debugCanvas.buff.GetMat(), rects[i].center, 3, COLOR_BLUE, -1);
					//}
					Bed_Index.push_back(i);
				}
				else if (ratio < buff_R_Ratio) {
					R_Index.push_back(i);
				}
				else {
					std::stringstream str;
					str << rects.back().size.width * rects.back().size.height << " - " << ratio;
					if constexpr (debugCanvas.buff) {
						//drawRotatedRect(rects[i], COLOR_GREEN, 1);
						cv::putText(debugCanvas.buff.GetMat(), toString(str), rects.back().center, cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_GREEN);
					}
				}
			}
		}
		// match Upper light & Bed light, find R light, get radius and angle
		for (auto& i : Upper_Index) {
			for (auto& j : Bed_Index) {
				//drawRotatedRect(rects[i], COLOR_LIGHTBLUE);
				//drawRotatedRect(rects[j], COLOR_BLUE);

				double delta_angle = abs(rects[i].angle - rects[j].angle);
				//cout << (int)rects[i].angle << " \t" << (int)rects[j].angle << "\t" << delta_angle << endl;
				if (delta_angle > buff_UB_AngleDif &&
					delta_angle < (90 - buff_UB_AngleDif)) continue;
				drawRotatedRect(rects[i], COLOR_RED);
				drawRotatedRect(rects[j], COLOR_GREEN);

				int buff_TD_dis = P2PDis(rects[i].center, rects[j].center);
				//cout << buff_UB_dis << endl;
				if (buff_TD_dis < buff_UB_MinDisMultiple * dis ||
					buff_TD_dis > buff_UB_MaxDisMultiple * dis) continue;

				dis = buff_Radius_Upper * (std::max)(rects[i].size.height, rects[i].size.width);
				//cout << dis << endl;

				//angle = atan2(rects[i].center.y - rects[j].center.y,
				//			    rects[i].center.x - rects[j].center.x);
				//angle = _angleFilter.Predict(angle);
				//cout << angle << endl;

				// R radius and angle
				cv::Point2f	R_Predict = rects[i].center +
					buff_R_DisMultiple * 1.73 * (rects[j].center - rects[i].center);
				for (auto& k : R_Index) {
					//if constexpr (debugCanvas.buff) {
					//	cv::circle(debugCanvas.buff.GetMat(), R_Predict, buff_R_ErrMultiple * dis, COLOR_RED, 1);
					//}

					if (P2PDis(rects[k].center, R_Predict) > buff_R_ErrMultiple * dis) continue;
					//if constexpr (debugCanvas.buff) {
					//	cv::circle(debugCanvas.buff.GetMat(), rects[k].center, 0.1 * dis, COLOR_RED, 1);
					//	rectangle(debugCanvas.buff.GetMat(), rects[k].boundingRect(), COLOR_RED, 1);
					//}

					angle = atan2(rects[i].center.y - rects[k].center.y,
								  rects[i].center.x - rects[k].center.x);
					cv::Point2f topPoints[4];
					cv::Point2f downPoints[4];
					cv::Point2f topLeft, topRight, downLeft, downRight;
					rects[i].points(topPoints);
					rects[j].points(downPoints);
					if (angle <= -MathConsts::Pi * 0.8) {
						if (rects[i].angle > 45) {
							topLeft = topPoints[2];
							topRight = topPoints[1];
						}
						else {
							topLeft = topPoints[3];
							topRight = topPoints[2];
						}
						if (rects[j].angle > 45) {
							downLeft = downPoints[3];
							downRight = downPoints[0];
						}
						else {
							downLeft = downPoints[0];
							downRight = downPoints[1];
						}
					}
					else if (angle <= -MathConsts::Pi * 0.7) {
						topLeft = topPoints[3];
						topRight = topPoints[2];
						downLeft = downPoints[0];
						downRight = downPoints[1];
					}
					else if (angle <= -MathConsts::Pi * 0.3) {
						if (rects[i].angle > 45) {
							topLeft = topPoints[3];
							topRight = topPoints[2];
						}
						else {
							topLeft = topPoints[0];
							topRight = topPoints[3];
						}
						if (rects[j].angle > 45) {
							downLeft = downPoints[0];
							downRight = downPoints[1];
						}
						else {
							downLeft = downPoints[1];
							downRight = downPoints[2];
						}
					}
					else if (angle <= -MathConsts::Pi * 0.2) {
						topLeft = topPoints[0];
						topRight = topPoints[3];
						downLeft = downPoints[1];
						downRight = downPoints[2];
					}
					else if (angle <= MathConsts::Pi * 0.2) {
						if (rects[i].angle > 45) {
							topLeft = topPoints[0];
							topRight = topPoints[3];
						}
						else {
							topLeft = topPoints[1];
							topRight = topPoints[0];
						}
						if (rects[j].angle > 45) {
							downLeft = downPoints[1];
							downRight = downPoints[2];
						}
						else {
							downLeft = downPoints[2];
							downRight = downPoints[3];
						}
					}
					else if (angle <= MathConsts::Pi * 0.3) {
						topLeft = topPoints[1];
						topRight = topPoints[0];
						downLeft = downPoints[2];
						downRight = downPoints[3];
					}
					else if (angle <= MathConsts::Pi * 0.7) {
						if (rects[i].angle > 45) {
							topLeft = topPoints[1];
							topRight = topPoints[0];
						}
						else {
							topLeft = topPoints[2];
							topRight = topPoints[1];
						}
						if (rects[j].angle > 45) {
							downLeft = downPoints[2];
							downRight = downPoints[3];
						}
						else {
							downLeft = downPoints[3];
							downRight = downPoints[0];
						}
					}
					else if (angle <= MathConsts::Pi * 0.8) {
						topLeft = topPoints[2];
						topRight = topPoints[1];
						downLeft = downPoints[3];
						downRight = downPoints[0];
					}
					else {
						if (rects[i].angle > 45) {
							topLeft = topPoints[2];
							topRight = topPoints[1];
						}
						else {
							topLeft = topPoints[3];
							topRight = topPoints[2];
						}
						if (rects[j].angle > 45) {
							downLeft = downPoints[3];
							downRight = downPoints[0];
						}
						else {
							downLeft = downPoints[0];
							downRight = downPoints[1];
						}
					}
					_leftPoint = (topLeft + downLeft) / 2;
					_rightPoint = (topRight + downRight) / 2;
					_rCenter = rects[k].center;
					_radius = P2PDis(rects[k].center, rects[i].center) * 0.86; // Dis(upper, center) / radius = 0.86
					break;
				}
			}
		}
		if (_last_angle.timeStamp != -1) {
			double delta_angle = angle - _last_angle.angle;
			if (delta_angle > 6) delta_angle -= 2 * MathConsts::Pi;
			else if (delta_angle < -6) delta_angle += 2 * MathConsts::Pi;
			TimeStamp delta_time = _latest_time - _last_angle.timeStamp;
			v = 1000 * delta_angle / delta_time;
			//cout << "time: " << delta_time << "\t" << "angular speed: " << delta_angle << endl;
			// filt speed
			if (abs(v) > 0.01 && abs(v) < 7) {
				if (_latest_time - invalid_t > 10) { //over 10ms since last tremble
					_data_valid = true;
					valid_v = v;
					_angularSpeed = BuffAngularSpeed(_latest_time, valid_v);
				}
			}
			else {
				invalid_t = _latest_time;
				_data_valid = false;
				return std::nullopt;
			}
		}
		//cout << valid_v << endl;

		if (abs(angle) > 1e-6) valid_a = angle;
		_last_angle = _angle;
		_angle = BuffAngle(_latest_time, valid_a);

		return BuffIdentifyData(_leftPoint, _rightPoint, _rCenter, _radius, 
								_angle, _angularSpeed);
	}

	std::string toString(std::ostream& str)	{
		std::ostringstream ss;
		ss << str.rdbuf();
		return ss.str();
	}

	//std::string getTime() {
	//	time_t timep;
	//	time(&timep);
	//	char tmp[64];
	//	strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&timep));
	//	return tmp;
	//}
};
