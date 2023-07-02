/******** file(s) not yet adapted to new version of UGAS ********/

/********************* BuffIdentifier_V1.h **********************/

//#pragma once
///*
//Creation Date:
//Latest Update:
//Developer(s):
//(C)Copyright: NJUST.Alliance - All rights reserved
//Header Functions:
//- power rune identify
//* [ImgPretreat from UGAS]
//*       |
//* [  Identify  ]
//*       |
//* [    Fit    ]
//*       |
//* [  Predict  ]
//*       |
//* [Point3f to Trajectory]
//* update list: 
//* - Ensure target point stability when identifying two sets of blades
//* ignore list: 
//*/
//#include "../BuffIdentifier.h"
//#include "../LM_Fitting/LM_Fitting.h"
//#include "../EKF/EKF.h"
//#include "../../Common/Filter/Filter.h"
//#include "../../Common/PnP/PnP.h"
//
//// The current(V1.h) is modified from the new(V2.h), there may be unused parts.
//class BuffIdentifier_V1 : public BuffIdentifier {
//private:
//    LM_fitting _lm;
//    BuffFitData _fd;
//    BuffAngle _angle, _last_angle;
//    BuffAngularSpeed _angularSpeed;
//    filters::linear::Linear_E<double, 1> _speedFilter;
//    ArmorPlate _armorPlate;
//    cv::Point2f _armorCenter; // predict hit point
//    cv::Point2f _rCenter;
//    double _radius;
//    TimeStamp _latest_time;
//    BuffMode _nowMode;
//    bool _armor_valid;
//    bool _data_valid;
//    bool _fit_valid;
//
//    // verify the hit point
//    cv::Point2f _historyShotPoint;
//    cv::Point2f _verifiedPoint;
//    TimeStamp _historyShotTime;
//    TimeStamp _verifiedTime;
//
//    void Reset() {
//        _angle = BuffAngle(-1, -1);
//        _last_angle = BuffAngle(-1, -1);
//        _angularSpeed = BuffAngularSpeed(-1, -1);
//        _speedFilter.Reset();
//        _armorCenter = cv::Point(-1, -1);
//        _rCenter = cv::Point(-1, -1);
//        _radius = -1;
//        _latest_time = -1;
//        _nowMode = UnknownBuff;
//        _armor_valid = false;
//        _data_valid = false;
//        _fit_valid = false;
//
//        _historyShotPoint = cv::Point(-1, -1);
//        _verifiedPoint = cv::Point(-1, -1);
//        _historyShotTime = -1;
//        _verifiedTime = -1;
//    }
//    void GetAngularSpeed(const Img& img, const Img& imgThre);
//    int getlayers(std::vector<std::vector<cv::Point>> contours,
//        std::vector<cv::Vec4i> hierarchy, int i) {
//        // get the number of sub contours
//        int num = 1;
//        if (hierarchy[i][2] == -1) return 0;
//        else {
//            i = hierarchy[i][2];
//            for (; hierarchy[i][0] != -1;) {
//                num++;
//                i = hierarchy[i][0];
//            }
//            return num;
//        }
//    }
//    void drawRotatedRect(cv::RotatedRect rr, cv::Scalar color,
//        bool draw_center = 0) {
//        cv::Point2f points[4];
//        rr.points(points);
//        if (draw_center)
//            cv::circle(debugImg, cv::Point(rr.center.x, rr.center.y), 2, color, -1, 8);
//        for (int j = 0; j < 4; j++)
//            cv::line(debugImg, points[j], points[(j + 1) % 4], color, 2, 8);
//    }
//    void Fit();
//    cv::Point3f Predict(TimeStamp ts);
//public:
//    BuffIdentifier_V1() : BuffIdentifier() {
//        Reset();
//    }
//    void Identify(const Img& img, const Img& imgThre) {
//        GetAngularSpeed(img, imgThre);
//        Fit();
//        cv::Point3f p = Predict(200);
//        //MAKE_GRAGH_DEFAULT
//        //	  GRAGH_ADD_VAR(p.x, COLOR_RED)
//        //    GRAGH_ADD_VAR(p.y, COLOR_GREEN)
//        //    GRAGH_ADD_VAR(p.z, COLOR_BLUE)
//        //SHOW_GRAGH(Buff_Shot3D)
//        //cv::circle(debugImg, PnPsolver.RevertPnP(p), 2, COLOR_GREEN, 3, 8);
//        //cout << p << endl;
//    };
//};

/******************** BuffIdentifier_V1.cpp *********************/

//#include "BuffIdentifier_V1.h"
//
//void BuffIdentifier_V1::GetAngularSpeed(const Img& img, const Img& imgThre) {
//	double max_R_ratio = 3;	// Maximum R ratio
//	int minBuffSize = 200;	// Minimum buff area
//	int maxBuffSize = 7500;	// Maximum buff area
//	int min_R_Size = 100;	// Minimum R area
//	int max_R_Size = 1000;	// Maximum R area
//	double multiple = 4;	// Predicted distance multiple between R and armor plate TRY:4/5.5
//	int centerError = 50;	// Allowable difference between predicted R and actual
//	int armorError = 150;	// The allowed difference between the current armor plate and the previous armor plate
//
//	static TimeStamp invalid_t = 0;
//	static double valid_a = 0.0;
//	static double valid_v = 0.0;
//
//	_latest_time = img.timeStamp;
//	double v = 0., angle = 0;
//	vector<vector<cv::Point>> contours;
//	vector<cv::Vec4i> hierarchy;
//
//	findContours(imgThre, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
//	//findContours(imgThre, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
//	//drawContours(img, contours, -1, Scalar(255, 255, 0), 1);
//
//	// Find armor plate
//	for (int i = 0; i < contours.size(); i++) {
//		//if (hierarchy[i][1] == -1 && hierarchy[i][0] == -1 &&hierarchy[i][2]==-1)
//		if (getlayers(contours, hierarchy, i) == 1) {
//			i = hierarchy[i][2];
//			float area = contourArea(contours[i]);
//			if (area < minBuffSize || area > maxBuffSize) continue;
//			cv::RotatedRect armorRect = minAreaRect(contours[i]);
//			//RotatedRect outsideRect = minAreaRect(contours[hierarchy[i][3]]);
//
//			//cout << P2PDis(armorRect.center, _armorCenter) << endl;
//			if (_armor_valid && P2PDis(armorRect.center, _armorCenter) > armorError) {
//				_armor_valid = false;
//				break;
//			}
//			_armor_valid = true;
//
//			cv::Point2f points[4];
//			armorRect.points(points);
//			//for (int i = 0; i < 4; i++) {
//			//	cv::putText(debugImg, to_string(i), points[i], cv::FONT_HERSHEY_SIMPLEX, 1, COLOR_GREEN);
//			//}
//			// The direction of the armor plate was not determined during construction (the translation amount is correct, and the rotation amount may be incorrect)
//			// Waring: When constructing LB, an angle is required, and the angle entered here is not a true value
//			if (P2PDis(points[0], points[1]) > P2PDis(points[1], points[2])) {
//				_armorPlate = ArmorPlate(LightBar(points[0], points[3], 0), LightBar(points[1], points[2], 0));
//			}
//			else {
//				_armorPlate = ArmorPlate(LightBar(points[1], points[0], 0), LightBar(points[2], points[3], 0));
//			}
//
//			// Draw armor plate: Draw the center point and each edge of armorRect
//			circle(debugImg, cv::Point(armorRect.center.x, armorRect.center.y), 2, COLOR_GREEN, -1, 8);
//			for (int j = 0; j < 4; j++)
//				line(debugImg, points[j], points[(j + 1) % 4], COLOR_GREEN, 2, 8);
//
//			//break;
//
//			//if (cv::contourArea(contours[hierarchy[i][3]]) > shotSize)
//				//continue; //shot
//
//			// Find R
//			cv::Point2i	cen1 = armorRect.center + multiple * (points[0] - points[1]),// ¨K
//				cen2 = armorRect.center + multiple * (points[1] - points[0]),// ¨I
//				cen3 = armorRect.center + multiple * (points[1] - points[2]),// ¨L
//				cen4 = armorRect.center + multiple * (points[2] - points[1]);// ¨J
//			//circle(debugImg, cen1, centerError, COLOR_GREEN, 1, 8);
//			//circle(debugImg, cen2, centerError, COLOR_GREEN, 1, 8);
//			//circle(debugImg, cen3, centerError, COLOR_GREEN, 1, 8);
//			//circle(debugImg, cen4, centerError, COLOR_GREEN, 1, 8);
//
//			for (auto& contour : contours) {
//				cv::Rect R_rect = boundingRect(contour);
//
//				double ratio = (double)R_rect.height / R_rect.width;
//				if (ratio < (1.0 / max_R_ratio) || ratio > max_R_ratio) continue;
//
//				int rectSize = R_rect.area();
//				if (rectSize < min_R_Size || rectSize > max_R_Size) continue;
//
//				cv::Point2i rectCenter = (R_rect.tl() + R_rect.br()) / 2;
//				if (P2PDis(rectCenter, cen1) < centerError || P2PDis(rectCenter, cen2) < centerError ||
//					P2PDis(rectCenter, cen3) < centerError || P2PDis(rectCenter, cen4) < centerError)
//				{
//					rectangle(debugImg, R_rect, COLOR_GREEN, 1, 8);
//					//	cout << "size: " << rectSize << endl;
//					//	cout << "ratio: " << ratio << endl;
//
//					// Convert to polar coordinate system to obtain angles and radii
//					cv::Point2i R_center = rectCenter;
//					double radius = P2PDis(R_center, armorRect.center);
//					double tan_angle = (armorRect.center.y - R_center.y) / (armorRect.center.x - R_center.x);
//					angle = atan(tan_angle);
//					if (armorRect.center.y - R_center.y > 0) {
//						if (armorRect.center.x - R_center.x < 0)
//							angle += PI;
//						else if (armorRect.center.x - R_center.x == 0)
//							angle = PI / 2;
//					}
//					else if (armorRect.center.y - R_center.y < 0) {
//						if (armorRect.center.x - R_center.x > 0)
//							angle += 2 * PI;
//						else if (armorRect.center.x - R_center.x < 0)
//							angle += PI;
//						else if (armorRect.center.x - R_center.x == 0)
//							angle = 1.5 * PI;
//					}
//					else {
//						if (armorRect.center.x - R_center.x < 0)
//							angle = PI;
//						else
//							angle = 0.0;
//					}
//					//cout << "radius: " << radius << endl;
//					//cout << "angle: " << angle / PI*180 << endl;
//					//cout << "angle: " << angle << endl;
//					if (angle > 1e-6 && angle < 7) {
//						valid_a = angle;
//					}
//					// Find angular velocity
//					if (_last_angle.timeStamp != -1) {
//						double delta_angle = angle - _last_angle.angle;
//						if (delta_angle > 6) delta_angle -= 2 * PI;
//						else if (delta_angle < -6) delta_angle += 2 * PI;
//						TimeStamp delta_time = _latest_time - _last_angle.timeStamp;
//						v = 1000 * delta_angle / delta_time;
//						//cout << "time: " << delta_time << "\t" << "angular velocity: " << delta_angle << endl;
//						//Filtering speed
//						if (abs(v) > 0.01 && abs(v) < 4) {
//							if (_latest_time - invalid_t > 20) { // According to the previous instability exceeding 10ms
//								_data_valid = true;
//								valid_v = _speedFilter.Predict(v);
//								_angularSpeed = BuffAngularSpeed(_latest_time, valid_v);
//							}
//						}
//						else {
//							invalid_t = _latest_time;
//							_data_valid = false;
//						}
//					}
//					if (abs(angle) > 1e-6) valid_a = angle;
//					_last_angle = _angle;
//					_angle = BuffAngle(_latest_time, valid_a);
//					_armorCenter = armorRect.center;
//					_rCenter = R_center;
//					_radius = radius;
//					break;
//				}
//			}
//		}
//	}
//	//MAKE_GRAGH_DEFAULT
//	//	GRAGH_ADD_VAR(valid_a, COLOR_GREEN)
//	//	//if (abs(v) < 0.1 || abs(v) > 4) v = 0;
//	//	//GRAGH_ADD_VAR(v, COLOR_PINK)
//	//	GRAGH_ADD_VAR(valid_v, COLOR_RED)
//	//SHOW_GRAGH(Gragh_Buff)
//
//	//LOG(INFO) << "\r" << time << "    " << valid_v << "                                " <<endl;
//	//cout << time << endl << endl << valid_v << endl << endl;
//}
//
//void BuffIdentifier_V1::Fit() {
//	//*/ Version 2 time-triggered
//	if (_data_valid) {
//		_lm.push(_angularSpeed);
//
//		if (_fit_valid == false || _angularSpeed.timeStamp - _fd.FitTime > 1000) {
//			_fd = _lm.Fitting();
//			_fit_valid = true; //Only fitted, but accuracy not test and verify
//			//cout << _fd.FitTime << setprecision(5) <<
//			//	"\t" << _fd.A << "\t" << _fd.B << "\t" << _fd.C << "\t" << _fd.D << endl;
//		}
//	}
//	/*/// Version 1 count-triggered
//		static int counter = 1;
//		if (_data_valid) {
//			lm.push(_angularSpeed);
//
//			if (!counter) {
//				_fd = _lm.fitting();
//				_fit_valid = true;
//				//cout << _fd.FitTime << setprecision(5) <<
//				//	"\t" << _fd.A << "\t" << _fd.B << "\t" << _fd.C << "\t" << _fd.D << endl;
//				counter = 1; //Modify trigger frame num
//			}
//			counter--;
//		}
//	//*/
//}
//
//cv::Point3f BuffIdentifier_V1::Predict(TimeStamp ts) {
//	double angle = 0;
//	double delta_angle = 0;
//	double latest_time = (double)_latest_time / 1000;
//	double calc_time = latest_time + (double)ts / 1000;
//
//	if (_angle.timeStamp != -1) angle += _angle.angle;
//	if (_fit_valid) {
//		delta_angle = _lm.PredictDeltaAngle(_latest_time, ts);
//	}
//	angle += delta_angle;
//	if (angle > 2 * PI) angle -= 2 * PI;
//	else if (angle < 0) angle += 2 * PI;
//	//cout << angle << "\t" << sin(angle) << "\t" << cos(angle) << endl;
//
//	_armorCenter = cv::Point2f(cos(angle), sin(angle));
//	_armorCenter *= _radius;
//	_armorCenter += _rCenter;
//
//	cv::circle(debugImg, _rCenter, 2, COLOR_RED, 3, 8);
//
//	// Verify the hit point
//	//if (_latest_time - _verifiedTime > 1100) {
//	//	if (_latest_time - _historyShotTime > ts) {
//	//		_historyShotPoint = _armorCenter;
//	//		_historyShotTime = _latest_time;
//	//	}
//	//	_verifiedPoint = Point2f(cos(_angle.angle), sin(_angle.angle));
//	//	_verifiedPoint *= _radius;
//	//	_verifiedPoint += _rCenter;
//
//	//	if (_latest_time - _verifiedTime > 1100 + ts) {
//	//		_verifiedTime = _latest_time;
//	//	}
//	//}
//	//cv::circle(debugImg, _historyShotPoint, 5, COLOR_PINK, 3, 8);
//	//cv::circle(debugImg, _verifiedPoint, 5, COLOR_GREEN, 3, 8);
//	cv::circle(debugImg, _armorCenter, 5, COLOR_RED, 3, 8);
//
//	//MAKE_GRAGH_DEFAULT
//	//	GRAGH_ADD_VAR(_angularSpeed.speed * ts / 1000, COLOR_RED)
//	//	GRAGH_ADD_VAR(_lm.PredictDeltaAngle(_latest_time, ts), COLOR_PINK)
//	//SHOW_GRAGH(Buff_Shot)
//
//	return PnPsolver.SolvePnP(_armorPlate);
//}
