#pragma once
/*
Creation Date: 2023/5/22
Latest Update: 2023/7/4
Developer(s): 21-WZY 21-WTR 22-IrumaMegumi
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- buff coordinate predictor
- update gyro data source
*/

#include <vector>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "Core/Predictor/BuffPredictorInterface.h"
#include "Core/PnPSolver/Buff/BuffPnPSolver_V1.h"
#include "./Filter/Filter.h"
#include "./Ceres/Ceres_V1.h"
#include "./Ceres/Ceres_V2.h"
#include "./EKF/EKF.h"
#include "./LM/LM.h"


class BuffPredictor_V1 {
private:
    filters::linear::Linear_E<double, 3> _speedFilter;
	Ceres_V1 _ceres;
	//EKF _ekf;
	//LM _lm;

	BuffMode _mode;

	BuffPnPSolver_V1 buffPnPSolver;

    TimeStamp _latest_time;
    TimeStamp _last_time;
	float _last_speed;
	float _last_angle;
    IMUTransformer _last_transformer;
	bool _fit_valid;

	BuffAngle _angle;
    cv::Point2f _rightUP;
    cv::Point2f _leftUP;
    cv::Point2f _leftLOW;
    cv::Point2f _rightLOW;
	cv::Point2f _rCenter;
	double _radius;

    const float max_v_small = 2.0;
	const float max_v_big = 3.0; // set max angular speed (rad/s)
	const float max_a = 3.0; // set max angular acceleration (rad/s^2)

	int _rotate_sign;

    Eigen::Vector3d _gimbal_gyro; // adapt for trajectory
    cv::Point3f _position;// adapt for trajectory

    //std::ofstream _pvdata;

public:

	BuffPredictor_V1() {
        _speedFilter.Reset();
        _latest_time = -10000;
        _last_time = -10000;
		_fit_valid = false;
		_rotate_sign = 0;
		_mode = BuffMode::UnknownBuff;
        _gimbal_gyro = {-1, -1, -1};
        _position = {-1, -1, -1};

        //_pvdata.open("/home/alliance/Desktop/buff_predictor_v.txt");
    }
	BuffPredictor_V1(const BuffPredictor_V1&) = delete;
	BuffPredictor_V1(BuffPredictor_V1&&) = delete;

	// update with external gyroscope
	void Update(
		const Buff5PointIdentifyData& buff5PointIdentifyData,
		const unsigned short remainingTime)
	{
		_angle = buff5PointIdentifyData.buffAngle;
		_rightUP = buff5PointIdentifyData.rightUp;
        _leftUP = buff5PointIdentifyData.leftUp;
        _leftLOW = buff5PointIdentifyData.leftLow;
        _rightLOW = buff5PointIdentifyData.rightLow;
        _rCenter = buff5PointIdentifyData.rCenter;
        _radius = buff5PointIdentifyData.radius;
        _last_transformer = buff5PointIdentifyData.transformer;

        if (remainingTime > 200) _mode = BuffMode::SmallBuff;
        else _mode = BuffMode::BigBuff;
        //_mode = BuffMode::UnknownBuff; // static buff mode change (no prediction)

        //std::cout << buff5PointIdentifyData.buffAngularSpeed.speed << std::endl;
        if (fabs(buff5PointIdentifyData.buffAngularSpeed.speed) > 4.0) { // switch leaf
            //std::cout << "\t\t" << buff5PointIdentifyData.buffAngularSpeed.speed << std::endl;
            _last_time = -1;
		}
		else if (_mode == BuffMode::BigBuff) {
			if (_last_time == -1) {
				int sign = buff5PointIdentifyData.buffAngularSpeed.speed >= 0 ? 1 : -1;
				if (fabs(buff5PointIdentifyData.buffAngularSpeed.speed) > max_v_big)
					_ceres.push(BuffAngularSpeed(buff5PointIdentifyData.buffAngularSpeed.timeStamp, max_v_big * sign));
				else _ceres.push(buff5PointIdentifyData.buffAngularSpeed);

                _last_time = buff5PointIdentifyData.buffAngularSpeed.timeStamp;
				_last_speed = buff5PointIdentifyData.buffAngularSpeed.speed;
			}
			else {
				float delta_speed = buff5PointIdentifyData.buffAngularSpeed.speed - _last_speed;
				TimeStamp delta_time = buff5PointIdentifyData.buffAngularSpeed.timeStamp - _last_time;
				auto accel = 1e3 * delta_speed / delta_time;
				if (fabs(buff5PointIdentifyData.buffAngularSpeed.speed) > max_v_big || accel > max_a) {
					_last_speed = _ceres.shiftWindowFilter(buff5PointIdentifyData.buffAngularSpeed);
                    _last_time = buff5PointIdentifyData.buffAngularSpeed.timeStamp;
				}
				else {
                    _fit_valid = _ceres.push(buff5PointIdentifyData.buffAngularSpeed);
                    _last_time = buff5PointIdentifyData.buffAngularSpeed.timeStamp;

                    if (debugCanvas.buff) {
                        cv::line(debugCanvas.buff.GetMat(), cv::Point(debugCanvas.buff.GetMat().cols-90, debugCanvas.buff.GetMat().rows/2),
                                 cv::Point(debugCanvas.buff.GetMat().cols-90,
                                           -100*buff5PointIdentifyData.buffAngularSpeed.speed + debugCanvas.buff.GetMat().rows/2), COLOR_GREEN, 1);
                    }

                    // mean filter
					//_last_speed = (_last_speed + buff5PointIdentifyData.buffAngularSpeed.speed) / 2;
                    _last_speed = _speedFilter.Predict(buff5PointIdentifyData.buffAngularSpeed.speed);

                    if (debugCanvas.buff) {
                        cv::line(debugCanvas.buff.GetMat(), cv::Point(debugCanvas.buff.GetMat().cols-70, debugCanvas.buff.GetMat().rows/2),
                                 cv::Point(debugCanvas.buff.GetMat().cols-70,
                                           -100*_last_speed + debugCanvas.buff.GetMat().rows/2), COLOR_GREEN, 1);
                    }
				}
			}
		}
		else if (_mode == BuffMode::SmallBuff) {
			if (_last_time == -1)	{
                _last_time = buff5PointIdentifyData.buffAngle.timeStamp;
				_last_angle = buff5PointIdentifyData.buffAngle.angle;
			}
			else {
				auto delta_angle = buff5PointIdentifyData.buffAngle.angle - _last_angle;
				auto delta_time = buff5PointIdentifyData.buffAngle.timeStamp - _last_time;
				auto speed = 1e3 * delta_angle / delta_time;
				float correct_angle = 0;
				if (delta_angle > 0) _rotate_sign++;
				else _rotate_sign--;
                //std::cout << _rotate_sign << std::endl;

				if (fabs(speed) > max_v_small) {
					correct_angle = -delta_angle / 2;
					_angle.angle += correct_angle;

					_rightUP -= _rCenter;
					_leftUP -= _rCenter;
					_leftLOW -= _rCenter;
					_rightLOW -= _rCenter;
					_rightUP = cv::Point2f(cos(correct_angle) * _rightUP.x - sin(correct_angle) * _rightUP.y,
						sin(correct_angle) * _rightUP.x + cos(correct_angle) * _rightUP.y);
					_leftUP = cv::Point(cos(correct_angle) * _leftUP.x - sin(correct_angle) * _leftUP.y,
						sin(correct_angle) * _leftUP.x + cos(correct_angle) * _leftUP.y);
					_leftLOW = cv::Point(cos(correct_angle) * _leftLOW.x - sin(correct_angle) * _leftLOW.y,
						sin(correct_angle) * _leftLOW.x + cos(correct_angle) * _leftLOW.y);
					_rightLOW = cv::Point(cos(correct_angle) * _rightLOW.x - sin(correct_angle) * _rightLOW.y,
						sin(correct_angle) * _rightLOW.x + cos(correct_angle) * _rightLOW.y);
					_rightUP += _rCenter;
					_leftUP += _rCenter;
					_leftLOW += _rCenter;
					_rightLOW += _rCenter;
				}
                _last_time = buff5PointIdentifyData.buffAngle.timeStamp;
				_last_angle = _angle.angle;
			}
		}
		else {
            _last_time = buff5PointIdentifyData.buffAngle.timeStamp;
		}

        Buff5Point bp;
        bp.Set(_rightUP, _leftUP, _leftLOW, _rightLOW, _rCenter);

        auto&& p = buffPnPSolver.Solve(bp, _last_transformer);
        _gimbal_gyro = *p;
	}

    // update latest time
    void Update(const TimeStamp latest_time) {
        _latest_time = latest_time;
    }

	// predict with last external gyroscope
	const std::optional<Eigen::Vector3d> Predict(
        const bool showPoints = false,
        const double deltaTimeSec = 0.0)
	{
        if (_latest_time < 0 || _latest_time - _last_time > 1000) {
            return std::nullopt;
        }

        double angle = _angle.angle;
        double delta_angle = 0;

		if (_mode == BuffMode::BigBuff) {
            if (!_fit_valid) {
                delta_angle = 0;
                //return std::nullopt;
            }
            else {
                const double fitTimeShiftSec = 0.0;
                delta_angle = _ceres.PredictDeltaAngle(
                        _last_time,
                        (_latest_time - _last_time) + (deltaTimeSec + fitTimeShiftSec) * 1000);
                //std::cout << "fitTimeShiftSec = " << fitTimeShiftSec << " deltaTimeSec = " << deltaTimeSec << " delta_angle = " << delta_angle * 180 / MathConsts::Pi << std::endl;
                //std::cout << "abs of delta angle = " << fabs(delta_angle * 180 / MathConsts::Pi) << std::endl;
                if (fabs(delta_angle * 180 / MathConsts::Pi) > 120 || delta_angle != delta_angle) {
                    std::cout << "error delta angle result" << std::endl;
                    _ceres.reset();
                }
            }
        }
		else if (_mode == BuffMode::SmallBuff) {
            delta_angle = 2 * MathConsts::Pi / 6 * ((double)(_latest_time - _last_time) * 0.001 + deltaTimeSec) * (_rotate_sign > 0 ? 1 : -1);
        }
		else return _gimbal_gyro;

        delta_angle = 0.0f;

        //std::cout << _latest_time - _last_time << " -> " << ((double)(_latest_time - _last_time) * 0.001 + deltaTimeSec) << " @ " << delta_angle << std::endl;
		if (debugCanvas.buff) {
            cv::line(debugCanvas.buff.GetMat(),
                     cv::Point(debugCanvas.buff.GetMat().cols - 50, debugCanvas.buff.GetMat().rows / 2),
                     cv::Point(debugCanvas.buff.GetMat().cols - 50,
                                   -100 * delta_angle + debugCanvas.buff.GetMat().rows / 2), COLOR_GREEN, 1);
        }

		angle += delta_angle;
		if (angle > 2 * MathConsts::Pi) angle -= 2 * MathConsts::Pi;
		else if (angle < 0) angle += 2 * MathConsts::Pi;
		//std::cout << angle << "\t" << sin(angle) << "\t" << cos(angle) << std::endl;

		//cv::Point2f armorCenter = cv::Point2f(cos(angle), sin(angle)); // no longer need
		//armorCenter *= _radius;
		//armorCenter = armorCenter + _rCenter;

        if (debugCanvas.buff) {
            if (true) {
                cv::circle(debugCanvas.buff.GetMat(), _rightUP, 2, COLOR_RED, 3, 8);
                cv::circle(debugCanvas.buff.GetMat(), _rightLOW, 2, COLOR_RED, 3, 8);
                cv::circle(debugCanvas.buff.GetMat(), _leftUP, 2, COLOR_RED, 3, 8);
                cv::circle(debugCanvas.buff.GetMat(), _leftLOW, 2, COLOR_RED, 3, 8);
                cv::circle(debugCanvas.buff.GetMat(), _rCenter, 2, COLOR_RED, 3, 8);
            }
        }

        cv::Point2f rightUP, leftUP, leftLOW, rightLOW;
        rightUP = _rightUP - _rCenter;
        leftUP = _leftUP - _rCenter;
        leftLOW = _leftLOW - _rCenter;
        rightLOW = _rightLOW - _rCenter;
        rightUP =   cv::Point2f(cos(delta_angle) * rightUP.x - sin(delta_angle) * rightUP.y,
                                sin(delta_angle) * rightUP.x + cos(delta_angle) * rightUP.y);
        leftUP =    cv::Point2f(cos(delta_angle) * leftUP.x - sin(delta_angle) * leftUP.y,
                                sin(delta_angle) * leftUP.x + cos(delta_angle) * leftUP.y);
        leftLOW =   cv::Point2f(cos(delta_angle) * leftLOW.x - sin(delta_angle) * leftLOW.y,
                                sin(delta_angle) * leftLOW.x + cos(delta_angle) * leftLOW.y);
        rightLOW =  cv::Point2f(cos(delta_angle) * rightLOW.x - sin(delta_angle) * rightLOW.y,
                                sin(delta_angle) * rightLOW.x + cos(delta_angle) * rightLOW.y);
        rightUP += _rCenter;
        leftUP += _rCenter;
        leftLOW += _rCenter;
        rightLOW += _rCenter;

		if (debugCanvas.buff && showPoints) {
            if (true) {
                cv::circle(debugCanvas.buff.GetMat(), rightUP, 2, COLOR_GREEN, 3, 8);
                cv::circle(debugCanvas.buff.GetMat(), rightLOW, 2, COLOR_GREEN, 3, 8);
                cv::circle(debugCanvas.buff.GetMat(), leftUP, 2, COLOR_GREEN, 3, 8);
                cv::circle(debugCanvas.buff.GetMat(), leftLOW, 2, COLOR_GREEN, 3, 8);
                cv::circle(debugCanvas.buff.GetMat(), _rCenter, 2, COLOR_GREEN, 3, 8);
            }
		}

        Buff5Point bp;
		bp.Set(rightUP, leftUP, leftLOW, rightLOW, _rCenter);

        /*
		// verify hit point
		//if (_last_time - _verifiedTime > 1100) {
		//	if (_last_time - _historyShotTime > ts) {
		//		_historyShotPoint = _armorCenter;
		//		_historyShotTime = _last_time;
		//	}
		//	_verifiedPoint = Point2f(cos(_angle.angle), sin(_angle.angle));
		//	_verifiedPoint *= _radius;
		//	_verifiedPoint += _rCenter;
		//
		//	if (_last_time - _verifiedTime > 1100 + ts) {
		//		_verifiedTime = _last_time;
		//	}
		//}
		// 
		//if constexpr (debugCanvas.buff) {
		//cv::circle(debugImg, _historyShotPoint, 5, COLOR_PINK, 3, 8);
		//cv::circle(debugImg, _verifiedPoint, 5, COLOR_GREEN, 3, 8);
		//cv::circle(debugImg, _armorCenter, 5, COLOR_RED, 3, 8)
		//}
		 */

        if (auto&& p = buffPnPSolver.Solve(bp, _last_transformer, showPoints)) {
            return *p;
        }
        else {
            return std::nullopt;
        }
	}
};
