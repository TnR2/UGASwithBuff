#pragma once

#include <cmath>

#include <tuple>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "Control/Gimbal/Gimbal.h"
#include "Core/Transformer/SimpleTransformer.h"
#include "Util/Parameter/Parameters.h"
#include "Util/TimeStamp/TimeStampCounter.h"

class BuffTrajectory_V1 {
private:
    //const float yawOffset = 0.0f, yawRatio = 1.00f, pitchLevel = 0.0, pitchOffsetLow = 0.0f, pitchOffsetHigh = 0.0f; // default offset data
    const float yawOffset = - -1.10f, yawRatio = 1.35f, pitchLevel = - 11.70f, pitchOffsetLow = - 2.00f, pitchOffsetHigh = - 1.90f; // offset data of Mecanum

    //std::ofstream _attidata;

public:
    BuffTrajectory_V1 () {
        //_attidata.open("/home/alliance/Desktop/buff_attitude.txt");
    }

    // calc buff: trajectory with external gyroscope
    template<typename TransformerType>
    std::optional<GimbalAttitude> GetShotAngle(
        BuffPredictor_V1& buffPredictor,
        TransformerType transformer)
    {
        float bullet_speed = 29.0f; // (m/s)
        float bullet_fly_time = 7.0f / bullet_speed + 0.12f; // (s)

        Eigen::Vector3d cam_gyro;
        if (auto&& gimbal_gyro_opt = buffPredictor.Predict(true, bullet_fly_time)) {
            cam_gyro = *gimbal_gyro_opt;
        }
        else {
            return std::nullopt;
        }

        const double bullet_speed_for_shot = 25.0f;

        Eigen::Vector3d shotVecGyro = GetShotVector(cam_gyro, bullet_speed_for_shot);
        Eigen::Vector3d shotVecLink = transformer.Gyro2Link(shotVecGyro);

        double yawGyro = atan2(shotVecGyro.y(), shotVecGyro.x());
        double pitchGyro = -atan2(shotVecGyro.z(), sqrt(shotVecGyro.y() * shotVecGyro.y() + shotVecGyro.x() * shotVecGyro.x()));
        double yawLink = -atan2(shotVecLink.y(), shotVecLink.x());
        double pitchLink = atan2(shotVecLink.z(), sqrt(shotVecLink.y() * shotVecLink.y() + shotVecLink.x() * shotVecLink.x()));
        // here: l+r-, d+u-

        GimbalAttitude attitude;

        attitude.yaw = yawLink + yawOffset * MathConsts::Pi / 180.0;
        attitude.yaw *= yawRatio;

        if (pitchGyro * 180.0 / MathConsts::Pi > pitchLevel) { // remember d+u-
            attitude.pitch = pitchGyro + pitchOffsetLow * MathConsts::Pi / 180.0;
        }
        else {
            attitude.pitch = pitchGyro + pitchOffsetHigh * MathConsts::Pi / 180.0;
        }

        //std::cout << -yawLink * 180.0 / MathConsts::Pi << "\t" << -pitchGyro * 180.0 / MathConsts::Pi << std::endl;
        //std::cout << -attitude.yaw * 180.0 / MathConsts::Pi << "\t" << -attitude.pitch * 180.0 / MathConsts::Pi << std::endl;
        if (abs(attitude.yaw) > 20 * MathConsts::Pi / 180.0) return std::nullopt;
        if (attitude.yaw != attitude.yaw) return std::nullopt; // return when yaw "NaN"
        if (attitude.pitch != attitude.pitch) return std::nullopt; // return when pitch "NaN"
        // here: l+r-, d+u-

        //std::cout << "attitude: " << attitude << std::endl;
        //_attidata << attitude.yaw << "\t" << attitude.pitch << std::endl;

        // draw target delta attitude
        if constexpr (debugCanvas.buff) {
            if (true) {
                float display_scale = 600;
                cv::Mat &debugImg = debugCanvas.buff.GetMat();
                cv::line(debugImg, cv::Point(debugImg.cols / 2 - 15, debugImg.rows / 2),
                         cv::Point(debugImg.cols / 2 + 15, debugImg.rows / 2), COLOR_RED, 1);
                cv::line(debugImg, cv::Point(debugImg.cols / 2, debugImg.rows / 2 - 15),
                         cv::Point(debugImg.cols / 2, debugImg.rows / 2 + 15), COLOR_RED, 1);
                cv::circle(debugImg,
                           cv::Point(+display_scale * (-attitude.yaw) + debugCanvas.buff.GetMat().cols / 2,
                                            -display_scale * (-attitude.pitch) + debugCanvas.buff.GetMat().rows / 2),
                           5, COLOR_RED, 3, 8);
                cv::circle(debugImg,
                           cv::Point(+display_scale * (-yawLink) + debugCanvas.buff.GetMat().cols / 2,
                                            -display_scale * (-pitchLink) + debugCanvas.buff.GetMat().rows / 2),
                           5, COLOR_PINK, 3, 8);
            }
        }

        return attitude;
    }

private:
    Eigen::Vector3d GetShotVector(const Eigen::Vector3d& targetGimbalGyro, const double speed) const {
        // 不考虑空气阻力

        double x = targetGimbalGyro.x() / 1000;
        double y = targetGimbalGyro.y() / 1000;
        double z = targetGimbalGyro.z() / 1000;

        double hDis = sqrt(x * x + y * y);

        double yaw = atan2(y, x);
        double pitch = 0;

        double a = speed * speed;                  // v0 ^ 2
        double b = a * a;                          // v0 ^ 4
        double c = hDis * hDis;                    // xt ^ 2
        double d = c * c;                          // xt ^ 4
        double e = MathConsts::G * MathConsts::G;  // g ^ 2

        double f = b * d * (b - e * c - 2 * MathConsts::G * a * z);
        if (f >= 0) {
            pitch = -atan((b * c - sqrt(f)) / (MathConsts::G * a * c * hDis));
        }

        return { cos(pitch) * cos(yaw), cos(pitch) * sin(yaw),-sin(pitch) };
    }
};
