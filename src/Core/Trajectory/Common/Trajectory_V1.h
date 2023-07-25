#pragma once

#include <cmath>

#include <tuple>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "Control/Gimbal/Gimbal.h"
#include "Core/Transformer/SimpleTransformer.h"
#include "Util/Parameter/Parameters.h"
#include "Util/TimeStamp/TimeStampCounter.h"

class Trajectory_V1 {
public:
    /*! 获取射击角度，不做预测处理。
    * \return 返回云台偏移量，格式为tuple[yaw, pitch]，单位弧度制，遵循右手定则。
    */
    template <typename TransformerType>
    auto GetShotAngle(const Eigen::Vector3d& targetGimbalGyro, const double speed, const TransformerType& transformer) const {
        std::tuple<double, double> result;
        auto& [yaw, pitch] = result;

        Eigen::Vector3d shotVec = GetShotVector(transformer.GimbalGyro2MuzzleGyro(targetGimbalGyro), speed);
        shotVec = transformer.Gyro2Link(shotVec);

        {
            if constexpr (debugCanvas.buff) {
                if (true) {
                    float xy_scale = 0.05, radius1 = 5000, radius2 = 7000, radius3 = 9000;
                    float z_scale = 0.1, height1 = 1000, height2 = 2000;

                    //Eigen::Vector3d targetGimbalLink_ = {3000, 1, 200};
                    //Eigen::Vector3d targetGimbalGyro_ = transformer.Link2Gyro(targetGimbalLink_);

                    Eigen::Vector3d mg = transformer.GimbalGyro2MuzzleGyro(targetGimbalGyro);
                    Eigen::Vector3d ml = transformer.Gyro2Link(mg);
                    //std::cout << "mg:\n" << mg << std::endl;
                    //std::cout << "ml:\n" << ml << std::endl;

                    cv::Mat &debugImg = debugCanvas.buff.GetMat();
                    // xy scale
                    cv::line(debugImg, cv::Point(debugImg.cols / 2 - 15, debugImg.rows / 2),
                             cv::Point(debugImg.cols / 2 + 15, debugImg.rows / 2), COLOR_LIME, 1);
                    cv::line(debugImg, cv::Point(debugImg.cols / 2, debugImg.rows / 2 - 15),
                             cv::Point(debugImg.cols / 2, debugImg.rows / 2 + 15), COLOR_LIME, 1);
                    cv::circle(debugImg, 0.5 * cv::Point2f(debugImg.cols, debugImg.rows), xy_scale * radius1,
                               COLOR_LIME);
                    cv::circle(debugImg, 0.5 * cv::Point2f(debugImg.cols, debugImg.rows), xy_scale * radius2,
                               COLOR_LIME);
                    cv::circle(debugImg, 0.5 * cv::Point2f(debugImg.cols, debugImg.rows), xy_scale * radius3,
                               COLOR_LIME);
                    // z scale
                    cv::line(debugImg, cv::Point(debugImg.cols - 30, debugImg.rows / 2),
                             cv::Point(debugImg.cols, debugImg.rows / 2), COLOR_LIME, 1);
                    cv::line(debugImg, cv::Point2f(debugImg.cols - 15, -z_scale * height1 + debugImg.rows / 2),
                             cv::Point2f(debugImg.cols, -z_scale * height1 + debugImg.rows / 2), COLOR_LIME);
                    cv::line(debugImg, cv::Point2f(debugImg.cols - 15, -z_scale * height2 + debugImg.rows / 2),
                             cv::Point2f(debugImg.cols, -z_scale * height2 + debugImg.rows / 2), COLOR_LIME);
                    cv::line(debugImg, cv::Point2f(debugImg.cols - 15, z_scale * height1 + debugImg.rows / 2),
                             cv::Point2f(debugImg.cols, z_scale * height1 + debugImg.rows / 2), COLOR_LIME);
                    cv::line(debugImg, cv::Point2f(debugImg.cols - 15, z_scale * height2 + debugImg.rows / 2),
                             cv::Point2f(debugImg.cols, z_scale * height2 + debugImg.rows / 2), COLOR_LIME);
                    // xy coordinate point
                    cv::Point2f cam_2D = xy_scale * cv::Point2f(-ml.y(), -ml.x()) +
                                         0.5 * cv::Point2f(debugImg.cols, debugImg.rows);
                    cv::circle(debugImg, cam_2D, 2, COLOR_GREEN, 3, 8);
                    cv::Point2f world_2D = xy_scale * cv::Point2f(-mg.y(), -mg.x()) +
                                           0.5 * cv::Point2f(debugImg.cols, debugImg.rows);
                    cv::circle(debugImg, world_2D, 2, COLOR_LIME, 3, 8);
                    // z coordinate point
                    cv::Point2f cam_hei = cv::Point2f(debugImg.cols, -z_scale * ml.z() + debugImg.rows / 2);
                    cv::circle(debugImg, cam_hei, 2, COLOR_GREEN, 3, 8);
                    cv::Point2f world_hei = cv::Point2f(debugImg.cols, -z_scale * mg.z() + debugImg.rows / 2);
                    cv::circle(debugImg, world_hei, 2, COLOR_LIME, 3, 8);
                }
            }
        }

        yaw = atan2(shotVec.y(), shotVec.x());
        pitch = -atan2(shotVec.z(), sqrt(shotVec.y() * shotVec.y() + shotVec.x() + shotVec.x()));

        return result;
    }

    /*! 获取射击角度，还没做预测。
    * \return 返回云台偏移量，格式为tuple[yaw, pitch]，单位弧度制，遵循右手定则。
    */
    template <typename TargetType, typename TransformerType>
    auto GetShotAngle(const TargetType& target, const double speed, const TransformerType& transformer) const {
        return GetShotAngle(target.Predict(0), speed, transformer);
    }


    template <typename TargetType>
    auto GetShotAngle(const TargetType& target, const double speed) const {
        auto pos = target.Predict(0);

        auto transformer = SimpleTransformer(pos);
        return GetShotAngle(transformer.CameraLink2GimbalLink(transformer.Link2Gyro(pos)), speed, transformer);
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
