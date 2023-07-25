#pragma once
/*
Creation Date: 2023/5/22
Latest Update: 2023/5/22
Developer(s): 21-WZY 21-WTR
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- solve image coordinate as robot coordinate
- using quaternion from gyroscope
*/

#include <optional>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "Core/Identifier/Buff/BuffStruct.h"
#include "Core/PnPSolver/BuffPnPSolverInterface.h"
#include "Util/Parameter/Parameters.h"
#include "Control/Gimbal/Gimbal.h"
#include "Control/Serial/HiPNUC.h"


class BuffPnPSolver_V1 {
public:
	BuffPnPSolver_V1() {
        //cv::namedWindow("Trackbar");
        //cv::createTrackbar("angle * 1000", "Trackbar", &_angle1000, 2 * MathConsts::Pi * 1000);
    }
	BuffPnPSolver_V1(const BuffPnPSolver_V1&) = delete;
	BuffPnPSolver_V1(BuffPnPSolver_V1&&) = delete;

    // solve pnp with external gyroscope
    template<typename TransformerType>
    std::optional<Eigen::Vector3d> Solve(
        const Buff5Point& buff5Point,
        const TransformerType transformer,
        const bool showPoints = false)
    {
        cv::Mat rvec, tvec;

		if (cv::solvePnP(Buff5Point3f, buff5Point.points,
						 CameraMatrix, DistCoeffs,
						 rvec, tvec, false, cv::SOLVEPNP_IPPE))
		{
			float x = tvec.at<double>(2), y = -tvec.at<double>(0), z = -tvec.at<double>(1);
            Eigen::Vector3d cam_link(x, y, z);

            //cam_link = {7000*cos(0.001*_angle1000-MathConsts::Pi), -7000*sin(0.001*_angle1000-MathConsts::Pi), 0};

            //Eigen::Vector3d gimbal_gyro = transformer.Link2Gyro(transformer.CameraLink2GimbalLink(cam_link));
            Eigen::Vector3d cam_gyro = transformer.Link2Gyro(Eigen::Vector3d {cam_link.x(), -cam_link.y(), -cam_link.z()});

            //std::cout << "link:\n" << cam_link << "\ntransformer: " << "\ngyro:\n" << cam_gyro << std::endl;
            if constexpr (debugCanvas.buff) {
                if (true && showPoints) {
                    float xy_scale = 0.05, radius1 = 5000, radius2 = 7000, radius3 = 9000;
                    float z_scale = 0.1, height1 = 1000, height2 = 2000;

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
                    cv::Point2f cam_2D = xy_scale * cv::Point2f(-cam_link.y(), -cam_link.x()) +
                                              0.5 * cv::Point2f(debugImg.cols, debugImg.rows);
                    cv::circle(debugImg, cam_2D, 2, COLOR_GREEN, 3, 8);
                    cv::Point2f world_2D = xy_scale * cv::Point2f(-cam_gyro.y(), -cam_gyro.x()) +
                                                0.5 * cv::Point2f(debugImg.cols, debugImg.rows);
                    cv::circle(debugImg, world_2D, 2, COLOR_LIME, 3, 8);
                    // z coordinate point
                    cv::Point2f cam_hei = cv::Point2f(debugImg.cols, -z_scale * cam_link.z() + debugImg.rows / 2);
                    cv::circle(debugImg, cam_hei, 2, COLOR_GREEN, 3, 8);
                    cv::Point2f world_hei = cv::Point2f(debugImg.cols, -z_scale * cam_gyro.z() + debugImg.rows / 2);
                    cv::circle(debugImg, world_hei, 2, COLOR_LIME, 3, 8);
                }
            }

            //return gimbal_gyro;
            //return cam_link;
            return cam_gyro;
		}
		return std::nullopt;
    }

private:
    //int _angle1000 = 3142;
};
