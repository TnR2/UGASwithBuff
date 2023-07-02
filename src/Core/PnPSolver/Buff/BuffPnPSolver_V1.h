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


//template <typename IMUType>
class BuffPnPSolver_V1 {
public:
	BuffPnPSolver_V1() { }
	//BuffPnPSolver_V1(IMUType& imu) : _imu(imu) { }
	BuffPnPSolver_V1(const BuffPnPSolver_V1&) = delete;
	BuffPnPSolver_V1(BuffPnPSolver_V1&&) = delete;

    //有陀螺仪版本的solvepnp，新代码的gimbal坐标系相当于咱们的world坐标系，具体可以查阅我写的坐标系说明
    template<typename TransformerType>
    std::optional<cv::Point3f> SolvewithGyro(const Buff5Point& buff5Point,TransformerType transformer,const cv::Scalar color = COLOR_GREEN)
    {
        cv::Mat rvec, tvec;

		if (cv::solvePnP(Buff5Point3f, buff5Point.points,
						 CameraMatrix, DistCoeffs,
						 rvec, tvec, false, cv::SOLVEPNP_IPPE))
		{
			float x = tvec.at<double>(2), y = tvec.at<double>(0), z = -tvec.at<double>(1);
            Eigen::Vector3d cam_pose(x,y,z);//相机坐标系下的三维点坐标
            std::cout<<cam_pose.x()<<std::endl;
            auto gimbal_pose_v=transformer.Link2Gyro(transformer.CameraLink2GimbalLink(cam_pose));//等价于你的world_pose
            if constexpr (debugCanvas.buff) {
                float xy_scale = 0.05, z_scale = 0.1, radius1 = 5000, radius2 = 10000, height1 = 1000, height2 = 2000;
                cv::Mat &debugImg = debugCanvas.buff.GetMat();
                cv::line(debugImg, cv::Point(debugImg.cols / 2 - 15, debugImg.rows / 2),
                         cv::Point(debugImg.cols / 2 + 15, debugImg.rows / 2), COLOR_LIME, 1);
                cv::line(debugImg, cv::Point(debugImg.cols / 2, debugImg.rows / 2 - 15),
                         cv::Point(debugImg.cols / 2, debugImg.rows / 2 + 15), COLOR_LIME, 1);
                cv::line(debugImg, cv::Point(debugImg.cols - 30, debugImg.rows / 2),
                         cv::Point(debugImg.cols, debugImg.rows / 2), COLOR_LIME, 1);

                cv::circle(debugImg, 0.5 * cv::Point2f(debugImg.cols, debugImg.rows), xy_scale * radius1, COLOR_LIME);
                cv::circle(debugImg, 0.5 * cv::Point2f(debugImg.cols, debugImg.rows), xy_scale * radius2, COLOR_LIME);
                cv::Point2f cam_2Dpos = xy_scale * cv::Point2f(cam_pose.y(), -cam_pose.x()) +
                                        0.5 * cv::Point2f(debugImg.cols, debugImg.rows);
                cv::circle(debugImg, cam_2Dpos, 2, COLOR_LIME, 3, 8);
                cv::Point2f world_2Dpos = xy_scale * cv::Point2f(gimbal_pose_v.y(), -gimbal_pose_v.x()) +
                                          0.5 * cv::Point2f(debugImg.cols, debugImg.rows);
                cv::circle(debugImg, world_2Dpos, 2, COLOR_GREEN, 3, 8);

                cv::line(debugImg, cv::Point2f(debugImg.cols - 15, -z_scale * height1 + debugImg.rows / 2),
                         cv::Point2f(debugImg.cols, -z_scale * height1 + debugImg.rows / 2), COLOR_LIME);
                cv::line(debugImg, cv::Point2f(debugImg.cols - 15, -z_scale * height2 + debugImg.rows / 2),
                         cv::Point2f(debugImg.cols, -z_scale * height2 + debugImg.rows / 2), COLOR_LIME);
                cv::line(debugImg, cv::Point2f(debugImg.cols - 15, z_scale * height1 + debugImg.rows / 2),
                         cv::Point2f(debugImg.cols, z_scale * height1 + debugImg.rows / 2), COLOR_LIME);
                cv::line(debugImg, cv::Point2f(debugImg.cols - 15, z_scale * height2 + debugImg.rows / 2),
                         cv::Point2f(debugImg.cols, z_scale * height2 + debugImg.rows / 2), COLOR_LIME);
                cv::Point2f cam_hei = cv::Point2f(debugImg.cols, -z_scale * cam_pose.z() + debugImg.rows / 2);
                cv::circle(debugImg, cam_hei, 2, COLOR_LIME, 3, 8);
                cv::Point2f world_hei = cv::Point2f(debugImg.cols, -z_scale * gimbal_pose_v.z() + debugImg.rows / 2);
                cv::circle(debugImg, world_hei, 2, COLOR_GREEN, 3, 8);
            }
            cv::Point3f gimbal_pose(gimbal_pose_v.x(),gimbal_pose_v.y(),gimbal_pose_v.z());
			return gimbal_pose;
		}
		return std::nullopt;
    }


    //c板的solvepnp
	std::optional<cv::Point3f> Solve(
            const Buff5Point& buff5Point, const GimbalAttitude gyrosAttitude,
            const cv::Scalar color = COLOR_GREEN) const
    {
		cv::Mat rvec, tvec;

		if (cv::solvePnP(Buff5Point3f, buff5Point.points,
						 CameraMatrix, DistCoeffs,
						 rvec, tvec, false, cv::SOLVEPNP_IPPE))
		{
			float x = tvec.at<double>(2), y = tvec.at<double>(0), z = -tvec.at<double>(1);
            cv::Point3f cam_pose = cv::Point3f(x, y, z);

			// transform camera position to world position
			float yaw = gyrosAttitude.yaw * MathConsts::Pi / 180;
			float pitch = gyrosAttitude.pitch * MathConsts::Pi / 180;
			cv::Mat transMat = (cv::Mat_<float>(3, 3) <<
				-sin(yaw),		sin(pitch) * cos(yaw),		cos(pitch) * cos(yaw),
				cos(yaw),		sin(pitch) * sin(yaw),		cos(pitch) * sin(yaw),
				0,				    -cos(pitch),				        sin(pitch));
			cv::Mat position = transMat * (cv::Mat_<float>(3, 1) <<
				tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
            cv::Point3f world_pose = cv::Point3f(position.at<float>(0), position.at<float>(1), position.at<float>(2));

            //std::cout << "camera pos = " << cam_pose << std::endl << "world pos = " << world_pose << std::endl;
            if constexpr (debugCanvas.buff) {
                float xy_scale = 0.05, z_scale = 0.1, radius1 = 5000, radius2 = 10000, height1 = 1000, height2 = 2000;
                cv::Mat &debugImg = debugCanvas.buff.GetMat();
                cv::line(debugImg, cv::Point(debugImg.cols / 2 - 15, debugImg.rows / 2),
                         cv::Point(debugImg.cols / 2 + 15, debugImg.rows / 2), COLOR_LIME, 1);
                cv::line(debugImg, cv::Point(debugImg.cols / 2, debugImg.rows / 2 - 15),
                         cv::Point(debugImg.cols / 2, debugImg.rows / 2 + 15), COLOR_LIME, 1);
                cv::line(debugImg, cv::Point(debugImg.cols - 30, debugImg.rows / 2),
                         cv::Point(debugImg.cols, debugImg.rows / 2), COLOR_LIME, 1);

                cv::circle(debugImg, 0.5 * cv::Point2f(debugImg.cols, debugImg.rows), xy_scale * radius1, COLOR_LIME);
                cv::circle(debugImg, 0.5 * cv::Point2f(debugImg.cols, debugImg.rows), xy_scale * radius2, COLOR_LIME);
                cv::Point2f cam_2Dpos = xy_scale * cv::Point2f(cam_pose.y, -cam_pose.x) +
                                        0.5 * cv::Point2f(debugImg.cols, debugImg.rows);
                cv::circle(debugImg, cam_2Dpos, 2, COLOR_LIME, 3, 8);
                cv::Point2f world_2Dpos = xy_scale * cv::Point2f(world_pose.y, -world_pose.x) +
                                          0.5 * cv::Point2f(debugImg.cols, debugImg.rows);
                cv::circle(debugImg, world_2Dpos, 2, COLOR_GREEN, 3, 8);

                cv::line(debugImg, cv::Point2f(debugImg.cols - 15, -z_scale * height1 + debugImg.rows / 2),
                         cv::Point2f(debugImg.cols, -z_scale * height1 + debugImg.rows / 2), COLOR_LIME);
                cv::line(debugImg, cv::Point2f(debugImg.cols - 15, -z_scale * height2 + debugImg.rows / 2),
                         cv::Point2f(debugImg.cols, -z_scale * height2 + debugImg.rows / 2), COLOR_LIME);
                cv::line(debugImg, cv::Point2f(debugImg.cols - 15, z_scale * height1 + debugImg.rows / 2),
                         cv::Point2f(debugImg.cols, z_scale * height1 + debugImg.rows / 2), COLOR_LIME);
                cv::line(debugImg, cv::Point2f(debugImg.cols - 15, z_scale * height2 + debugImg.rows / 2),
                         cv::Point2f(debugImg.cols, z_scale * height2 + debugImg.rows / 2), COLOR_LIME);
                cv::Point2f cam_hei = cv::Point2f(debugImg.cols, -z_scale * cam_pose.z + debugImg.rows / 2);
                cv::circle(debugImg, cam_hei, 2, COLOR_LIME, 3, 8);
                cv::Point2f world_hei = cv::Point2f(debugImg.cols, -z_scale * world_pose.z + debugImg.rows / 2);
                cv::circle(debugImg, world_hei, 2, COLOR_GREEN, 3, 8);
            }
			return world_pose;
		}

		return std::nullopt;
	}

private:
	//const IMUType& _imu;
};
