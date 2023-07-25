#pragma once
/*
Creation Date: 2023/05/26
Latest Update: 2023/05/26
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
基于IMU的坐标系转换器。
*/

#include <eigen3/Eigen/Dense>

class IMUTransformer {
public:
    IMUTransformer() {}
    IMUTransformer(const Eigen::Quaterniond& quat, bool available = true) : _transQuat(quat), _available(available) { }
    IMUTransformer(const Eigen::Quaternionf& quat, bool available = true) : _transQuat(quat), _available(available) { }

    // 返回该transformer是否有效，在IMU断联或帧率异常时，该函数应返回false
    bool Available() {
        return _available;
    }

    Eigen::Vector3d CameraLink2GimbalLink(const Eigen::Vector3d& srcPos) const {
        static const Eigen::Vector3d uav = { 118.05, 67.5, -41.7 };
        static const Eigen::Vector3d sentry = { 150, 0, 0 };
        return srcPos + sentry;
    }

    Eigen::Vector3d Link2Gyro(const Eigen::Vector3d& srcPos) const {
        auto dstPos = _transQuat * Eigen::Quaterniond{ 0, srcPos.x(), srcPos.y(), srcPos.z() } *_transQuat.inverse();
        return { dstPos.x(), dstPos.y(), dstPos.z() };
    }

    Eigen::Vector3d Gyro2Link(const Eigen::Vector3d& srcPos) const {
        auto dstPos = _transQuat.inverse() * Eigen::Quaterniond{ 0, srcPos.x(), srcPos.y(), srcPos.z() } *_transQuat;
        return { dstPos.x(), dstPos.y(), dstPos.z() };
    }

    Eigen::Vector3d GimbalGyro2MuzzleGyro(const Eigen::Vector3d& srcPos) const {
        static const Eigen::Vector3d uav = { 69.4, 67.5, 0 };
        static const Eigen::Vector3d sentry = { 100, 0, 40 };
        return srcPos - Link2Gyro(sentry);
    }

private:
    Eigen::Quaterniond _transQuat;
    bool _available;
};
