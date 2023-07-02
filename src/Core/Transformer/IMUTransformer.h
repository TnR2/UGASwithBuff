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
    IMUTransformer(const Eigen::Quaterniond& quat, bool available = true) : _transQuat(quat), _available(available) { }
    IMUTransformer(const Eigen::Quaternionf& quat, bool available = true) : _transQuat(quat), _available(available) { }

    // 返回该transformer是否有效，在IMU断联或帧率异常时，该函数应返回false
    bool Available() {
        return _available;
    }
    /******************* 坐标系说明，非常重要 **********************/
    //CameraLink:相机坐标系，xy1坐标系。
    //GimbalLink:云台坐标系，可以理解为世界坐标系，以机器人自身中心为原点的坐标系，xyz轴方向待核对，个人盲猜如果这套代码给哨兵用了，那么左侧为x正方向，前侧为y正方向，上为z正方向，给飞机用不太好确定，飞机结构我忘了。（相当于我和tiri的世界坐标系）
    //Link：也是云台坐标系，不过此云台和GimbalLink里面的云台不一样，这个我感觉是枪口的云台。
    //对于步兵来说，GimbalLink和Link是一样的，但是对于哨兵那种阴间的连杆结构，二者我觉得有区别，你少了一个yaw的转换。
    //Gyro:陀螺仪坐标系，直接参考陀螺仪的说明书就能知道原点、x、y、z了。
    Eigen::Vector3d CameraLink2GimbalLink(const Eigen::Vector3d& srcPos) const {
        return { srcPos.x() + 118.05, srcPos.y() + 67.5, srcPos.z() - 41.7 };
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
        return srcPos - Link2Gyro({ 69.4, 67.5, 0 });
    }

private:
    Eigen::Quaterniond _transQuat;
    bool _available;
};