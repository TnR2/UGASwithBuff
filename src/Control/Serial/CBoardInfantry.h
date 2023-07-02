#pragma once
/*
Creation Date: Unknown
Latest Update: 2023/04/21
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
与步兵的串口通讯
*/

#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

#include "Control/Gimbal/Gimbal.h"
#include "Util/Serial/CRC.h"
#include "Util/Serial/SerialUtil.h"
#include "Util/Debug/Log.h"
#include "Util/Parameter/Parameters.h"

class CBoardInfantry {
public:

#pragma pack(push, 1)
    struct DataSend {
        float yaw, pitch;
    };
    struct DataReceive {
        uint8_t selfColor;             // 自身队伍颜色：1-红，2-蓝
        uint8_t presetBulletSpeed;     // 预设弹速，单位：m/s
        float bulletSpeed;             // 实时弹速，单位：m/s
        uint8_t     aimingState;	// 自瞄状态 0:inative, 1:active, 2:buff
        float       yaw, pitch;		// C板陀螺仪
        uint16_t    remaningTime;   // 比赛剩余时间
        float       initYaw;        // 开启Buff的yaw
        uint8_t     r[9];			// 占位符
    };
#pragma pack(pop)

    CBoardInfantry(const char* portName) :
        _serial(portName, 115200, serial::Timeout::simpleTimeout(0)),
        _sender(_serial),
        _receiver(_serial) {
    }

    ~CBoardInfantry() {    }

    /*! 向除哨兵外的地面兵种发送云台瞄准数据
    * \param yaw pitch 单位使用弧度制，方向遵循右手定则
    */
    void Send(double yaw, double pitch) {
        _sender.Data.yaw = -yaw * 180.0 / MathConsts::Pi;
        _sender.Data.pitch = -pitch * 180.0 / MathConsts::Pi;
        _sender.Send();
        std::cout<<"send data: ["<<_sender.Data.yaw<<", "<<_sender.Data.pitch<<"]"<<std::endl;
    }

    /*! 向无人机发送云台瞄准数据
    * \param yaw pitch 单位使用弧度制，方向遵循右手定则
    */
    void SendUAV(double yaw, double pitch) {
        _sender.Data.yaw = -yaw * 180.0 / MathConsts::Pi;
        _sender.Data.pitch = -pitch * 180.0 / MathConsts::Pi;
        _sender.Send();
        std::cout<<"send data: ["<<_sender.Data.yaw<<", "<<_sender.Data.pitch<<"]"<<std::endl;
    }
    
    void Receive() {
        bool received = false;

        while (true) {
            auto result = _receiver.Receive();
            if (result == SerialUtil::ReceiveResult::Success)
                received = true;
            else if (result == SerialUtil::ReceiveResult::Timeout)
                break;
            else if (result == SerialUtil::ReceiveResult::InvaildHeader) {
                //LOG(WARNING) << "CboardInfantry: Invaild Header!";
            }
            else if (result == SerialUtil::ReceiveResult::InvaildVerifyDegit) {
                //LOG(WARNING) << "CboardInfantry: Invaild Verify Degit!";
            }
        }

        if (received) {
            const auto& data = _receiver.GetReceivedData();

            if (data.selfColor == 1)        // 己方红色，击打蓝色
                _enemyColor = ArmorColor::Blue;
            else if (data.selfColor == 2)   // 己方蓝色，击打红色
                _enemyColor = ArmorColor::Red;

            _bulletSpeed = data.presetBulletSpeed;  // 暂时不处理实时弹速
            
            _initYaw = -data.initYaw;
            //std::cout << "init yaw = " << -data.yaw << std::endl;

            _tempLastAimingState = _tempAimingState;
            _tempAimingState = static_cast<AimingState>(data.aimingState);
            static int aseqltimes = 0;
            if (_tempLastAimingState == _tempAimingState) aseqltimes++;
            else aseqltimes = 0;
            if (aseqltimes > 5) {
                //std::cout << aseqltimes << " update state, yaw, pitch" << std::endl;

                _lastAimingState = _aimingState;
                _aimingState = _tempAimingState;

                _yaw = -data.yaw - _initYaw;
                _pitch = -data.pitch;
                //std::cout << "received gyros attitude: [" << _yaw << ", " << _pitch << "]" << std::endl;
            }
            else {
                //std::cout << aseqltimes << " didn't update state, yaw, pitch" << std::endl;
            }
//                std::cout << aseqltimes << " @  " <<
//                    (int)_tempLastAimingState << "-" << (int)_tempAimingState << "  " <<
//                    (int)_lastAimingState << "-" << (int)_aimingState << std::endl;
//
//                if(_lastAimingState != AimingState::Buff && _aimingState == AimingState::Buff) {
//                    std::cout << "init yaw = " << -data.yaw << std::endl;
//                    _initYaw = -data.yaw;
//                }

            _remaningTime = data.remaningTime;
        }
    }

    ArmorColor GetEnemyColor() {
        return _enemyColor;
    }

    float GetBulletSpeed() {
        return _bulletSpeed;
    }

    AimingState GetAimingState() {
        return _aimingState;
    }

    GimbalAttitude GetGyrosAttitude() {
        return { _yaw, _pitch };
    }

    unsigned short GetRemaningTime() {
        return _remaningTime;
    }

private:
    serial::Serial _serial;

    SerialUtil::SerialSender<DataSend, SerialUtil::Head<uint8_t, 0xff>, CRC::DjiCRC8Calculator> _sender;
    SerialUtil::SerialReceiver<DataReceive, SerialUtil::Head<uint8_t, 0xff>, CRC::DjiCRC8Calculator> _receiver;
    ArmorColor _enemyColor = Parameters::DefaultEnemyColor;
    float _bulletSpeed = Parameters::DefaultBulletSpeed;
    AimingState _lastAimingState = AimingState::Inactive, _aimingState = AimingState::Inactive, 
                _tempLastAimingState = AimingState::Inactive, _tempAimingState = AimingState::Inactive;
    float _initYaw = .0f, _yaw = .0f, _pitch = .0f;
    uint16_t _remaningTime = 3600;

};