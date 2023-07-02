#include "Gimbal.h"

#include <thread>
#include <opencv2/opencv.hpp>

#include "Core/ImgCapture/Common/CVVideoCapture.h"
#include "Core/ImgCapture/Common/HTCameraCapture.h"
#include "Core/ImgCapture/Common/HikCameraCapture.h"
#include "Core/ImgCapture/Common/ResizeCapture.h"
#include "Core/ImgCapture/Common/RotateCapture.h"
#include "Core/Pretreator/Armor/ArmorPretreator_V1.h"
#include "Core/Pretreator/Armor/ArmorPretreator_V2.h"
#include "Core/Identifier/Armor/ArmorIdentifier_V1.h"
#include "Core/Identifier/Armor/ArmorIdentifier_V2.h"
#include "Core/Identifier/Armor/ArmorIdentifier_V3.h"
#include "Core/Identifier/Number/NullNumberIdentifier.h"
#include "Core/Identifier/Number/NumberIdentifier_V1.h"
#include "Core/Identifier/Color/ColorIdentifier_V1.h"
#include "Core/Identifier/Buff/BuffIdentifier_V1.h"
#include "Core/Identifier/Buff/BuffIdentifier_V2.h"
#include "Core/Identifier/Buff/BuffIdentifier_V3.h"
#include "Core/Identifier/Buff/BuffIdentifier_V4.h"
#include "Core/Identifier/Buff/BuffIdentifier_V5.h"
#include "Core/PnPSolver/Armor/ArmorPnPSolver.h"
#include "Core/PnPSolver/Buff/BuffPnPSolver_V1.h"
#include "Core/Predictor/Armor/SimplePredictor.h"
#include "Core/Predictor/Buff/BuffPredictor_V1.h"
#include "Core/Strategy/Common/Strategy_V1.h"
#include "Core/Trajectory/Common/Trajectory_V1.h"
#include "Core/Transformer/SimpleTransformer.h"
#include "Core/VideoRecorder/VideoRecorder.h"
#include "Control/Serial/CBoardInfantry.h"
#include "Control/Serial/VirtualCBoard.h"
#include "Control/Serial/GYH1.h"
#include "Control/Serial/HiPNUC.h"
#include "Util/Debug/DebugCanvas.h"
#include "Util/Debug/DebugSettings.h"
#include "Util/Recorder/PNGRecorder.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Util/Debug/MatForm/RectangleControl.hpp"

void Gimbal::Always() {
//    auto imgCapture = RotateCapture<HikCameraCapture>(cv::RotateFlags::ROTATE_180);
    auto imgCapture = HikCameraCapture();
//    auto imgCapture = CVVideoCapture("buff_blue2.mp4");
//    auto imgCapture = CVVideoCapture("buff_red.mp4");

    auto rec = VideoRecorder_V1(cv::Size(1440, 1080)); // video(1920,1080) cam(1440,1080)

    auto hipnuc = HiPNUC("/dev/IMU");
    auto cboard = CBoardInfantry("/dev/CBoard");
    //auto cboard = VirtualCBoard();

    /********************************/

    auto armorIdentifier = ArmorIdentifier_V3<NumberIdentifier_V1>("../models/NumberIdentifyModelV3.pb");

    //auto pnpSolver = ArmorPnPSolver_V1();
    auto pnpSolver = ArmorPnPSolver();
    auto armorPredictor = SimplePredictor();
    auto strategy = Strategy_V1();

    /********************************/

    auto buffIdentifier = BuffIdentifier_V5();
    auto buffPredictor = BuffPredictor_V1();

    /********************************/

    auto trajectory = Trajectory_V1();

    auto fps = FPSCounter_V2();
    auto recorder = PNGRecorder("../images/", 0.0);

    TimeStamp aimTimeStamp, buffTimeStamp;

    while (true) {
        try {

            cboard.Receive();
            auto [img, timestamp] = imgCapture.Read();
            auto timeStamp = std::chrono::steady_clock::now();
            auto transformer = hipnuc.GetTransformer();
            //std::cout<< timeStamp << " @ " << (int)cboard.GetAimingState() << " " << cboard.GetGyrosAttitude() << std::endl;

            if constexpr (debugCanvas.master) {
                debugCanvas.master.LoadMat(img);
            }
            
            //Shoot buff mode
            if (false || cboard.GetAimingState() == AimingState::Buff) {
                //陀螺仪正常时使用陀螺仪数据的代码
                if(transformer.Available())
                {
                    buffTimeStamp = timestamp;
                    if (auto&& buffIdentifyData = buffIdentifier.Identify(img, timestamp, cboard.GetEnemyColor())) 
                    {
                        buffPredictor.Update(*buffIdentifyData,true?cboard.GetRemaningTime():100,transformer);
                        if (auto&& attitude = trajectory.GetShotAnglewithGyro(buffPredictor, cboard.GetGyrosAttitude())) 
                            {
                                if (timestamp - aimTimeStamp > 100) 
                                { 
                                    cboard.Send((*attitude).yaw * MathConsts::Pi / 180.0, (*attitude).pitch * MathConsts::Pi / 180.0);
                                }
                            }
                            else 
                            {
                                //std::cout<<"no target attitude"<<std::endl;
                                //cboard.Send(0.0, 0.0); // using the world position/attitude, buff needn't send (0,0)
                            }
                    }
                    else
                    {
                        //cboard.Send(1.0, 1.0);
                        //std::cout<<"no target"<<std::endl
                    }
                }

                //陀螺仪坏了使用C板数据
                else
                {
                    buffTimeStamp = timestamp;

                    //std::cout << "EnemyColor = " << (int)cboard.GetEnemyColor() << std::endl;
                    //std::cout << "RemaningTime = " << cboard.GetRemaningTime() << std::endl;

                    if (auto&& buffIdentifyData = buffIdentifier.Identify(img, timestamp, cboard.GetEnemyColor())) {
                    //if (auto&& buffIdentifyData = buffIdentifier.Identify(img, timeStamp, ArmorColor::Blue)) {
                    //std::cout<<"update target"<<std::endl;
                        buffPredictor.Update(*buffIdentifyData,
                                        cboard.GetGyrosAttitude(),
                                        true ? cboard.GetRemaningTime() : 100); // remaining time >200: smallbuff, <200 bigbuff

                            if (auto&& attitude = trajectory.GetShotAngle(buffPredictor, cboard.GetGyrosAttitude())) 
                            {
                            //std::cout<<"target attitude"<<std::endl;

                            //std::cout << "buff has run for " << timeStamp - aimTimeStamp << "ms" << std::endl;
                                if (timestamp - aimTimeStamp > 100) 
                                { // jump over the first several attitude result (in 100ms)
                                    cboard.Send((*attitude).yaw * MathConsts::Pi / 180.0, (*attitude).pitch * MathConsts::Pi / 180.0);
                                }
                            }
                            else 
                            {
                                //std::cout<<"no target attitude"<<std::endl;
                                //cboard.Send(0.0, 0.0); // using the world position/attitude, buff needn't send (0,0)
                            }

                    }
                    else 
                    {
                        //cboard.Send(1.0, 1.0);
                        //std::cout<<"no target"<<std::endl;
                    }
                }

            }
            //shoot armor mode
            else {

                //std::cout << timeStamp << " normal aiming state" << std::endl;
                if constexpr (debugCanvas.buff) {
                    //cv::rectangle(debugCanvas.buff.GetMat(), cv::Point(0,0), cv::Point(debugCanvas.buff.GetMat().cols, debugCanvas.buff.GetMat().rows), COLOR_BLUE, -1);
                }

                aimTimeStamp = timestamp;

                auto armors = armorIdentifier.Identify(img, cboard.GetEnemyColor());

                if (transformer.Available()) {
                    // 陀螺仪工作正常
                    auto armors3d = pnpSolver.SolveAll(armors, transformer);
                    if (auto target = armorPredictor.Update(armors3d, std::chrono::steady_clock::now())) {
                        auto [yaw, pitch] = trajectory.GetShotAngle(*target, cboard.GetBulletSpeed(), transformer);
                        yaw += 0.5 / 180.0 * MathConsts::Pi;
                        pitch -= 0.0 / 180.0 * MathConsts::Pi;
                        cboard.SendUAV(yaw, pitch);
                    }
                    else {
                        cboard.SendUAV(0, 0);
                    }
                }
                else {
                    // 陀螺仪工作不正常，采用低保运行模式
                    auto armors3d = pnpSolver.SolveAll(armors);
                    if (auto target = armorPredictor.Update(armors3d, std::chrono::steady_clock::now())) {
                        auto [yaw, pitch] = trajectory.GetShotAngle(*target, cboard.GetBulletSpeed());
                        yaw -= -1.0 / 180.0 * MathConsts::Pi;
                        pitch -= 0.9 / 180.0 * MathConsts::Pi;
                        cboard.SendUAV(yaw, pitch);
                        //std::cout<<timeStamp<<" @ ["<<yaw<<", "<<pitch<<"]"<<std::endl;
                    }
                    else {
                        cboard.SendUAV(0, 0);
                    }
                }

            }

//            static double offset = 10.0;
//            static TimeStamp offsetTS = 0;
//            if((TimeStampCounter::GetTimeStamp() % 2000) < offsetTS)
//                offset *= -1;
//            offsetTS = TimeStampCounter::GetTimeStamp() % 2000;
//            double offset = sin((timeStamp % 100000000000) / 1000);

//            cboard.Send(offset*MathConsts::Pi/180.0, offset*MathConsts::Pi/180.0);
//            std::cout<<offset<<std::endl;

            if constexpr (RECORDER) {
                rec.RecCam(img);
                rec.RecUI(debugCanvas.GetAll());
            }

            if constexpr (DEBUG_IMG) {
                debugCanvas.ShowAll();
                cv::waitKey(1);
            }

            if (fps.Count()) {
                //fps.PrintFPS(debugCanvas.fps.GetMat());
                std::cout << "Fps: " << fps.GetFPS() << '\n';
            }

            recorder.Record(img, timeStamp);

        }
        catch (const char* str) { // 重包装异常
            throw_with_trace(std::runtime_error, str);
        }
    }

}
