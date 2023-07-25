#include "Gimbal.h"

#include <thread>
#include <opencv2/opencv.hpp>

#include "config.h"

#include "Core/ImgCapture/Common/CVVideoCapture.h"
#include "Core/ImgCapture/Common/HTCameraCapture.h"
#include "Core/ImgCapture/Common/HikCameraCapture.h"
#include "Core/ImgCapture/Common/ResizeCapture.h"
#include "Core/ImgCapture/Common/RotateCapture.h"
#include "Core/ImgCapture/Common/ImageFolderCapture.h"
#include "Core/Pretreator/Armor/ArmorPretreator_V1.h"
#include "Core/Pretreator/Armor/ArmorPretreator_V2.h"
#include "Core/Identifier/Armor/ArmorIdentifier_V1.h"
#include "Core/Identifier/Armor/ArmorIdentifier_V2.h"
#include "Core/Identifier/Armor/ArmorIdentifier_V3.h"
#include "Core/Identifier/Number/NullNumberIdentifier.h"
#include "Core/Identifier/Number/NumberIdentifier_V1.h"
#include "Core/Identifier/Number/NumberIdentifier_V2.h"
#include "Core/Identifier/Color/ColorIdentifier_V1.h"
#include "Core/PnPSolver/Armor/ArmorPnPSolver.h"
#include "Core/Predictor/Armor/SimplePredictor.h"
#include "Core/Tracker/Armor/ArmorEKFTracker.h"
#include "Core/Tracker/Armor/VerticalTracker.h"
#include "Core/Strategy/Common/Strategy_V1.h"
#include "Core/Trajectory/Common/Trajectory_V1.h"
#include "Core/Transformer/SimpleTransformer.h"
#include "Control/Serial/CBoardInfantry.h"
#include "Control/Serial/CBoardSentry.h"
#include "Control/Serial/VirtualCBoard.h"
#include "Control/Serial/GYH1.h"
#include "Control/Serial/HiPNUC.h"

#if (ENABLE_OPENVINO)
    #include "Core/Identifier/Buff/BuffIdentifier_V6.h"
    #include "Core/Identifier/Buff/BuffIdentifier_V7.h"
    #include "Core/Predictor/Buff/BuffPredictor_V1.h"
    #include "Core/Trajectory/Buff/BuffTrajectory_V1.h"
#endif // ENABLE_OPENVINO

#include "Util/Debug/DebugCanvas.h"
#include "Util/Recorder/PNGRecorder.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Util/Debug/MatForm/RectangleControl.hpp"


[[noreturn]] void Gimbal::Always() {
    auto imgCapture = HikCameraCapture();
    //auto imgCapture = RotateCapture<HikCameraCapture>(cv::RotateFlags::ROTATE_180);
    //auto imgCapture = ImageFolderCapture("../UGAS-record/sentry-record-230610-8am/");
    //auto imgCapture = CVVideoCapture("buff_blue2.mp4");

    auto hipnuc = HiPNUC("/dev/IMU");
    auto cboard = CBoardInfantry("/dev/CBoard");
    //auto cboard = VirtualCBoard();

    /********************************/

    auto armorIdentifier = ArmorIdentifier_V3<NumberIdentifier_V2>("../models/NumberIdentifyModelV3.pb", "../models/mlp.onnx");
    //auto armorIdentifier = ArmorIdentifier_V3<NumberIdentifier_V1>("models/NumberIdentifyModelV4.pb");

    auto pnpSolver = ArmorPnPSolver();
    auto simplePredictor = SimplePredictor();
    auto ekfTracker = ArmorEKFTracker();
    auto strategy = Strategy_V1();
    auto trajectory = Trajectory_V1();

    /********************************/

#if (ENABLE_OPENVINO)
    auto buffIdentifier = BuffIdentifier_V6();
    auto buffPredictor = BuffPredictor_V1();
    auto buffTrajectory = BuffTrajectory_V1();
#endif // ENABLE_OPENVINO

    /********************************/

    auto fpsV1 = FPSCounter_V1();
    auto fpsV2 = FPSCounter_V2();
    auto fps_buff = FPSCounter_V2();

    auto recorder = PNGRecorder("../images/", ENABLE_RECORDING ? 3.0 : 0.0);

    AutoscopeState lastAutoscopeState = AutoscopeState::Disable;
    TimeStamp aimTimeStamp, buffTimeStamp;

    while (true) {
        cboard.Receive();

        auto [img, timestamp_ull] = imgCapture.Read();
        auto timestamp = std::chrono::steady_clock::now();
        auto transformer = hipnuc.GetTransformer();

        if constexpr (debugCanvas.master) {
            debugCanvas.master.LoadMat(img);
        }

        //std::cout << "ts = " << timestamp_ull << std::endl;

        if (false || cboard.GetAutoscopeState() == AutoscopeState::Buff) {

        #if (ENABLE_OPENVINO)
            buffTimeStamp = timestamp_ull;
            static float last_pitch = 0.0f;

			auto enemyColor = false ? cboard.GetEnemyColor() : ArmorColor::Blue;
			auto remainingTime = false ? cboard.GetRemainingTime() : 300; // remaining time >200: smallbuff, <200 bigbuff
            //std::cout << "EnemyColor = " << enemyColor << std::endl;
            //std::cout << "RemaningTime = " << remaningTime << std::endl;

            if(transformer.Available()) {
            // using external gyroscope data while external gyroscope is working normally
                //std::cout<<"transformer available"<<std::endl;

                buffPredictor.Update(timestamp_ull);
                if (auto&& buffIdentifyData = buffIdentifier.Update(img, timestamp_ull, transformer, enemyColor)) { // V6
                //if (auto&& buffIdentifyData = buffIdentifier.Update(img, timestamp_ull, transformer)) { // V7
                    buffPredictor.Update(*buffIdentifyData, remainingTime);
                    //std::cout<<"update target"<<std::endl;
                    //if (fps_buff.Count()) {
                    //    std::cout << "buff fps: " << fps_buff.GetFPS() << '\n';
                    //}
                }
                if (auto&& attitude = buffTrajectory.GetShotAngle(buffPredictor, transformer)) {
                    //std::cout<<"trajectory angle"<<std::endl;
                    if (timestamp_ull - aimTimeStamp > 100) {
                        cboard.Send(0.8*(*attitude).yaw, -(*attitude).pitch); // adapt world atti pitch (negtive) for cboard
                        last_pitch = -(*attitude).pitch;
                    }
                }
                else {
                    //std::cout<<"no trajectory angle"<<std::endl;
                    cboard.Send(0.0, last_pitch);
                }
            }
            else {
            // external gyroscope is abnormal
                std::cout<<"transformer unavailable"<<std::endl;
            }
        #endif // ENABLE_OPENVINO

        }
        else {

            aimTimeStamp = timestamp_ull;

            auto armors = armorIdentifier.Identify(img, cboard.GetEnemyColor());

            if (transformer.Available()) {
	            // 陀螺仪工作正常
	            auto armors3d = pnpSolver.SolveAll(armors, transformer);

	            // 自瞄开启瞬间，重置跟踪目标
	            if (lastAutoscopeState == AutoscopeState::Disable &&
                    cboard.GetAutoscopeState() != AutoscopeState::Disable)
	                ekfTracker.lastTarget = ArmorID::Unknown;
                lastAutoscopeState = cboard.GetAutoscopeState();

	            if (auto&&target = ekfTracker.Update(armors3d, timestamp, transformer)) {
	                //auto&& pos = target->Predict(0);
	                //std::cout << pos.x() << ' ' << pos.y() << ' ' << pos.z() << '\n';
	                auto [yaw, pitch] = trajectory.GetShotAngle(*target, cboard.GetBulletSpeed(), transformer);
	                // sentry y+1.7 p+0.0
	                yaw += 0.8 / 180.0 * MathConsts::Pi;
	                pitch += - 3.6 / 180.0 * MathConsts::Pi;
	                cboard.Send(yaw, pitch);// , target->Shotable(trajectory._flyTime + 0.1));
	                recorder.Record(img, timestamp);

	                //std::cout << yaw * 53 << ' ' << pitch * 53 << '\n';
	            }
	            else {
	                cboard.Send(0, 0);//, false);
	                //std::cout << "sending zero!\n";
	            }
	        }
	        else {
	            // 陀螺仪工作不正常，采用低保运行模式
	            auto armors3d = pnpSolver.SolveAll(armors);
	            if (auto target = simplePredictor.Update(armors3d, timestamp)) {
	                auto [yaw, pitch] = trajectory.GetShotAngle(*target, cboard.GetBulletSpeed());
	                yaw += 0.0 / 180.0 * MathConsts::Pi;
	                pitch += 1.7 / 180.0 * MathConsts::Pi;
	                //cboard.Send(yaw, pitch);// , false);
	                cboard.Send(0, 0);
	                recorder.Record(img, timestamp);
	            }
	            else {
	                cboard.Send(0, 0);// , false);
	            }
	        }

        }

        if (fpsV1.Count()) {
            if constexpr (ENABLE_DEBUG_CANVAS) {
                fpsV1.PrintFPS(debugCanvas.fps.GetMat());
            }
        }
        if (fpsV2.Count()) {
            std::cout << "gimbal fps: " << fpsV2.GetFPS() << '\n';
        }

        if constexpr (ENABLE_DEBUG_CANVAS) {
            debugCanvas.ShowAll();
            cv::waitKey(1);
        }

        //cv::imwrite("test.png", img, compression_params);
    }
}
