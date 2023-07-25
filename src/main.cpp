#include <chrono>
#include <thread>

#include "Control/Gimbal/Gimbal.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Util/ROS/Node.h"

int main(int argc, char* argv[]) {
    //ros_util::init(argc, argv);

	for (int restartTime = 0;; ++restartTime) {
        LOG(INFO) << "Program started.";
		try {
			Gimbal gimbal;
			gimbal.Always();
		}
		catch (const std::exception& e) {
			LOG(ERROR) << "Uncaught " << typeid(e).name() << ": " << e.what();
		}
        catch (const char* str) {
            LOG(ERROR) << "Uncaught const char* : " << str;
        }
		catch (...) {
			LOG(ERROR) << "Uncaught unknown error";
		}
		LOG(ERROR) << "Program crashed, will restart in 0 seconds... ";
		std::this_thread::sleep_for(std::chrono::seconds(0));
	}
	return 0;
}
