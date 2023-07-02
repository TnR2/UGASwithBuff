#pragma once
/*
Creation Date: 2023/6/8
Latest Update: 2023/6/8
Developer(s): 21-WTR
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
-
*/
#include "Util/Debug/DebugCanvas.h"


class VideoRecorder_V1 {
private:
    cv::VideoWriter _vw;
    cv::VideoWriter _vwUI;
    std::string _vwFilename = getTime() + ".avi";
    std::string _vwFilenameUI = getTime() + " UI.avi";
    int _vwCoder = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    double _vwFps = 30.0;

    std::string getTime() {
        time_t timep;
        time(&timep);
        char tmp[64];
        strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&timep));
        return tmp;
    }

public:
    VideoRecorder_V1(const cv::Size& imgSize){
        _vw.open(_vwFilename, _vwCoder, _vwFps, imgSize, true);
        if (!_vw.isOpened()) {
            std::cout << "video writer open failed" << std::endl;
        }
        _vwUI.open(_vwFilenameUI, _vwCoder, _vwFps, imgSize, true);
        if (!_vwUI.isOpened()) {
            std::cout << "video writer (UI) open failed" << std::endl;
        }
    }

    ~VideoRecorder_V1(){
        _vw.release();
        _vwUI.release();
    }

    void RecCam(const cv::Mat& img) {
        //thread recCamThread([&](const cv::Mat& img) { _vw.write(img); }, img);
        //recCamThread.detach();
        _vw.write(img);
    }

    void RecUI(const cv::Mat& img) {
        //thread recUIThread([&](const cv::Mat& img) { _vwUI.write(img); }, img);
        //recUIThread.detach();
        _vwUI.write(img);
    }
};
