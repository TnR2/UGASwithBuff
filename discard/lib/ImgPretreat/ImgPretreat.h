#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供图片预处理类的接口
*/
#include <GimbalSerial/GimbalSerialHandle.h>
#include <Common/DebugTools/DebugHeader.h>

class ImgPretreat {
public:
	virtual ~ImgPretreat() = default;

	virtual std::tuple<cv::Mat, cv::Mat> GetPretreated(const cv::Mat& img) = 0;
};
