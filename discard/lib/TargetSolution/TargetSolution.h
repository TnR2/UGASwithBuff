#pragma once
/*
Creation Date: 2022/11/19
Latest Update: 2022/11/19
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 更新robots
*/
#include <Common/UniversalStruct.h>
#include <Common/Robot/Robot.h>
#include <Common/DebugTools/DebugHeader.h>

class TargetSolution {
public:
	virtual ~TargetSolution() = default;

	virtual void Solve(TimeStamp ImgTime, std::vector<ArmorPlate>& armors) = 0;
};
