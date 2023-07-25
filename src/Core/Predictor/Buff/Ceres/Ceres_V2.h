#pragma once
/*
Creation Date: 2023/05/24
Latest Update: 2023/07/18
Developer(s): 21-WZY
Reference(s): TUP-InfantryVision-2022-2.0.0
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- Fitting Trigonometric functions with Ceres library
*/
#include <iostream>
#include <fstream>
#include <cmath>

#include <ceres/ceres.h>

#include "Core/Identifier/Buff/BuffStruct.h"
#include "Util/Parameter/Parameters.h"


class Ceres_V2 {
private:
	static const int MaxData_ = 100;
	static const int MaxData = MaxData_ + 5;
	int _total_data;
	BuffFitData _lastBuffFitData;
	CircularQueue<BuffAngularSpeed, MaxData> _data;
	double _last_cost;
	TimeStamp _t0;
	TimeStamp _fitTimeStamp;

	int _rotateSign;					//旋转方向，顺时针为1，逆时针为-1
	const int max_cost = 10;			//回归函数最大Cost
	bool is_params_confirmed = false;

	int sum;
	bool Fitting(bool mode);				//拟合模式：0为全拟合，1为只拟合相位

public:
	int window_size = 2;				//滑窗滤波大小
    Ceres_V2() {
		reset();
	}
	void reset()
	{
		_total_data = 0;
		_data.clear();
		_last_cost = 1e10;
		_t0 = -1;
		_fitTimeStamp = -1;
		is_params_confirmed = false;
		sum = -1;
		_rotateSign = 0;
		_lastBuffFitData = BuffFitData(0, 0, 0, 0, 0);
	};
	bool push(BuffAngularSpeed BuffData) {
		if (BuffData.timeStamp - _data.first().timeStamp > 60000)
		{
			this->reset();
		}
		if (_total_data < MaxData_) {
			_data.push_back(BuffData);
			_total_data++;
		}
		else {
			_data.pop();
			_data.push_back(BuffData);
		}
		if (_total_data == MaxData_) {
			if (!is_params_confirmed) //未得到有效拟合结果时，每40个数据拟合一次，全拟合
			{
				sum++;
				if (!sum)
				{
					_fitTimeStamp = BuffData.timeStamp;
					return Fitting(false);
				}
				else if (sum == 40)
				{
					sum = -1;
					return false;
				}
			}
			else if (BuffData.timeStamp - _fitTimeStamp > 500) // 已经得到有效拟合数据
			{
				_fitTimeStamp = BuffData.timeStamp;
				return Fitting(true);
			}
			return true;
		}
		return false;
	}

	double PredictDeltaAngle(TimeStamp now, TimeStamp predict_timeStamp) const;
	double shiftWindowFilter(BuffAngularSpeed buffSpeed);
};
