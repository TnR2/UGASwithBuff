#pragma once
/*
Creation Date: 2023/5/24
Latest Update: 2023/5/24
Developer(s): IrumaMegumi 21-WZY 21-WTR
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- power rune identifier with onnx model from yolo
*/

#include <optional>
#include <fstream>

#include <openvino/openvino.hpp>

#include "Core/Identifier/BuffIdentifierInterface.h"
#include "Util/Debug/DebugCanvas.h"
#include "Util/UtilFunctions.h"

#define ROI

class BuffIdentifier_V3 {
private:
	BuffAngle _last_angle;
	BuffAngularSpeed _last_angularSpeed;

	TimeStamp _latest_time;
	cv::Point2f _buffPlate;
	cv::Point2f _rCenter;
	double _radius;

    cv::Point2f _buffRPoint;
    cv::Point2f _buffPlate5Points[5];

	ov::Core _core;
    float _size_model;
    ov::CompiledModel _compiled_model;
    ov::InferRequest _infer_request;
    ov::Output<const ov::Node> _input_port;
	std::vector<cv::Scalar> _colors = { cv::Scalar(255, 0, 0), cv::Scalar(255, 0, 255), cv::Scalar(170, 0, 255),
										cv::Scalar(255, 0, 85), cv::Scalar(255, 0, 170) };

	//std::ofstream _pdata;
	//std::ofstream _vdata;

public:
	BuffIdentifier_V3() {
		// read model from file
        _size_model = 640;
        //_compiled_model = _core.compile_model("../models/buffsj_4000.xml", "AUTO");
        _compiled_model = _core.compile_model("../models/buff_nocolor.onnx", "AUTO"); // bad
        //_compiled_model = _core.compile_model("../models/buff_half.xml", "AUTO");
        _infer_request = _compiled_model.create_infer_request();
        _input_port = _compiled_model.input();

		_latest_time = -1;
		_last_angle.timeStamp = -1;
		_last_angularSpeed.timeStamp = -1;

		//_pdata.open("/home/alliance/Desktop/buff_points.txt");
		//_vdata.open("/home/alliance/Desktop/buff_v.txt");
	}

	BuffIdentifier_V3(const BuffIdentifier_V3&) = delete;
    BuffIdentifier_V3(BuffIdentifier_V3&&) = delete;

	std::optional<Buff5PointIdentifyData> Identify(const cv::Mat& img, const TimeStamp& timeStamp, const ArmorColor enemyColor) {
        // return nullopt if onnx has no result
        _buffPlate5Points[4].x = 0;
		_buffRPoint.x = 0;

		// image pretreat
        cv::Mat letterbox_img = letterbox(img);
        float scale = letterbox_img.size[0] / 640.0;

        cv::Mat blob;
        blob = cv::dnn::blobFromImage(letterbox_img, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true);

		// image input
		ov::Tensor input_tensor(_input_port.get_element_type(), _input_port.get_shape(), blob.ptr(0));

        // set input tensor for model with one input
		_infer_request.set_input_tensor(input_tensor);

		// start inference
		_infer_request.infer();

		// get the inference result
		auto output = _infer_request.get_output_tensor(0);
		auto output_shape = output.get_shape();
		// [8400, 25] 8400 anchors in total, 25 nums in one anchor (x, y, w, h, confidence, data of points (x, y, confidence))
		//std::cout << "The shape of output tensor: " << output_shape << std::endl;

		// temp start
		float* data = output.data<float>();
		cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, data);
		transpose(output_buffer, output_buffer);
		//std::cout << output_buffer.size() << output_buffer.rows;

		float* target[6] = { 0 };
		float confidence[6];

		for (int i = 0; i < output_buffer.rows; ++i) {
			int type, valid = 0;
			float conf;
            float conf_dest = (enemyColor == ArmorColor::Blue ? 0.6f : 1e-3f);
			for (int j = 4; j < 4 + 6; ++j) {
				conf = output_buffer.at<float>(i, j);
				if (conf > conf_dest) {
					type = j - 4;
					++valid;
				}
			}
			if (valid == 1) {
				if (target[type] == nullptr || conf > confidence[type]) {
					target[type] = &output_buffer.at<float>(i, 0);
					confidence[type] = conf;
				}
			}
		}
		for (int ri = 0; ri < 2; ri++) {
			if (target[ri] != nullptr) {
				float rx = target[ri][0] * scale;
				float ry = target[ri][1] * scale;
				_buffRPoint = cv::Point2f(rx, ry);
                circle(img, _buffRPoint, 3, _colors[1], -1);
			}
		}
		for (int i = 2; i < 6; ++i) {
			if (target[i] != nullptr) {
				//std::cout << "Type " << i << ": ";
				for (int j = 4; j < 4 + 6; ++j) {
					//std::cout << target[i][j] << ' ';
				}
				//std::cout << '\n';
				if (i == 2 || i == 3) {
					float cx = target[i][0];
					float cy = target[i][1];
					float w = target[i][2];
					float h = target[i][3];
					int left = int((cx - 0.5 * w) * scale);
					int top = int((cy - 0.5 * h) * scale);
					int width = int(w * scale);
					int height = int(h * scale);
					//cv::rectangle(img, cv::Rect(left, top, width, height), cv::Scalar(0, 0, 255), 1);
					//cv::circle(img, cv::Point(int(cx*scale), int(cy*scale)), 3, _colors[i], -1);
					for (int j = 0; j < 5; j++) {
						float x = target[i][10 + j * 3 + 0] * scale;
						float y = target[i][10 + j * 3 + 1] * scale;
						circle(img, cv::Point(x, y), 3, _colors[j], -1);
						_buffPlate5Points[j] = cv::Point2f(x, y);
					}
				}
			}
			else {
				//std::cout << "Type " << i << ": Not found.\n";
			}
		}
//		cv::namedWindow("YOLOv8-Pose OpenVINO Inference C++ Demo", cv::WINDOW_AUTOSIZE);
//		cv::imshow("YOLOv8-Pose OpenVINO Inference C++ Demo", img);
//		cv::waitKey(1);

		if (_buffPlate5Points[4].x == 0) {
			return std::nullopt; 
		}
		if (_buffRPoint.x == 0 || _buffRPoint.y == 0) {
			return std::nullopt;
		}

		// output buff points data
//		_pdata << "time:" << timeStamp << std::endl;
//		_pdata << "0: " << _buffPlate5Points[0].x << " " << _buffPlate5Points[0].y << std::endl;
//		_pdata << "1: " << _buffPlate5Points[1].x << " " << _buffPlate5Points[1].y << std::endl;
//		_pdata << "2: " << _buffPlate5Points[2].x << " " << _buffPlate5Points[2].y << std::endl;
//		_pdata << "3: " << _buffPlate5Points[3].x << " " << _buffPlate5Points[3].y << std::endl;
//		_pdata << "4: " << _buffPlate5Points[4].x << " " << _buffPlate5Points[4].y << std::endl;
//		_pdata << "R: " << _buffRPoint.x << " " << _buffRPoint.y << std::endl;
//		_pdata << std::endl << std::endl << std::endl;

		_latest_time = timeStamp;
		double v = 0, angle = 0;
		_rCenter = _buffRPoint;
		angle = atan2(_buffPlate5Points[0].y + _buffPlate5Points[1].y - 2 * _rCenter.y,
					  _buffPlate5Points[0].x + _buffPlate5Points[1].x - 2 * _rCenter.x);
		_buffPlate = (	_buffPlate5Points[0] + _buffPlate5Points[1] +
						_buffPlate5Points[2] + _buffPlate5Points[4]) / 4;
		_radius = P2PDis(_buffPlate, _rCenter);
		//std::cout << angle << std::endl;
		//std::cout << _radius << std::endl;
		//std::cout << _last_angle.timeStamp << std::endl;

		if (_last_angle.timeStamp != -1) {
			double delta_angle = angle - _last_angle.angle;
			if (delta_angle > 6.28) delta_angle -= 2 * MathConsts::Pi;
			else if (delta_angle < -6.28) delta_angle += 2 * MathConsts::Pi;
			//std::cout << _latest_time << "  " << delta_angle << std::endl;

			TimeStamp delta_time = _latest_time - _last_angle.timeStamp;
			if (delta_time == 0) return std::nullopt;

			v = 1000 * delta_angle / delta_time;
			//std::cout << "time: " << delta_time << "\t" << "angular speed: " << delta_angle << std::endl;

			_last_angle = BuffAngle(_latest_time, angle);

			_last_angularSpeed = BuffAngularSpeed(_latest_time, v);

			//_vdata << _latest_time << "   " << valid_v << std::endl;

            if constexpr (debugCanvas.buff) {
                cv::circle(debugCanvas.buff.GetMat(), _buffPlate5Points[0], 2, COLOR_RED, 3, 8);
                cv::circle(debugCanvas.buff.GetMat(), _buffPlate5Points[1], 2, COLOR_RED, 3, 8);
                cv::circle(debugCanvas.buff.GetMat(), _buffPlate5Points[2], 2, COLOR_RED, 3, 8);
                cv::circle(debugCanvas.buff.GetMat(), _buffPlate5Points[4], 2, COLOR_RED, 3, 8);
            }

			return Buff5PointIdentifyData(_buffPlate5Points[0], _buffPlate5Points[1],
				_buffPlate5Points[2], _buffPlate5Points[4],
				_rCenter, _radius, _last_angle, _last_angularSpeed);
		}
		else {
            _last_angularSpeed.timeStamp=_last_angle.timeStamp=_latest_time;
            _last_angle.angle = angle;
            _last_angularSpeed.speed=0;

            return std::nullopt;
		}
	}

	cv::Mat letterbox(const cv::Mat& source) {
		int col = source.cols;
		int row = source.rows;
		int _max = MAX(col, row);
		cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
		source.copyTo(result(cv::Rect(0, 0, col, row)));
		return result;
	}
};
