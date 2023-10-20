#pragma once
#ifndef PATHGENERATION_H
#define PATHGENERATION_H
#include "SharedBuffer.hpp"
#include "CommandParser.hpp"
#include "Motor.hpp"
#include "bin2str.hpp"
#include "RL_assign.hpp"
#include "str2bin.hpp"
#include "qd2sd.hpp"
#include "qd2sd_F.hpp"
#include "IKfun.hpp"
#include "connect.hpp"
#include <vector>
#include <string>
using namespace std;

class PathGeneration
{
	vector<double> c_MotorAngle;
	vector<double> P1;
	vector<double> P2;
	double time;
	double state_P1;
	double state_P2;
	int start_R = 0;
	int start_L = 0;
	vector<double> R = { 0.368, 0.414, 0.368, 0.414 };
	double s = 0.530;
	double z0 = 0.000;

public:
	PathGeneration();
	vector<vector<int>> Path();
	void input(vector<double>& Q_val, vector<double>& P1_val, vector<double>& P2_val, double time_val, double state1_val, double state2_val);
};


#endif // !PATHGENERATION_H