#pragma once
#ifndef QD2SD_F
#define QD2SD_F
#include <vector>
#include <string>
using namespace std;

class qd2sd_F
{
	vector<int> qd_arr;
	vector<double> sd_arr;
	int n_time, n_inst;

public:
	qd2sd_F(vector<int>& Arm_arr);
	vector<double> Arm_st();
};


#endif // !QD2SD_F