#pragma once
#ifndef QD2SD
#define QD2SD
#include <vector>
#include <string>
using namespace std;

class qd2sd
{
	vector<vector<int>> qd_arr;
	vector<vector<double>> sd_arr;
	int n_time, n_inst;

public:
	qd2sd(vector<vector<int>>& Arm_arr);
	vector<vector<double>> Arm_st();
};


#endif // !QD2SD