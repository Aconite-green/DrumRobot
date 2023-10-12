#pragma once
#ifndef BIN2STR_H
#define BIN2STR_H
#include <vector>
#include <string>
using namespace std;

class bin2str
{
	vector<vector<int>> qd_arr;
	vector<string> inst_arr;
	vector<int> RF;
	vector<int> LF;
	int n_time, n_inst;

public:
	bin2str(vector<vector<int>>& qd);
	vector<string> Arm_arr();
	vector<int> RF_arr();
	vector<int> LF_arr();
};

#endif // !BIN2STR_H