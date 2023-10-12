#include <iostream>
#include <vector>
#include <string>
#include "bin2str.hpp"
using namespace std;

bin2str::bin2str(vector<vector<int>>& qd)
{
	n_time = qd[0].size();
	n_inst = qd.size();

	qd_arr.resize(n_inst, vector<int>(n_time));

	for (int i = 0; i < n_inst; i++) {
		for (int j = 0; j < n_time; j++) {
			qd_arr[i][j] = qd[i][j];
		}
	}
}

vector<string> bin2str::Arm_arr() 
{	
	string instrument_names[10] = {"B", "CC_R", "R", "S", "HH", "L_foot", "FT", "MT", "CC_L", "HT"};

	inst_arr.resize(n_time);

	for (int i = 0; i < n_time; ++i) {
		for (int j = 0; j < n_inst; ++j) {
			if (qd_arr[j][i] == 1) {
				if (inst_arr[i].empty()) {
					inst_arr[i] = instrument_names[j];
				}
				else {
					inst_arr[i] += "+" + instrument_names[j];
				}
			}
		}
	}
	
	return inst_arr;
}

vector<int> bin2str::RF_arr() 
{
	for (int i = 0; i < n_time; i++) {
		RF.push_back(qd_arr[0][i]);
		qd_arr[0][i] = 0;
	}

	return RF;
}

vector<int> bin2str::LF_arr() 
{
	for (int i = 0; i < n_time; i++) {
		LF.push_back(qd_arr[5][i]);
		qd_arr[5][i] = 0;
	}

	return LF;
}