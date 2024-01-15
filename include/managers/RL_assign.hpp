#pragma once
#ifndef RLASSIGN_H
#define RLASSIGN_H
#include <vector>
#include <string>
using namespace std;

class RL_assign
{
	vector<string> arr;
	vector<string> R_hnd;
	vector<string> L_hnd;

public:
	RL_assign(vector<string>& inst_arr);
	vector<string> RH_arr();
	vector<string> LH_arr();
};


#endif // !RLASSIGN_H