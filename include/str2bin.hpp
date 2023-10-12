#pragma once
#ifndef STR2BIN_H
#define STR2BIN_H
#include <vector>
#include <string>
using namespace std;

class str2bin
{
	vector<string> str_arr;
	vector<vector<int>> bin_arr;

public:
	str2bin(vector<string>& hnd_arr);
	vector<vector<int>> Arm_arr();
};


#endif // !STR2BIN_H