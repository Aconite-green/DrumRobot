#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include "str2bin.hpp"
using namespace std;

str2bin::str2bin(vector<string>& hnd_arr)
{
    str_arr = hnd_arr;
}

vector<vector<int>> str2bin::Arm_arr()
{
    int n_inst = 10;
    int n_time = str_arr.size();
    vector<vector<int>> bin_arr(n_time, vector<int>(n_inst, 0));

    vector<string> instrument_names = { "B", "CC_R", "R", "S", "HH", "L_foot", "FT", "MT", "CC_L", "HT" };

    for (int i = 0; i < n_time; ++i) {
        for (int j = 0; j < n_inst; ++j) {
            if (str_arr[i] == instrument_names[j]) {
                bin_arr[i][j] = 1;
            }
        }
    }

    return bin_arr;
}