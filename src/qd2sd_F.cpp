#include "qd2sd_F.hpp"
#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

qd2sd_F::qd2sd_F(vector<int>& Arm_arr)
{
    n_time = Arm_arr.size();

    qd_arr = Arm_arr;
}

vector<double> qd2sd_F::Arm_st()
{
    sd_arr.resize(n_time*2, 0.0);

    for (int k = 0; k < n_time; ++k) {
        sd_arr[2 * k] = qd_arr[k];
    }

    for (int i = 1; i < 2 * n_time; ++i) {
        if ((sd_arr[i - 1] == 1 || sd_arr[i - 1] == -0.5) && sd_arr[i] == 0) {
            sd_arr[i] = -0.5;
        }
    }

    for (int i = 2 * n_time - 2; i >= 0; --i) {
        if (sd_arr[i + 1] == 1 && sd_arr[i] != 1) {
            sd_arr[i] = -1;
        }
    }

    return sd_arr;
}