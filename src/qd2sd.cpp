#include "qd2sd.hpp"
#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

qd2sd::qd2sd(vector<vector<int>>& Arm_arr)
{
	n_inst = Arm_arr[0].size();
	n_time = Arm_arr.size();

	qd_arr.resize(n_time, vector<int>(n_inst));

	for (int i = 0; i < n_time; i++) {
		for (int j = 0; j < n_inst; j++) {
			qd_arr[i][j] = Arm_arr[i][j];
		}
	}
}

double calculateNorm(const vector<double>& v) {
    double sumOfSquares = 0;

    for (long unsigned int i = 0; i < v.size(); ++i) {
        sumOfSquares += v[i] * v[i];
    }

    return sqrt(sumOfSquares);
}

vector<vector<double>> qd2sd::Arm_st()
{
    sd_arr.resize(2 * n_time, vector<double>(n_inst, 0.0));

    for (int k = 0; k < n_time; ++k) {
        for (int j = 0; j < n_inst; ++j) {
            sd_arr[2 * k][j] = qd_arr[k][j];
        }
    }

    for (int i = 1; i < 2 * n_time; ++i) {
        for (int j = 0; j < n_inst; ++j) {
            if ((sd_arr[i - 1][j] == 1 || sd_arr[i - 1][j] == -0.5) && calculateNorm(sd_arr[i]) == 0) {
                sd_arr[i][j] = -0.5;
            }
        }
    }

    for (int i = 2 * n_time - 2; i >= 0; --i) {
        for (int j = 0; j < n_inst; ++j) {
            if (sd_arr[i + 1][j] == 1 && sd_arr[i][j] != 1) {
                fill(sd_arr[i].begin(), sd_arr[i].end(), 0);
                sd_arr[i][j] = -1;
            }
        }
    }

	return sd_arr;
}