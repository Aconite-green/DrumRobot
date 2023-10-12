#include <iostream>
#include <vector>
#include <cmath>
#include "connect.hpp"
using namespace std;

connect::connect(vector<double>& q1, vector<double>& q2, int k_val, int n_val, double s1_val, double s2_val)
{
	Q1 = q1;
	Q2 = q2;
	k = k_val;
	n = n_val;
	s1 = s1_val;
	s2 = s2_val;
}
vector<double> connect::Run() {
	std::vector<double> A, B;
    const double PI = 3.14159265358979;

    // Compute A and B
    for (size_t i = 0; i < Q1.size(); ++i) {
        A.push_back(0.5 * (Q1[i] - Q2[i]));
        B.push_back(0.5 * (Q1[i] + Q2[i]));
    }

    // Compute Qi using the provided formula
    for (size_t i = 0; i < Q1.size(); ++i) {
        double val = A[i] * cos(PI * k / n) + B[i];
        Qi.push_back(val);
    }

    return Qi;
}