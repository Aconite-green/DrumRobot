#include "../include/PathGeneration.hpp"
#include <iostream>
#include <vector>
using namespace std;

PathGeneration::PathGeneration()
{
}

void PathGeneration::input(vector<double>& Q_val, vector<double>& P1_val, vector<double>& P2_val, double time_val, double state1_val, double state2_val)
{
    c_MotorAngle = Q_val;
    P1 = P1_val;
    P2 = P2_val;
    time = time_val;
    state_P1 = state1_val;
    state_P2 = state2_val;
}

vector<vector<int>> PathGeneration::Path()
{
    int state;
    double timest = time / 2;

    vector<double> P0_R = { 0.265, -0.391, -0.039837 };
	vector<double> P0_L = { -0.265, -0.391, -0.039837 };

    vector<vector<double>> Q(2, vector<double>(7, 0));

    if (start_R == 0) {
        IKfun ikfun(P0_R, P2, R, s, z0);
        vector<double> Qf = ikfun.Run();

        Q.push_back(Qf);
    }
    else if (start_L == 0) {
        IKfun ikfun(P1, P0_L, R, s, z0);
        vector<double> Qf = ikfun.Run();

        Q.push_back(Qf);
    }
    else {
        IKfun ikfun(P1, P2, R, s, z0);
        vector<double> Qf = ikfun.Run();

        Q.push_back(Qf);
    }

}