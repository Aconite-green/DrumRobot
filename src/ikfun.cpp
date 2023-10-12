#include <iostream>
#include <cmath>
#include <vector>
#include <limits>
#include <time.h>
#include "IKfun.hpp"
using namespace std;

IKfun::IKfun(vector<double>& p1, vector<double>& p2, vector<double>& r, double s_val, double z0_val)
{
    P1 = p1;
    P2 = p2;
    R = r;
    s = s_val;
    z0 = z0_val;
}
vector<double> IKfun::Run() {

    const double PI = 3.14159265358979;

    double X1 = P1[0], Y1 = P1[1], z1 = P1[2];
    double X2 = P2[0], Y2 = P2[1], z2 = P2[2];
    double r1 = R[0], r2 = R[1], r3 = R[2], r4 = R[3];

    int j = 0;
    vector<double> the3(100);
    for (int i = 0; i < 100; i++) {
        the3[i] = -PI / 2 + (PI * i) / 99;
    }

    double zeta = z0 - z2;

    double det_the4;
    double the34;
    double the4;
    double r;
    double det_the1;
    double the1;
    double det_the0;
    double the0;
    double L;
    double det_the2;
    double the2;
    double T;
    double det_the5;
    double sol;
    double the5;
    double alpha, beta, gamma;
    double det_the6;
    double rol;
    double the6;
    double Z;


    vector<double> Q0;
    vector<double> Q1;
    vector<double> Q2;
    vector<double> Q3;
    vector<double> Q4;
    vector<double> Q5;
    vector<double> Q6;

    for (int i = 0; i < 99; i++) {
        det_the4 = (z0 - z1 - r1 * cos(the3[i])) / r2;

        if (det_the4 < 1 && det_the4 > -1) {
            the34 = acos((z0 - z1 - r1 * cos(the3[i])) / r2);
            the4 = the34 - the3[i];

            if (the4 > 0) {
                r = r1 * sin(the3[i]) + r2 * sin(the34);

                det_the1 = (X1 * X1 + Y1 * Y1 - r * r - s * s / 4) / (s * r);
                if (det_the1 < 1 && det_the1 > -1) {
                    the1 = acos(det_the1);

                    alpha = asin(X1 / sqrt(X1 * X1 + Y1 * Y1));
                    det_the0 = (s / 4 + (X1 * X1 + Y1 * Y1 - r * r) / s) / sqrt(X1 * X1 + Y1 * Y1);
                    if (det_the0 < 1 && det_the0 > -1) {
                        the0 = asin(det_the0) - alpha;

                        L = sqrt(pow(X2 - 0.5 * s * cos(the0 + PI), 2) +
                            pow(Y2 - 0.5 * s * sin(the0 + PI), 2));
                        det_the2 = (X2 + 0.5 * s * cos(the0)) / L;

                        if (det_the2 < 1 && det_the2 > -1) {
                            the2 = acos(det_the2) - the0;

                            T = (zeta * zeta + L * L + r3 * r3 - r4 * r4) / (r3 * 2);
                            det_the5 = L * L + zeta * zeta - T * T;

                            if (det_the5 > 0) {
                                sol = T * L - abs(zeta) * sqrt(L * L + zeta * zeta - T * T);
                                sol /= (L * L + zeta * zeta);
                                the5 = asin(sol);

                                alpha = L - r3 * sin(the5);
                                beta = r4 * sin(the5);
                                gamma = r4 * cos(the5);

                                det_the6 = gamma * gamma + beta * beta - alpha * alpha;

                                if (det_the6 > 0) {
                                    rol = alpha * beta - abs(gamma) * sqrt(det_the6);
                                    rol /= (beta * beta + gamma * gamma);
                                    the6 = acos(rol);
                                    Z = z0 - r1 * cos(the5) - r2 * cos(the5 + the6);

                                    if (Z < z2 + 0.001 && Z > z2 - 0.001) {
                                        Q0.push_back(the0);
                                        Q1.push_back(the1);
                                        Q2.push_back(the2);
                                        Q3.push_back(the3[i]);
                                        Q4.push_back(the4);
                                        Q5.push_back(the5);
                                        Q6.push_back(the6);
                                        
                                        j++;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    vector<vector<double>> Q;
    Q.push_back(Q0);
    Q.push_back(Q1);
    Q.push_back(Q2);
    Q.push_back(Q3);
    Q.push_back(Q4);
    Q.push_back(Q5);
    Q.push_back(Q6);

    // Find the median index
    int num_columns = Q[0].size();
    int index_theta0_min = 0, index_theta0_max = 0;

    // Find index of minimum and maximum values in the first row of A
    for (int i = 1; i < num_columns; i++) {
        if (Q[0][i] > Q[0][index_theta0_max])
            index_theta0_max = i;
        if (Q[0][i] < Q[0][index_theta0_min])
            index_theta0_min = i;
    }

    // Calculate the median index of the min and max
    int index_theta0_med = round((index_theta0_min + index_theta0_max) / 2);

    Qf.resize(7);

    for (int i = 0; i < 7; i++) {
        Qf[i] = Q[i][index_theta0_med];
    }

    return Qf;
}