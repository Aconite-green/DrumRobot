#pragma once
#ifndef IKFUN_H
#define IKFUN_H
#include <vector>
using namespace std;

class IKfun
{
	vector<double> P1;
	vector<double> P2;
	vector<double> R;
	double s;
	double z0;
	vector<double> Qf;
public:
	IKfun(vector<double>& p1, vector<double>& p2, vector<double>& r, double s_val, double z0_val);
	vector<double> Run();
};

#endif // !IKFUN_H