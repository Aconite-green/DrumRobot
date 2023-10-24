#pragma once
#include "SharedBuffer.hpp"
#include "CommandParser.hpp"
#include "Motor.hpp"
#include "bin2str.hpp"
#include "RL_assign.hpp"
#include "str2bin.hpp"
#include "qd2sd.hpp"
#include "qd2sd_F.hpp"
#include "IKfun.hpp"
#include "connect.hpp"
#include <map>
#include <algorithm>

class PathManager
{
public:
    PathManager(std::map<std::string, std::shared_ptr<TMotor>> &tmotors);
    void ready(SharedBuffer<can_frame> &buffer);
    string trimWhitespace(const std::string& str);
    void operator()(SharedBuffer<can_frame> &buffer);

private:
    TMotorCommandParser Parser;
    std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
    
    const double pi = 3.14159265358979;

	double theta0_standby = 0;
	double theta1_standby = pi / 2;
	double theta2_standby = pi / 2;
	double theta3_standby = pi / 6;
	double theta4_standby = 2 * pi / 3;
	double theta5_standby = pi / 6;
	double theta6_standby = 2 * pi / 3;

	vector<double> standby_L = {theta0_standby, theta2_standby, theta5_standby, theta6_standby};
	vector<double> standby_R = {theta0_standby, theta1_standby, theta3_standby, theta4_standby};
	vector<double> standby = {theta0_standby, theta1_standby, theta2_standby, theta3_standby, theta4_standby, theta5_standby, theta6_standby};

    struct can_frame frame;
	vector<double> c_MotorAngle;

    vector<vector<double>> right_inst;
    vector<vector<double>> left_inst;

    int bpm = 80;
    vector<double> time_arr;
	vector<vector<int>> RA, LA;
	vector<int> RF, LF;
};