#include "../include/PathManager.hpp"
#include <time.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <sstream>
#include <fstream>
#include <cmath>
#include <map>
using namespace std;

PathManager::PathManager(std::map<std::string, std::shared_ptr<TMotor>> &tmotors)
	: tmotors(tmotors)
{
}

std::string trimWhitespace(const std::string& str) {
    size_t first = str.find_first_not_of(" \t");
    if (std::string::npos == first) {
        return str;
    }
    size_t last = str.find_last_not_of(" \t");
    return str.substr(first, (last - first + 1));
}

void PathManager::operator()(SharedBuffer<can_frame> &buffer)
{
	/////////// 악기를 칠때의 손목위치 assignment
	// Load data from the file
	ifstream inputFile("../include/rT.txt");

	if (!inputFile.is_open())
	{
		cerr << "Failed to open the file."
			 << "\n";
	}

	// Read data into a 2D vector
	vector<vector<double>> inst_xyz(6, vector<double>(8, 0));

	for (int i = 0; i < 6; ++i)
	{
		for (int j = 0; j < 8; ++j)
		{
			inputFile >> inst_xyz[i][j];
		}
	}

	// Extract the desired elements
	vector<double> right_B = {0, 0, 0};
	vector<double> right_S;
	vector<double> right_FT;
	vector<double> right_MT;
	vector<double> right_HT;
	vector<double> right_HH;
	vector<double> right_R;
	vector<double> right_RC;
	vector<double> right_LC;

	for (int i = 0; i < 3; ++i)
	{
		right_S.push_back(inst_xyz[i][0]);
		right_FT.push_back(inst_xyz[i][1]);
		right_MT.push_back(inst_xyz[i][2]);
		right_HT.push_back(inst_xyz[i][3]);
		right_HH.push_back(inst_xyz[i][4]);
		right_R.push_back(inst_xyz[i][5]);
		right_RC.push_back(inst_xyz[i][6]);
		right_LC.push_back(inst_xyz[i][7]);
	}

	vector<double> left_B = {0, 0, 0};
	vector<double> left_S;
	vector<double> left_FT;
	vector<double> left_MT;
	vector<double> left_HT;
	vector<double> left_HH;
	vector<double> left_R;
	vector<double> left_RC;
	vector<double> left_LC;

	for (int i = 3; i < 6; ++i)
	{
		left_S.push_back(inst_xyz[i][0]);
		left_FT.push_back(inst_xyz[i][1]);
		left_MT.push_back(inst_xyz[i][2]);
		left_HT.push_back(inst_xyz[i][3]);
		left_HH.push_back(inst_xyz[i][4]);
		left_R.push_back(inst_xyz[i][5]);
		left_RC.push_back(inst_xyz[i][6]);
		left_LC.push_back(inst_xyz[i][7]);
	}

	// Combine the elements into right_inst and left_inst
	vector<vector<double>> right_inst = {right_B, right_RC, right_R, right_S, right_HH, right_HH, right_FT, right_MT, right_LC, right_HT};
	vector<vector<double>> left_inst = {left_B, left_RC, left_R, left_S, left_HH, left_HH, left_FT, left_MT, left_LC, left_HT};

	/////////// 드럼로봇 악기정보 텍스트 -> 딕셔너리 변환
	map<string, int> instrument_mapping = {
		{"0", 10}, {"1", 3}, {"2", 6}, {"3", 7}, {"4", 9}, {"5", 4}, {"6", 5}, {"7", 4}, {"8", 8}, {"11", 3}, {"51", 3}, {"61", 3}, {"71", 3}, {"81", 3}, {"91", 3}};

	string score_path = "../include/codeConfession.txt";
	vector<double> time_arr;
	vector<vector<int>> RA, LA;
	vector<int> RF, LF;

	ifstream file(score_path);
	if (!file.is_open())
	{
		cerr << "Error opening file." << endl;
	}

	string line;
	int lineIndex = 0;
	while (getline(file, line))
	{
		istringstream iss(line);
		string item;
		vector<string> columns;
		while (getline(iss, item, '\t'))
		{
			item = trimWhitespace(item);
			columns.push_back(item);
		}

		vector<int> inst_arr_R(10, 0), inst_arr_L(10, 0);
		time_arr.push_back(stod(columns[1]));

		if (columns[2] != "0")
		{
			inst_arr_R[instrument_mapping[columns[2]]] = 1;
		}
		if (columns[3] != "0")
		{
			inst_arr_L[instrument_mapping[columns[3]]] = 1;
		}

		RF.push_back(stoi(columns[6]) == 1 ? 1 : 0);
		LF.push_back(stoi(columns[7]) == 2 ? 1 : 0);

		RA.push_back(inst_arr_R);
		LA.push_back(inst_arr_L);

		lineIndex++;
	}

	file.close();

	// initialize
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

	vector<double> Q0(7, 0);
	vector<vector<double>> q_ready;

	//// 준비자세 배열 생성
	struct can_frame frame;
	vector<double> c_MotorAngle;
	int n = 200;
	for (int k = 1; k <= n; ++k)
	{
		connect cnt(Q0, standby, k, n);
		c_MotorAngle = cnt.Run();
		q_ready.push_back(c_MotorAngle);

		int j = 0; // motor num
		for (auto &entry : tmotors)
		{
			std::shared_ptr<TMotor> &motor = entry.second;
			float p_des = c_MotorAngle[j];
			Parser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, 0, 8, 1, 0);
			buffer.push(frame);

			j++;
		}
		// cout << "\n";
	}

	////////////////////////////////////////////////////////////////////////////////////////////

	double p_R = 0; // 오른손 이전 악기 유무
	double p_L = 0; // 왼손 이전 악기 유무
	double c_R = 0; // 오른손 현재 악기 유무
	double c_L = 0; // 왼손 현재 악기 유무재

	vector<double> P1 = {0.265, -0.391, -0.039837};	 // RightArm Standby
	vector<double> P2 = {-0.265, -0.391, -0.039837}; // LeftArm Standby
	int n_inst = 10;

	/*
	vector<double> R = {0.368, 0.414, 0.368, 0.414};
	double s = 0.530;
	double z0 = 0.000;
	*/
	vector<double> R = {0.500, 0.400, 0.500, 0.400};
	double s = 0.600;
	double z0 = 0.000;
	vector<vector<double>> Q(2, vector<double>(7, 0));
	vector<vector<double>> q;

	for (long unsigned int i = 0; i < RF.size(); i++)
	{

		c_R = 0;
		c_L = 0;

		for (int j = 0; j < n_inst; ++j)
		{
			if (RA[i][j] != 0)
			{
				P1 = right_inst[j];
				c_R = 1;
			}
			if (LA[i][j] != 0)
			{
				P2 = left_inst[j];
				c_L = 1;
			}
		}

		// int Time = 0;
		// clock_t start = clock();

		if (c_R == 0 && c_L == 0)
		{ // 왼손 & 오른손 안침
			Q[0] = c_MotorAngle;
			if (p_R == 1)
			{
				Q[0][4] = Q[0][4] + pi / 18;
			}
			if (p_L == 1)
			{
				Q[0][6] = Q[0][6] + pi / 18;
			}
			Q[1] = Q[0];
		}
		else
		{
			IKfun ikfun(P1, P2, R, s, z0);
			Q[0] = ikfun.Run();
			Q[1] = Q[0];
			if (c_R == 0)
			{ // 왼손만 침
				Q[0][4] = Q[0][4] + pi / 18;
				Q[1][4] = Q[1][4] + pi / 18;
				Q[0][6] = Q[0][6] + pi / 6;
			}
			if (c_L == 0)
			{ // 오른손만 침
				Q[0][4] = Q[0][4] + pi / 6;
				Q[0][6] = Q[0][6] + pi / 18;
				Q[1][6] = Q[1][6] + pi / 18;
			}
			else
			{ // 왼손 & 오른손 침
				Q[0][4] = Q[0][4] + pi / 6;
				Q[0][6] = Q[0][6] + pi / 6;
			}
		}

		p_R = c_R;
		p_L = c_L;

		vector<double> Qi;
		double timest = time_arr[i] / 2;
		n = round(timest / 0.002);
		for (int k = 0; k < n; ++k)
		{
			connect cnt(c_MotorAngle, Q[0], k, n);
			Qi = cnt.Run();
			q.push_back(Qi);

			int j = 0; // motor num
			for (auto &entry : tmotors)
			{
				std::shared_ptr<TMotor> &motor = entry.second;
				float p_des = Qi[j];
				Parser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, 0, 8, 1, 0);
				buffer.push(frame);

				j++;
			}
			// cout << "\n";
		}
		for (int k = 0; k < n; ++k)
		{
			connect cnt(Q[0], Q[1], k, n);
			Qi = cnt.Run();
			q.push_back(Qi);

			int j = 0; // motor num
			for (auto &entry : tmotors)
			{
				std::shared_ptr<TMotor> &motor = entry.second;
				float p_des = Qi[j];
				Parser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, 0, 13, 1, 0);
				buffer.push(frame);

				j++;
			}
			// cout << "\n";
		}

		c_MotorAngle = Qi;

		// Time += ((int)clock() - start) / (CLOCKS_PER_SEC / 1000);
		// cout << "TIME : " << Time << "ms\n";
	}

}