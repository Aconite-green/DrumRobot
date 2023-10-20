#include "../include/PathManager.hpp"
#include <time.h>
#include <iostream>
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

void PathManager::operator()(SharedBuffer<can_frame> &buffer)
{
	/////////// 악기를 칠때의 손목위치 assignment
	// Load data from the file
	ifstream inputFile("rT.txt");

	if (!inputFile.is_open()) {
		cerr << "Failed to open the file." << "\n";
	}

	// Read data into a 2D vector
	vector<vector<double>> inst_xyz(6, vector<double>(8, 0));

	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 8; ++j) {
			inputFile >> inst_xyz[i][j];
		}
	}

	// Extract the desired elements
	vector<double> right_B = { 0, 0, 0 };
	vector<double> right_S;
	vector<double> right_FT;
	vector<double> right_MT;
	vector<double> right_HT;
	vector<double> right_HH;
	vector<double> right_R;
	vector<double> right_RC;
	vector<double> right_LC;

	for (int i = 0; i < 3; ++i) {
		right_S.push_back(inst_xyz[i][0]);
		right_FT.push_back(inst_xyz[i][1]);
		right_MT.push_back(inst_xyz[i][2]);
		right_HT.push_back(inst_xyz[i][3]);
		right_HH.push_back(inst_xyz[i][4]);
		right_R.push_back(inst_xyz[i][5]);
		right_RC.push_back(inst_xyz[i][6]);
		right_LC.push_back(inst_xyz[i][7]);
	}

	vector<double> left_B = { 0, 0, 0 };
	vector<double> left_S;
	vector<double> left_FT;
	vector<double> left_MT;
	vector<double> left_HT;
	vector<double> left_HH;
	vector<double> left_R;
	vector<double> left_RC;
	vector<double> left_LC;

	for (int i = 3; i < 6; ++i) {
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
	vector<vector<double>> right_inst = { right_B, right_RC, right_R, right_S, right_HH, right_HH, right_FT, right_MT, right_LC, right_HT };
	vector<vector<double>> left_inst = { left_B, left_RC, left_R, left_S, left_HH, left_HH, left_FT, left_MT, left_LC, left_HT };

	
	// initialize
	const double pi = 3.14159265358979;

	vector<double> R = { 0.368, 0.414, 0.368, 0.414 };
	double s = 0.530;
	double z0 = 0.000;
	
	double theta0_standby = 0;
	double theta1_standby = pi / 2;
	double theta2_standby = pi / 2;
	double theta3_standby = pi / 6;
	double theta4_standby = 2 * pi / 3;
	double theta5_standby = pi / 6;
	double theta6_standby = 2 * pi / 3;

	vector<double> standby_L = { theta0_standby, theta2_standby, theta5_standby, theta6_standby };
	vector<double> standby_R = { theta0_standby, theta1_standby, theta3_standby, theta4_standby };
	vector<double> standby = { theta0_standby, theta1_standby, theta2_standby, theta3_standby, theta4_standby, theta5_standby, theta6_standby };

	vector<double> Q0_LEFT(4, 0);
	vector<double> Q0_RIGHT(4, 0);
	vector<vector<double>> q_ready_L;
	vector<vector<double>> q_ready_R;

	//// 준비자세 배열 생성
	vector<double> Qi;
	int n = 200;
	for (int k = 1; k <= n; ++k) {
		connect cnt_rL(Q0_LEFT, standby_L, k, n, 0, 1);
		Qi = cnt_rL.Run();
		q_ready_L.push_back(Qi);

		connect cnt_rR(Q0_RIGHT, standby_R, k, n, 0, 1);
		Qi = cnt_rR.Run();
		q_ready_R.push_back(Qi);
	}


	/////////// 드럼로봇 악기정보 텍스트 -> 딕셔너리 변환
	map<string, int> instrument_mapping = {
		{"0", 10}, {"1", 3}, {"2", 6}, {"3", 7}, {"4", 9},
		{"5", 4}, {"6", 5}, {"7", 4}, {"8", 8}, {"11", 3},
		{"51", 3}, {"61", 3}, {"71", 3}, {"81", 3}, {"91", 3}
	};

	string score_path = "D:/drumrobot2/codeConfession.txt";
	vector<double> time_arr;
	vector<vector<int>> RA, LA;
	vector<int> RF, LF;

	ifstream file(score_path);
	if (!file.is_open()) {
		cerr << "Error opening file." << endl;
	}

	string line;
	int lineIndex = 0;
	while (getline(file, line)) {
		istringstream iss(line);
		string item;
		vector<string> columns;
		while (getline(iss, item, '\t')) {
			columns.push_back(item);
		}

		vector<int> inst_arr_R(10, 0), inst_arr_L(10, 0);
		time_arr.push_back(stod(columns[1]));

		if (columns[2] != "0") {
			inst_arr_R[instrument_mapping[columns[2]]] = 1;
		}
		if (columns[3] != "0") {
			inst_arr_L[instrument_mapping[columns[3]]] = 1;
		}

		RF.push_back(stoi(columns[6]) == 1 ? 1 : 0);
		LF.push_back(stoi(columns[7]) == 2 ? 1 : 0);

		RA.push_back(inst_arr_R);
		LA.push_back(inst_arr_L);

		lineIndex++;
	}

	file.close();

////////////////////////////////////////////////////////////////////////////////////////////


	//// 걸리는 시간을 절반으로 나눔 (-1, 1, 0, -0.5 구분) => 행 2배
	std::vector<double> timest_arr;

	for (size_t k = 0; k < time_arr.size(); ++k) {
		timest_arr.push_back(time_arr[k] / 2);
		timest_arr.push_back(time_arr[k] / 2);
	}

	int n_time = timest_arr.size();
	int n_inst = 10;

	//////// qd2sd : 악보를 기준으로 1, -1, 0, -0.5
	qd2sd q2s_r(RA);
	qd2sd q2s_l(LA);
	qd2sd_F q2s_F_r(RF);
	qd2sd_F q2s_F_l(LF);
	vector<vector<double>> RAst = q2s_r.Arm_st();
	vector<vector<double>> LAst = q2s_l.Arm_st();
	vector<double> RFst = q2s_F_r.Arm_st();
	vector<double> LFst = q2s_F_l.Arm_st();

	vector<double> P1;
	vector<double> P2;
	vector<int> state_P1(n_time, 0);
	vector<int> state_P2(n_time, 0);
	vector<vector<double>> Q(n_time, vector<double>(7, 0));
	vector<vector<double>> Q_LEFT(n_time, vector<double>(4, 0));
	vector<vector<double>> Q_RIGHT(n_time, vector<double>(4, 0));
	vector<vector<double>> q_LEFT;
	vector<vector<double>> q_RIGHT;

	vector<double> P0_R = { 0.265, -0.391, -0.039837 };
	vector<double> P0_L = { -0.265, -0.391, -0.039837 };

	int start_R = 0;
	int start_L = 0;
	
	///////// IK함수 & connect 함수
	// Assuming RAst and LAst are 2D vectors

	//int Time = 0;
	//clock_t start = clock();

	for (int i = 0; i < n_time; ++i) {

		for (int j = 0; j < n_inst; ++j) {
			if (RAst[i][j] != 0) {
				P1 = right_inst[j];
				state_P1[i] = RAst[i][j];
				start_R = 1;
			}

			if (LAst[i][j] != 0) {
				P2 = left_inst[j];
				state_P2[i] = LAst[i][j];
				start_L = 1;
			}
		}

		if (start_R == 0 && start_L == 0) {
			Q.push_back(standby);
		}
		else if (start_R == 0) {
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

		if (state_P1[i] == -1) {
			Q[i][4] = Q[i][4] + pi / 6;
		}
		else if (state_P1[i] == -0.5) {
			Q[i][4] = Q[i][4] + pi / 18;
		}

		if (state_P2[i] == -1) {
			Q[i][6] = Q[i][6] + pi / 6;
		}
		else if (state_P2[i] == -0.5) {
			Q[i][6] = Q[i][6] + pi / 18;
		}

		Q_LEFT[i] = { Q[i][0], Q[i][2], Q[i][5], Q[i][6] };
		Q_RIGHT[i] = { Q[i][0], Q[i][1], Q[i][3], Q[i][4] };

	}


	/////// 5ms로 쪼개기
	n = round(timest_arr[1] / 0.005);
	/* ///////////////////수정/////////////////////
	for (int k = 0; k < n; ++k) {
		connect cnt_r(Q_RIGHT[0], Q_RIGHT[0], k, n, 0, state_P1[0]);
		vector<double> Qi = cnt_r.Run();
		q_RIGHT[i * n + k] = Qi;

		connect cnt_l(Q_LEFT[0], Q_LEFT[0], k, n, 0, state_P2[0]);
		Qi = cnt_l.Run();
		q_LEFT[i * n + k] = Qi;
	}

	if (i > 0) {
		for (int k = 0; k < n; ++k) {
			connect cnt_r(Q_RIGHT[i - 1], Q_RIGHT[i], k, n, state_P1[i - 1], state_P1[i]);
			vector<double> Qi = cnt_r.Run();
			q_RIGHT[(i - 1) * n + k] = Qi;

			connect cnt_l(Q_LEFT[i - 1], Q_LEFT[i], k, n, state_P2[i - 1], state_P2[i]);
			Qi = cnt_l.Run();
			q_LEFT[(i - 1) * n + k] = Qi;
		}
	}
	*/

	//Time += ((int)clock() - start) / (CLOCKS_PER_SEC / 1000);

	//cout << "TIME : " << Time << "ms\n";

	/*
	cout << "Q : " << Q.size() << " x " << Q[0].size() << "\n";
	for (int i = 0; i < Q.size(); ++i) {
		for (int j = 0; j < Q[0].size(); ++j) {
			cout << Q[i][j] << " ";
		}
		cout << "\n";
	}
	cout << "\n";

	cout << "q_RIGHT : " << q_RIGHT.size() << " x " << q_RIGHT[0].size() << "\n";
	for (int i = 0; i < q_RIGHT.size(); ++i) {
		for (int j = 0; j < q_RIGHT[0].size(); ++j) {
			cout << q_RIGHT[i][j] << " ";
		}
		cout << "\n";
	}
	cout << "\n";

	cout << "q_LEFT : " << q_LEFT.size() << " x " << q_LEFT[0].size() << "\n";
	for (int i = 0; i < q_LEFT.size(); ++i) {
		for (int j = 0; j < q_LEFT[0].size(); ++j) {
			cout << q_LEFT[i][j] << " ";
		}
		cout << "\n";
	}
	cout << "\n";
	

   
    struct can_frame frame;
    for (long unsigned int i = 0; i < q.size(); ++i)    // time num
    {
        int j = 0;  // instrument num
        for (auto &entry : tmotors)
        {

            std::shared_ptr<TMotor> &motor = entry.second;

            float p_des = q[i][j];

            Parser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, 0, 8, 1, 0);

            buffer.push(frame);

            j++;
        }
        cout << "\n";
    }
    */
}