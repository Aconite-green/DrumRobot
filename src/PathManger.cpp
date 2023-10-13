#include "../include/PathManager.hpp"
#include <time.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
using namespace std;

PathManager::PathManager(std::map<std::string, std::shared_ptr<TMotor>> &tmotors)
    : tmotors(tmotors)
{
}

void PathManager::operator()(SharedBuffer<can_frame> &buffer)
{
    //////// input value
    vector<vector<int>> qd = {
        {1, 0, 1, 0, 1, 1, 0, 1}, // base
        {0, 0, 0, 0, 1, 0, 0, 0}, // right crash
        {1, 1, 0, 1, 0, 1, 1, 0}, // ride
        {0, 0, 0, 1, 0, 0, 0, 0}, // snare
        {0, 0, 0, 0, 0, 0, 0, 0}, // open high hat
        {0, 0, 0, 0, 0, 0, 0, 0}, // closed high hat
        {0, 0, 1, 0, 1, 0, 0, 1}, // floor tom
        {1, 0, 0, 0, 0, 1, 0, 0}, // mid tom
        {0, 0, 0, 0, 0, 0, 0, 0}, // left crash
        {0, 1, 0, 0, 0, 0, 1, 0}  // high tom
    };

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

    /////////////////////////////////////////////////////////////////////////////////////////
    /*
    // motor initial value
    const double pi = 3.14159265358979;

    double theta5_stay = 0;
    double theta6_stay = pi / 18;
    double theta5_standby = pi / 6;
    double theta6_standby = pi / 3;

    double theta3_standby = theta5_standby;
    double theta4_standby = theta6_standby;
    double theta3_stay = theta5_stay;
    double theta4_stay = theta6_stay;
    */

    vector<double> R = {0.500, 0.400, 0.500, 0.400};
    double s = 0.600;
    double z0 = 0.000;

    int n_time = qd[0].size();
    int n_inst = qd.size();

    ////////// bin2str
    bin2str b2s(qd);
    vector<int> RF = b2s.RF_arr();
    vector<int> LF = b2s.LF_arr();
    vector<string> inst_arr = b2s.Arm_arr();

    ////////// RL_assign
    RL_assign rl_assign(inst_arr);
    vector<string> R_hnd = rl_assign.RH_arr();
    vector<string> L_hnd = rl_assign.LH_arr();

    ///////// str2bin
    str2bin s2b_r(R_hnd);
    str2bin s2b_l(L_hnd);
    vector<vector<int>> RA = s2b_r.Arm_arr();
    vector<vector<int>> LA = s2b_l.Arm_arr();

    //////// qd2sd
    qd2sd q2s_r(RA);
    qd2sd q2s_l(LA);
    qd2sd_F q2s_F_r(RF);
    qd2sd_F q2s_F_l(LF);
    vector<vector<double>> RAst = q2s_r.Arm_st();
    vector<vector<double>> LAst = q2s_l.Arm_st();
    vector<double> RFst = q2s_F_r.Arm_st();
    vector<double> LFst = q2s_F_l.Arm_st();

    ////////////////////////////////////////////////////////////////////////////////////////////
    const int n = 25; // Assuming n is a constant

    int n_time_32 = n_time * 2;
    vector<double> state_P1(n_time_32, 0);
    vector<double> state_P2(n_time_32, 0);
    vector<vector<double>> Q;
    vector<vector<double>> Q_LEFT(n_time_32, vector<double>(4));
    vector<vector<double>> Q_RIGHT(n_time_32, vector<double>(4));
    vector<vector<double>> q(n_time_32 * n, vector<double>(7));
    vector<vector<double>> q_LEFT(n_time_32 * n, vector<double>(4));
    vector<vector<double>> q_RIGHT(n_time_32 * n, vector<double>(4));
    vector<double> P1;
    vector<double> P2;

    ///////// IKfun & connect
    // Assuming RAst and LAst are 2D vectors

    // int Time = 0;
    // clock_t start = clock();

    for (int i = 0; i < n_time_32; ++i)
    {
        for (int j = 0; j < n_inst; ++j)
        {
            if (RAst[i][j] != 0)
            {
                P1 = right_inst[j];
                state_P1[i] = RAst[i][j];
            }

            if (LAst[i][j] != 0)
            {
                P2 = left_inst[j];
                state_P2[i] = LAst[i][j];
            }
        }

        IKfun ikfun(P1, P2, R, s, z0);
        vector<double> Qf = ikfun.Run();
        vector<double> Qi;

        Q.push_back(Qf);
        Q_LEFT[i] = {Q[i][0], Q[i][2], Q[i][5], Q[i][6]};
        Q_RIGHT[i] = {Q[i][0], Q[i][1], Q[i][3], Q[i][4]};

        for (int k = 0; k < n; ++k)
        {
            connect cnt_r(Q_RIGHT[0], Q_RIGHT[0], k, n, 0, state_P1[0]);
            vector<double> Qi = cnt_r.Run();
            q_RIGHT[i * n + k] = Qi;

            connect cnt_l(Q_LEFT[0], Q_LEFT[0], k, n, 0, state_P2[0]);
            Qi = cnt_l.Run();
            q_LEFT[i * n + k] = Qi;
        }

        if (i > 0)
        {
            for (int k = 0; k < n; ++k)
            {
                connect cnt_r(Q_RIGHT[i - 1], Q_RIGHT[i], k, n, state_P1[i - 1], state_P1[i]);
                vector<double> Qi = cnt_r.Run();
                q_RIGHT[(i - 1) * n + k] = Qi;

                connect cnt_l(Q_LEFT[i - 1], Q_LEFT[i], k, n, state_P2[i - 1], state_P2[i]);
                Qi = cnt_l.Run();
                q_LEFT[(i - 1) * n + k] = Qi;
            }
        }
    }

    for (long unsigned int i = 0; i < q_RIGHT.size(); ++i)
    {
        q[i] = {q_RIGHT[i][0], q_RIGHT[i][1], q_LEFT[i][1], q_RIGHT[i][2], q_RIGHT[i][3], q_LEFT[i][2], q_LEFT[i][3]};
    }

    // Time += ((int)clock() - start) / (CLOCKS_PER_SEC / 1000);

    // cout << "TIME : " << Time << "ms\n";

    cout << "Q : " << Q.size() << " x " << Q[0].size() << "\n";
    for (long unsigned int i = 0; i < Q.size(); ++i)
    {
        for (long unsigned int j = 0; j < Q[0].size(); ++j)
        {
            cout << Q[i][j] << " ";
        }
        cout << "\n";
    }
    cout << "\n";

    cout << "q : " << q.size() << " x " << q[0].size() << "\n";
    for (long unsigned int i = 0; i < q.size(); ++i)
    {
        for (long unsigned int j = 0; j < q[0].size(); ++j)
        {
            cout << q[i][j] << " ";
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
    
}