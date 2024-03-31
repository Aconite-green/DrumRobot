#pragma once

#include <stdio.h>
#include "../include/managers/CanManager.hpp"
#include "../include/managers/PathManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/tasks/SystemState.hpp"
#include <map>
#include <memory>
#include <string>
#include <functional>
#include <queue>
#include <algorithm>
#include <thread>
#include <cerrno>  // errno
#include <cstring> // strerror
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <iostream>
#include <vector>
#include <limits>
#include <ctime>
#include <fstream>
#include <atomic>
#include <cmath>
#include <chrono>
#include <set>

using namespace std;

class TestManager
{
public:
    TestManager(State &stateRef, CanManager &canManagerRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    void SendTestProcess();

private:
    State &state;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;

    vector<string> InputData;

    /*For SendTestProcess*/
    double q[9] = {0.0};
    double R_xyz[3] = {0.0};
    double L_xyz[3] = {0.0};
    double part_length[6] = {0.363, 0.3835, 0.363, 0.3835, 0.417, 0.417};
    double s = 0.600;  ///< 허리 길이.
    double z0 = 1.026; ///< 바닥부터 허리까지의 높이.

    map<std::string, int> motor_mapping = { ///< 각 관절에 해당하는 열 정보.
        {"waist", 0},
        {"R_arm1", 1},
        {"L_arm1", 2},
        {"R_arm2", 3},
        {"R_arm3", 4},
        {"L_arm2", 5},
        {"L_arm3", 6},
        {"R_wrist", 7},
        {"L_wrist", 8},
        {"maxonForTest", 8}};

    std::shared_ptr<GenericMotor> virtualMaxonMotor;
    int maxonMotorCount = 0;
     struct can_frame frame;

    /*Value Test Code*/
    void getMotorPos(double c_MotorAngle[]);
    vector<double> connect(double Q1[], double Q2[], int k, int n);
    vector<double> ikfun_final(double pR[], double pL[], double part_length[], double s, double z0);
    void GetArr(double arr[]);
     

    /*Hayeon Test Code*/
    void mkArr(vector<string> &motorName, int time, int cycles, int LnR, double amp);
    void SendLoop();
    void parse_and_save_to_csv(const std::string &csv_file_name);
    void multiTestLoop();
    void TestArr(double t, int cycles, int type, int LnR, double amp[]);

    /*Stick Test Code*/
    void TestStickLoop();
    void TestStick(const std::string selectedMotor, int des_tff, float tffThreshold, float posThreshold, int backTorqueUnit);
    bool dct_fun(float positions[], float vel_th);
};
