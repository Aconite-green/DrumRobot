#pragma once

#include <stdio.h>
#include "../include/managers/CanManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/tasks/TaskUtility.hpp"
#include "../include/usbio/Global.hpp"
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

class PathManager
{

public:
    PathManager(SystemState &systemStateRef,
                CanManager &canManagerRef,
                std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    void ApplyDir();
    void GetDrumPositoin();
    void GetMusicSheet();
    void PathLoopTask();
    void GetArr(vector<double> &arr);
    void TestArr(double t, int cycles, int type, int LnR);

    // 실시간X, 전체 악보 처리 후 연주
    int total = 0;
    int line = 0;

    // 악보에 따른 position & velocity 값 저장 (5ms 단위)
    vector<vector<double>> p;
    vector<vector<double>> v;

    //     Ready Array      : waist, R_arm1, L_arm1, R_arm2, R_arm3, L_arm2, L_arm3, R_wrist, L_wrist
    //                      : 0    , 90    , 90    , 45    , 75    , 45    , 75    , 0      , 0
    vector<double> standby = {0, M_PI / 2, M_PI / 2, M_PI / 4, M_PI / 2.4, M_PI / 4, M_PI / 2.4, 0, 0};
    vector<double> backarr = {0, M_PI / 2, M_PI / 2, 0, 0, 0, 0, M_PI / 3, M_PI / 3};

private:
    TMotorCommandParser TParser;
    MaxonCommandParser MParser;
    
    SystemState &systemState;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    // Functions for DrumRobot PathGenerating
    vector<double> c_MotorAngle = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    vector<vector<double>> right_inst;
    vector<vector<double>> left_inst;

    double bpm = 10;
    vector<double> time_arr;
    vector<vector<int>> RA, LA;
    vector<int> RF, LF;

    double p_R = 0; // 오른손 이전 악기 유무
    double p_L = 0; // 왼손 이전 악기 유무
    double c_R = 0; // 오른손 현재 악기 유무
    double c_L = 0; // 왼손 현재 악기 유무

    double r_wrist = 0.0;
    double l_wrist = 0.0;
    int n_inst = 10;

    /* 실측값
    vector<double> P1 = {0.265, -0.6187, -0.0532};	 // RightArm Standby
    vector<double> P2 = {-0.265, -0.6187, -0.0532}; // LeftArm Standby
    vector<double> R = {0.368, 0.414, 0.368, 0.414};
    double s = 0.530;
    double z0 = 0.000;
    */

    // 임의값
    vector<double> P1 = {0.3, -0.45, -0.0866};  // RightArm Standby
    vector<double> P2 = {-0.3, -0.45, -0.0866}; // LeftArm Standby
    vector<double> R = {0.500, 0.400, 0.500, 0.400};
    double s = 0.600;
    double z0 = 0.000;

    // Q1, Q3 : 악기를 연주하기 전 들어올린 상태 / Q2 : 이번에 치는 악기 위치 / Q4 : 다음에 치는 악기 위치
    vector<double> Q1, Q2, Q3, Q4;

    // 각 악기별 치는 손목 각도
    vector<double> wrist = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // 각 관절에 해당하는 열
    map<string, int> motor_mapping = {
        {"waist", 0}, {"R_arm1", 1}, {"L_arm1", 2}, {"R_arm2", 3}, {"R_arm3", 4}, {"L_arm2", 5}, {"L_arm3", 6}, {"R_wrist", 7}, {"L_wrist", 8}};
    // 각 열에 해당하는 관절방향
    map<int, int> motor_dir = { // 1 : CW , -1 : CCW
        {0, 1},
        {1, 1},
        {2, 1},
        {3, 1},
        {4, 1},
        {5, 1},
        {6, 1},
        {7, 1},
        {8, 1}};

    vector<double> connect(vector<double> &Q1, vector<double> &Q2, int k, int n);
    void iconnect(vector<double> &P0, vector<double> &P1, vector<double> &P2, vector<double> &V0, double t1, double t2, double t);
    vector<double> IKfun(vector<double> &P1, vector<double> &P2, vector<double> &R, double s, double z0);
    void getDrummingPosAndAng();
    void getMotorPos();
    void getQ1AndQ2();
    void getQ3AndQ4();
    void Motors_sendBuffer();
};