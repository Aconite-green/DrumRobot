#include <stdio.h>
#include "../include/CanSocketUtils.hpp"
#include "../include/CommandParser.hpp"
#include "../include/Motor.hpp"
#include "../include/TaskUtility.hpp"
#include "../include/Global.hpp"
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
    PathManager(queue<can_frame> &sendBufferRef, map<string, shared_ptr<TMotor>> &tmotorsRef, std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotorsRef);

    void motorInitialize(map<string, shared_ptr<TMotor>> &tmotorsRef, std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotorsRef);

    void GetMusicSheet();
    void GetReadyArr();
    void PathLoopTask();
    void GetBackArr();

    int end = 0;
    int line = 0;
    vector<vector<double>> p;
    vector<vector<double>> v;

private:
    queue<can_frame> &sendBuffer;
    TMotorCommandParser TParser;
    MaxonCommandParser MParser;
    std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
    std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors;

    // Functions for DrumRobot PathGenerating
    vector<double> c_MotorAngle = {0, 0, 0, 0, 0, 0, 0};
    vector<vector<double>> right_inst;
    vector<vector<double>> left_inst;

    int bpm = 100;
    vector<double> time_arr;
    vector<vector<int>> RA, LA;
    vector<int> RF, LF;

    // Ready Array : 0, 90, 90, 45, 75, -45, -75    (**모터 회전방향에 따라 부호 조절)
    vector<double> standby = {0, M_PI / 2, M_PI / 2, M_PI / 4, M_PI / 2.4, -M_PI / 4, -M_PI / 2.4};

    double p_R = 0; // 오른손 이전 악기 유무
    double p_L = 0; // 왼손 이전 악기 유무
    double c_R = 0; // 오른손 현재 악기 유무
    double c_L = 0; // 왼손 현재 악기 유무

    /*
    vector<double> P1 = {0.265, -0.6187, -0.0532};	 // RightArm Standby
    vector<double> P2 = {-0.265, -0.6187, -0.0532}; // LeftArm Standby
    int n_inst = 10;

    vector<double> R = {0.368, 0.414, 0.368, 0.414};
    double s = 0.530;
    double z0 = 0.000;
    */
    vector<double> P1 = {0.3, -0.45, -0.0866};  // RightArm Standby
    vector<double> P2 = {-0.3, -0.45, -0.0866}; // LeftArm Standby
    int n_inst = 10;

    vector<double> R = {0.500, 0.400, 0.500, 0.400};
    double s = 0.600;
    double z0 = 0.000;

    vector<double> Q1, Q2, Q3, Q4;

    map<string, int> motor_mapping = {
        {"L_arm1", 2}, {"L_arm2", 5}, {"L_arm3", 6}, {"R_arm1", 1}, {"R_arm2", 3}, {"R_arm3", 4}, {"waist", 0}};

    string trimWhitespace(const std::string &str);
    vector<double> connect(vector<double> &Q1, vector<double> &Q2, int k, int n);
    void iconnect(vector<double> &P0, vector<double> &P1, vector<double> &P2, vector<double> &V0, double t1, double t2, double t);
    vector<double> IKfun(vector<double> &P1, vector<double> &P2, vector<double> &R, double s, double z0);
};