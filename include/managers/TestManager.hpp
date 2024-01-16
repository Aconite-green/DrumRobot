#include <stdio.h>
#include "../include/managers/CanManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/tasks/TaskUtility.hpp"
#include "../include/usbio/Global.hpp"
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
    TestManager(queue<can_frame> &sendBufferRef, queue<can_frame> &recieveBufferRef, map<string, shared_ptr<TMotor>> &tmotorsRef, std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotorsRef);

    void motorInitialize(map<string, shared_ptr<TMotor>> &tmotorsRef, std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotorsRef);

    void run();
    void TestArr(double t, int cycles, int type, int LnR);

private:
    queue<can_frame> &sendBuffer;
    queue<can_frame> &recieveBuffer;
    TMotorCommandParser TParser;
    MaxonCommandParser MParser;
    std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
    std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors;

    // 각 관절에 해당하는 열
    map<string, int> motor_mapping = {
        {"waist", 0}, {"R_arm1", 1}, {"L_arm1", 2}, {"R_arm2", 3}, {"R_arm3", 4}, {"L_arm2", 5}, {"L_arm3", 6}, {"R_wrist", 7}, {"L_wrist", 8}};
    // 각 열에 해당하는 관절방향
    map<int, int> motor_dir = {     // 1 : CW , -1 : CCW
        {0, 1}, {1, 1}, {2, 1}, {3, 1}, {4, 1}, {5, 1}, {6, 1}, {7, 1}, {8, 1}};


    void ApplyDir();
    void waistarr(vector<vector<double>> &T, int time);
    void arm1arr(vector<vector<double>> &T, int time, int LnR);
    void arm2arr(vector<vector<double>> &T, int time, int LnR);
    void arm3arr(vector<vector<double>> &T, int time, int LnR);
};