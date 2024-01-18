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

class TestManager
{
public:
    TestManager(SystemState &systemStateRef, 
    CanManager &canManagerRef, std::map<std::string, 
    std::shared_ptr<GenericMotor>> &motorsRef);

    void motorInitialize(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);
    void mainLoop();
    void run();
    void TestArr(double t, int cycles, int type, int LnR, double amp[]);

private:
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;
    SystemState &systemState;

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    
    void ApplyDir();
    void getMotorPos();
    void wristarr(vector<vector<double>> &T, int time, int LnR, double amp, int kp[]);
    void waistarr(vector<vector<double>> &T, int time, double amp, int kp[]);
    void arm1arr(vector<vector<double>> &T, int time, int LnR, double amp, int kp[]);
    void arm2arr(vector<vector<double>> &T, int time, int LnR, double amp, int kp[]);
    void arm3arr(vector<vector<double>> &T, int time, int LnR, double amp, int kp[]);
    void SendLoop();

    vector<double> c_MotorAngle = {0, -M_PI / 2, M_PI / 2, M_PI / 4, -M_PI / 2.4, -M_PI / 4, -M_PI / 2.4, 0, 0};
};