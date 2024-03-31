#pragma once

#include <stdio.h>
#include "../include/managers/CanManager.hpp"
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
    double L_xyz[3] = {0.0};
    double R_xyz[3] = {0.0};

    std::shared_ptr<GenericMotor> virtualMaxonMotor;
    int maxonMotorCount = 0;
     struct can_frame frame;
     

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
