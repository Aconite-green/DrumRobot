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
    TestManager(SystemState &systemStateRef,
                CanManager &canManagerRef,
                std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    void mainLoop();
    void multiTestLoop();
    void TestArr(double t, int cycles, int type, int LnR, double amp[]);

private:
    SystemState &systemState;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;

    vector<string> InputData;

    // Multi Test
    void move();
    void mkArr(vector<string> &motorName, int time, int cycles, int LnR, double amp);
    void SendLoop();
    void parse_and_save_to_csv(const std::string &csv_file_name);

    // Single Mode Test
    void FixMotorPosition(std::shared_ptr<GenericMotor> &motor);
    void TuningTmotor(float kp, float kd, float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType);
    void TuningLoopTask();
    void InitializeParameters(const std::string selectedMotor, float &kp, float &kd, float &peakAngle, int &pathType, int &controlType, int &des_vel, int &des_tff, int &direction);
    void TuningMaxonCSP(float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType);
    void TuningMaxonCSV(const std::string selectedMotor, int des_vel, int direction);
    void TuningMaxonCST(const std::string selectedMotor, int des_tff, int direction);
    void setMaxonMode(std::string targetMode);
    int kbhit();

    // Stick  Mode Test
    void TestStickLoop();
    void TestStick(const std::string selectedMotor, int des_tff, int direction, float tffThreshold, float posThreshold, int backTorqueUnit);
    bool dct_fun(float positions[], float vel_th);
};