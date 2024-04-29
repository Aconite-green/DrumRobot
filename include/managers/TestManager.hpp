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
    void MaxonEnable();
    void setMaxonMode(std::string targetMode);

    bool isMaxonEnable = false;

    vector<vector<float>> Input_pos;

private:
    State &state;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    TMotorServoCommandParser tservocmd;

    vector<string> InputData;
    bool error = false;

    /*For SendTestProcess*/
    int method = 0;
    float q[9] = {0.0};
    float R_xyz[3] = {0.0};
    float L_xyz[3] = {0.0};
    float part_length[6] = {0.313, 0.3335, 0.313, 0.3335, 0.367, 0.367};
    float s = 0.600;  ///< 허리 길이.
    float z0 = 1.026; ///< 바닥부터 허리까지의 높이.
    int cnt = 0;

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

    std::shared_ptr<MaxonMotor> virtualMaxonMotor;
    int maxonMotorCount = 0;
    struct can_frame frame;

    /*Value Test Code*/
    void getMotorPos(float c_MotorAngle[]);
    vector<float> connect(float Q1[], float Q2[], int k, int n);
    vector<float> ikfun_final(float pR[], float pL[], float part_length[], float s, float z0);
    void fkfun(float arr[]);
    void GetArr(float arr[]);
   
    /* Single Test Code */
    string selectedMotor = "waist";
    float t = 4.0;
    int cycles = 1;
    float amp = 0.2; // [Radian]
    float kp = 20;
    float kd = 3.0;
    float Kp_for_Fixed = 300;
    float Kd_for_Fixed = 3;
    void singleTestLoop();
    void startTest(string selectedMotor, float t, int cycles, float amp, float kp, float kd);
    void save_to_txt_inputData(const string &csv_file_name);

    /*Multi Test Code*/
    void mkArr(vector<string> &motorName, int time, int cycles, int LnR, float amp);
    void SendLoop();
    void parse_and_save_to_csv(const std::string &csv_file_name);
    void multiTestLoop();
    void TestArr(float t, int cycles, int type, int LnR, float amp[]);

    /*Stick Test Code*/
    void TestStickLoop();
    void TestStick(const std::string selectedMotor, int des_tff, float tffThreshold, float posThreshold, int backTorqueUnit);
    bool dct_fun(float positions[], float vel_th);
    int drumHitDuration, drumReachedDuration;

    /*Servo Mode Test Code*/
    string selectedMotor_servo = "waist";
    float pos = 30;
    float vel = 50;
    float acl = 50;
    float t_servo = 4.0;
    int cycles_servo = 1;
    float amp_servo = 0.2; // [Radian]
    float targetpos_servo, current_servo;
    
    void startTest_servo(string selectedMotor_servo, float pos, float vel, float acl);

};
