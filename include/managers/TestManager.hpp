#pragma once

#include <stdio.h>
#include "../include/managers/CanManager.hpp"
#include "../include/managers/PathManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/tasks/SystemState.hpp"
#include "../include/USBIO_advantech/USBIO_advantech.hpp"
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
    TestManager(State &stateRef, CanManager &canManagerRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef, USBIO &usbioRef);

    void SendTestProcess();
    void MaxonEnable();
    void setMaxonMode(std::string targetMode);

    bool isMaxonEnable = false;

    vector<vector<float>> Input_pos;

private:

    State &state;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    USBIO &usbio;

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

    map<std::string, int> motor_mapping = { //< 각 관절에 해당하는 열 정보.
        {"waist", 0},
        {"R_arm1", 1},
        {"L_arm1", 2},
        {"R_arm2", 3},
        {"R_arm3", 4},
        {"L_arm2", 5},
        {"L_arm3", 6},
        {"R_wrist", 7},
        {"L_wrist", 8},
        {"maxonForTest", 8},
        {"R_foot", 9},
        {"L_foot", 10}};

    //                            Waist   Rarm1   Larm1   Rarm2   Rarm3   Larm2   Larm3   Rwrist  Lwrist  Rfoot   Lfoot   [deg]
    const float motorMaxArr[11] = {90.0,  150.0,  180.0,  90.0,   144.0,  90.0,   144.0,  135.0,  135.0,  135.0,  135.0};
    const float motorMinArr[11] = {-90.0, 0.0,    30.0,   -60.0,  -30.0,  -60.0,  -30.0,  -108.0, -108.0, -90.0,  -90.0};

    std::shared_ptr<MaxonMotor> virtualMaxonMotor;
    int maxonMotorCount = 0;
    struct can_frame frame;

    /*Value Test Code*/
    void getMotorPos(float c_MotorAngle[]);
    vector<float> connect(float Q1[], float Q2[], int k, int n);
    vector<float> makeProfile(float Q1[], float Q2[], vector<float> &Vmax, float acc, float t, float t2);
    vector<float> cal_Vmax(float q1[], float q2[],  float acc, float t2);
    vector<float> sinProfile(float q1[], float q2[], float t, float t2);
    vector<float> ikfun_final(float pR[], float pL[], float part_length[], float s, float z0);
    void fkfun(float arr[]);
    void GetArr(float arr[]);
    void GetArr_test(float arr[]);


    bool sin_flag = false;
    bool brake_flag[7] = {false, false, false, false, false, false, false};
    bool single_brake_flag[7] = {false, false, false, false, false, false, false};
    float brake_start_time[7] = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
    float brake_end_time[7] = {4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0};
    float extra_time = 1.0;
    int n_repeat = 1;


    vector<float> test_mode1(float Q1[], float Q2[], float k, float n);
    vector<float> test_mode2(float Q1[], float Q2[], float k, float n);
    vector<float> test_mode3(float Q1[], float Q2[], float k, float n);
    int getArr_test_mode = 0;
   
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
    void startTest(string selectedMotor, float t, int cycles, float amp);
    void save_to_txt_inputData(const string &csv_file_name);

    /*Multi Test Code*/
    void mkArr(vector<string> &motorName, int time, int cycles, int LnR, float amp);
    void SendLoop();
    void parse_and_save_to_csv(const std::string &csv_file_name);
    void TestArr(float t, int cycles, int type, int LnR, float amp[]);

    /*Servo Mode Test Code*/
    string selectedMotor_servo = "R_arm2";
    float time_servo = 1;
    float targetpos_des = 1.57;   // Radian
    float targetpos_coo;
    float vel = 500;
    float acl = 3000;
    float t_servo = 4.0;
    int cycles_servo = 1;
    float amp_servo = 0.2; // [Radian]
    float current_servo;
    
    void startTest_servo(const string selectedMotor_servo, float pos, float vel, float acl);

    void UnfixedMotor();

    void testUSBIO_4761();
};
