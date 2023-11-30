#pragma once

#include <stdio.h>
#include "../include/CanSocketUtils.hpp"
#include "../include/CommandParser.hpp"
#include "../include/ErrorHandle.hpp"
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

#include "SystemState.hpp"

using namespace std;

class SendLoopTask
{
public:
    // 생성자 선언
    SendLoopTask(SystemState &systemStateRef, CanSocketUtils &canUtilsRef);

    // operator() 함수 선언
    void operator()();

private:
    SystemState &systemState;
    CanSocketUtils &canUtils;
    std::map<std::string, std::shared_ptr<TMotor>, CustomCompare> tmotors; // 모터 배열
    std::map<std::string, std::shared_ptr<MaxonMotor>> maxonMotors;        // 가정된 MaxonMotor 배열

    TMotorCommandParser TParser;
    MaxonCommandParser MParser;

    // System Initiallize
    void initializeTMotors();
    void initializeCanUtils();
    void ActivateControlTask();
    vector<string> extractIfnamesFromMotors(const map<string, shared_ptr<TMotor>, CustomCompare> &motors);
    void DeactivateControlTask();

    // SenSor
    int DeviceID = USB2051_32;
    BYTE BoardID = 0x02;
    BYTE total_di;
    int DevNum, res;
    char module_name[15];
    DWORD DIValue = 0, o_dwDICntValue[USBIO_DI_MAX_CHANNEL];

    void SensorLoopTask(queue<int> &sensorBuffer);
    void ActivateSensor();
    void DeactivateSensor();

    // Home
    void SetHome();
    void CheckCurrentPosition(std::shared_ptr<TMotor> motor);
    float MoveMotorToSensorLocation(std::shared_ptr<TMotor> &motor, const std::string &motorName, int sensorBit);
    void RotateMotor(std::shared_ptr<TMotor> &motor, const std::string &motorName, double direction, float midpoint);
    void SendCommandToMotor(std::shared_ptr<TMotor> &motor, struct can_frame &frame, const std::string &motorName);
    bool PromptUserForHoming(const std::string &motorName);

    // Tune
    void FixMotorPosition();
    void Tuning(float kp, float kd, float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType);
    void TuningLoopTask();
    void InitializeTuningParameters(const std::string selectedMotor, float &kp, float &kd, float &peakAngle, int &pathType);
    
    //Path
    vector<double> c_MotorAngle = {0, 0, 0, 0, 0, 0, 0};
    vector<vector<double>> right_inst;
    vector<vector<double>> left_inst;

    int bpm = 80;
    vector<double> time_arr;
    vector<vector<int>> RA, LA;
    vector<int> RF, LF;

    // Ready Array : 0, 90, 90, 45, 75, 45, 75
    vector<double> standby = {0, M_PI / 2, M_PI / 2, M_PI / 4, M_PI / 2.4, M_PI / 4, M_PI / 2.4};

    int end = 0;
    int line = 0;

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

    vector<vector<double>> q;

    map<string, int> motor_mapping = {
        {"L_arm1", 2}, {"L_arm2", 5}, {"L_arm3", 6}, {"R_arm1", 1}, {"R_arm2", 3}, {"R_arm3", 4}, {"waist", 0}};

    string trimWhitespace(const std::string &str);
    vector<double> connect(vector<double> &Q1, vector<double> &Q2, int k, int n);
    vector<double> IKfun(vector<double> &P1, vector<double> &P2, vector<double> &R, double s, double z0);
    void GetMusicSheet();
    void GetReadyArr(queue<can_frame> &sendBuffer);
    void PathLoopTask(queue<can_frame> &sendBuffer);
    void GetBackArr();

    //Send Loop
    template <typename MotorMap>
    void writeToSocket(MotorMap &motorMap, std::queue<can_frame> &sendBuffer, const std::map<std::string, int> &sockets);
    void SendLoop(queue<can_frame> &sendBuffer);

    int writeFailCount;
};
