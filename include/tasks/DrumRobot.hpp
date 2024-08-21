#pragma once

#include <stdio.h>
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
#include "../include/usbio/SenSor.hpp"
#include "../include/managers/CanManager.hpp"
#include "../include/managers/PathManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/managers/TestManager.hpp"
#include "../include/managers/HomeManager.hpp"
#include "../include/USBIO_advantech/USBIO_advantech.hpp"


using namespace std;


class DrumRobot
{
public:
    DrumRobot(State &StateRef,
              CanManager &canManagerRef,
              PathManager &pathManagerRef,
              HomeManager &homeManagerRef,
              TestManager &testManagerRef,
              std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef,
              Sensor &sensorRef,
              USBIO &usbioRef);

    void stateMachine();
    void sendLoopForThread();
    void recvLoopForThread();

    // Qt Input
    std::string m_Input;

private:
    State &state;
    CanManager &canManager;
    PathManager &pathManager;
    HomeManager &homeManager;
    TestManager &testManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;
    Sensor &sensor;
    USBIO &usbio;

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    TMotorServoCommandParser tservocmd;

    // 제어 주기
    chrono::system_clock::time_point ReadStandard;
    chrono::system_clock::time_point SendStandard;
    chrono::system_clock::time_point addStandard;
    std::shared_ptr<MaxonMotor> virtualMaxonMotor;

    // 쓰레드 루프 주기
    std::chrono::_V2::steady_clock::time_point send_time_point;
    std::chrono::_V2::steady_clock::time_point recv_time_point;
    std::chrono::_V2::steady_clock::time_point state_time_point;

    // State Utility 메소드들
    void displayAvailableCommands() const;
    bool processInput(const std::string &input);
    void idealStateRoutine();
    void checkUserInput();
    int kbhit();

    bool isReady = false; ///< 준비 상태 플래그.
    bool getReady = false;
    bool isBack = false; ///< 되돌아가기 플래그.
    bool getBack = false;
    bool sendCheckFrame = false;

    // System Initialize 메소드들
    void initializeMotors();
    void initializecanManager();
    void DeactivateControlTask();
    void ClearBufferforRecord();
    void printCurrentPositions();
    void motorSettingCmd();
    void setMaxonMode(std::string targetMode);

    // Send Thread Loop 메소드들
    void save_to_txt_inputData(const string &csv_file_name);
    int writeFailCount;
    int maxonMotorCount = 0;
    void initializePathManager();
    void clearMotorsSendBuffer();
    void SendPerformProcess(int periodMicroSec);
    void SendAddStanceProcess(int periodMicroSec);
    void UnfixedMotor();
    void clearMotorsCommandBuffer();

    vector<vector<float>> Input_pos;
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

    can_frame frame;
    int des = 0;
    int act = 0;
    int cnt = 0;
    int k = 0;

    // Receive Thread Loop 메소드들
    const int TIME_THRESHOLD_MS = 5;
    void parse_and_save_to_csv(const std::string &csv_file_name);
    void ReadProcess(int periodMicroSec);
    bool dct_fun(float positions[], float vel_th);

};
