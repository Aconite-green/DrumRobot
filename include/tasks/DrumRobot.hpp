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
#include "../include/managers/CanManager.hpp"
#include "../include/managers/PathManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/managers/TestManager.hpp"
#include "../include/USBIO_advantech/USBIO_advantech.hpp"


using namespace std;


class DrumRobot
{
public:
    DrumRobot(State &StateRef,
              CanManager &canManagerRef,
              PathManager &pathManagerRef,
              TestManager &testManagerRef,
              std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef,
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
    TestManager &testManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;
    USBIO &usbio;

    // Parsing
    TMotorServoCommandParser tservocmd;
    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;

    // 제어 주기
    chrono::system_clock::time_point ReadStandard;
    chrono::system_clock::time_point SendStandard;
    chrono::system_clock::time_point addStandard;

    // Sync command
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

    // 로봇 상태 플래그
    bool getHome = false; ///< 홈 위치 플래그.
    bool isHome = false;
    bool isReady = false; ///< 준비 상태 플래그.
    bool getReady = false;
    bool isBack = false; ///< 되돌아가기 플래그.
    bool getBack = false;
    bool sendCheckFrame = false;
    void flag_setting(string flag);
    bool getBackAndShutdown = false;

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
        {"maxonForTest", 8},
        {"R_foot", 9},
        {"L_foot", 10}};

    // 로봇 고정했을 때 각 모터의 관절각      Waist   Rarm1   Larm1   Rarm2   Rarm3   Larm2   Larm3   Rwrist  Lwrist  Rfoot   Lfoot   [deg]
    const float initial_joint_angles[11] = {10.0,   90.0,   90.0,   0.0,    40.0,  0.0,    40.0,   95.0,   95.0,    0.0,    0.0};

    // 로봇의 관절각 범위
    //                                 Waist   Rarm1   Larm1   Rarm2   Rarm3   Larm2   Larm3   Rwrist  Lwrist  Rfoot   Lfoot   [deg]
    const float joint_range_max[11] = {90.0,  150.0,  180.0,  90.0,   120.0,  90.0,   120.0,  135.0,  135.0,  135.0,  135.0};
    const float joint_range_min[11] = {-90.0, 0.0,    30.0,   -60.0,  -30.0,  -60.0,  -30.0,  -108.0, -108.0, -90.0,  -90.0};

    can_frame frame;
    int des = 0;
    int act = 0;
    int cnt = 0;
    int k = 0;

    // Receive Thread Loop 메소드들
    const int TIME_THRESHOLD_MS = 5;
    void ReadProcess(int periodMicroSec);
    bool dct_fun(float positions[], float vel_th);

    // Maxon 모터 Homing 함수
    void homingMaxonEnable();
    void homingSetMaxonMode(std::string targetMode);
};
