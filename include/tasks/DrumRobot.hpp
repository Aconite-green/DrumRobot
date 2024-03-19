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

using namespace std;

class DrumRobot
{
public:
    DrumRobot(State &StateRef,
              CanManager &canManagerRef,
              PathManager &pathManagerRef,
              HomeManager &homeManagerRef,
              TestManager &testManagerRef,
              std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    void stateMachine();
    void sendLoopForThread();
    void recvLoopForThread();

private:
    State &state;
    CanManager &canManager;
    PathManager &pathManager;
    HomeManager &homeManager;
    TestManager &testManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    Sensor sensor;

    chrono::system_clock::time_point ReadStandard;
    chrono::system_clock::time_point SendStandard;
    std::shared_ptr<GenericMotor> virtualMaxonMotor;

    // State Utility 메소드들
    void displayAvailableCommands() const;
    bool processInput(const std::string &input);
    void idealStateRoutine();
    void checkUserInput();
    void printCurrentPositions();
    int kbhit();

    bool isReady = false; ///< 준비 상태 플래그.
    bool isBack = false; ///< 되돌아가기 플래그.

    // System Initialize 메소드들
    void initializeMotors();
    void initializecanManager();
    void DeactivateControlTask();
    void motorSettingCmd();
    void setMaxonMode(std::string targetMode);
    void MaxonEnable();
    void MaxonDisable();

    // Send Thread Loop 메소드들
    void SendLoop();
    void save_to_txt_inputData(const string &csv_file_name);
    void SendReadyLoop();
    int writeFailCount;
    int maxonMotorCount = 0;
    void initializePathManager();
    void clearMotorsSendBuffer();
    void SendPerformProcess(int periodMicroSec);
    void SendHomeProcess();
    void SendAddStanceProcess(int periodMicroSec);

    can_frame frame;

    // Receive Thread Loop 메소드들
    const int TIME_THRESHOLD_MS = 5;
    void parse_and_save_to_csv(const std::string &csv_file_name);
    void ReadProcess(int periodMicroSec);
};
