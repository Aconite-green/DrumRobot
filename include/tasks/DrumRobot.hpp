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
// #include "../include/tasks/TaskUtility.hpp"
#include "../include/managers/TestManager.hpp"
#include "../include/managers/HomeManager.hpp"

// #include <QObject>

using namespace std;

class DrumRobot /*: public QObject*/
{
    // Q_OBJECT

    /*signals:
        void stateChanged(Main newState);*/

public:
    // 생성자 선언
    DrumRobot(SystemState &systemStateRef,
              CanManager &canManagerRef,
              PathManager &pathManagerRef,
              HomeManager &homeManagerRef,
              TestManager &testManagerRef,
              std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    void stateMachine();
    void sendLoopForThread();
    void recvLoopForThread();

private:
    SystemState &systemState; // 상태 참조
    CanManager &canManager;
    PathManager &pathManager;
    HomeManager &homeManager;
    TestManager &testManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    Sensor sensor;

    // State Utility
    void displayAvailableCommands() const;
    bool processInput(const std::string &input);
    void idealStateRoutine();
    void checkUserInput();
    void printCurrentPositions();
    int kbhit();

    bool isReady = false;
    bool isBack = false;

    // System Initiallize
    void initializeMotors();
    void initializecanManager();
    void DeactivateControlTask();
    void motorSettingCmd();
    void setMaxonMode(std::string targetMode);
    void MaxonEnable();
    void MaxonDisable();

    // Send Thread Loop
    void SendLoop();
    void save_to_txt_inputData(const string &csv_file_name);
    void SendReadyLoop();
    int writeFailCount = 0;
    void initializePathManager();
    void clearMotorsSendBuffer();

    // Recive Thread Loop
    const int NUM_FRAMES = 10;
    const int TIME_THRESHOLD_MS = 5;

    void RecieveLoop();
    void parse_and_save_to_csv(const std::string &csv_file_name);
};
