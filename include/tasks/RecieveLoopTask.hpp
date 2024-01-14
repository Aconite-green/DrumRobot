#pragma once

#include <stdio.h>
#include "../include/managers/CanManager.hpp"
#include "../include/motors/CommandParser.hpp"

#include "../include/motors/Motor.hpp"
#include "../include/tasks/TaskUtility.hpp"
#include "../include/usbio/Global.hpp"
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

using namespace std;

class RecieveLoopTask
{
public:
    // 생성자 선언
    RecieveLoopTask(SystemState &systemStateRef,
                    CanManager &canManagerRef,
                    std::map<std::string, std::shared_ptr<TMotor>> &tmotorsRef,
                    std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotorsRef,
                    queue<can_frame> &recieveBufferRef);
    // operator() 함수 선언
    void operator()();

private:
    SystemState &systemState;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
    std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors; // 가정된 MaxonMotor 배열
    queue<can_frame> &recieveBuffer;
    
    TMotorCommandParser TParser;
    MaxonCommandParser MParser;
    Sensor sensor;

    const int NUM_FRAMES = 10;
    const int TIME_THRESHOLD_MS = 5;
    int writeFailCount = 0;

    void RecieveLoop(queue<can_frame> &recieveBuffer);
    void handleSocketRead(int socket_descriptor, queue<can_frame> &recieveBuffer);
    void parse_and_save_to_csv(const std::string &csv_file_name);
    void checkMotors();
    bool checkTmotors(std::shared_ptr<TMotor> motor);
    bool checkMmotors(std::shared_ptr<MaxonMotor> motor);
};