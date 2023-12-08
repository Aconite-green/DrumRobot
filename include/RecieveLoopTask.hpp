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
#include "SenSor.hpp"

using namespace std;

class RecieveLoopTask
{
public:
    // 생성자 선언
    RecieveLoopTask(SystemState &systemStateRef, CanSocketUtils &canUtilsRef);

    // operator() 함수 선언
    void operator()();

private:
    SystemState &systemState;
    CanSocketUtils &canUtils;
    std::map<std::string, std::shared_ptr<TMotor>, CustomCompare> tmotors; // 모터 배열
    std::map<std::string, std::shared_ptr<MaxonMotor>> maxonMotors;        // 가정된 MaxonMotor 배열

    queue<can_frame> recieveBuffer;
    TMotorCommandParser TParser;
    MaxonCommandParser MParser;
    Sensor sensor;

    const int NUM_FRAMES = 20;
    const int TIME_THRESHOLD_MS = 5;
    int writeFailCount = 0;

    void checkUserInput();
    void RecieveLoop(queue<can_frame> &recieveBuffer);
    void handleSocketRead(int socket_descriptor, queue<can_frame> &recieveBuffer);
    void parse_and_save_to_csv(const std::string &csv_file_name);
};