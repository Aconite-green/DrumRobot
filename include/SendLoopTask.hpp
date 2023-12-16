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
#include "PathManager.hpp"

using namespace std;

class SendLoopTask
{
public:
    // 생성자 선언
    SendLoopTask(SystemState &systemStateRef,
                 CanSocketUtils &canUtilsRef,
                 std::map<std::string, std::shared_ptr<TMotor>> &tmotorsRef,
                 std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotorsRef,
                 queue<can_frame> &sendBufferRef);

    // operator() 함수 선언
    void operator()();

private:
    SystemState &systemState;
    CanSocketUtils &canUtils;
    std::map<std::string, std::shared_ptr<TMotor>> &tmotors;         // 모터 배열
    std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors; // 가정된 MaxonMotor 배열
    queue<can_frame> &sendBuffer;

    TMotorCommandParser TParser;
    MaxonCommandParser MParser;
    Sensor sensor;

    // Perform
    template <typename MotorMap>
    void writeToSocket(MotorMap &motorMap, const std::map<std::string, int> &sockets);
    void SendLoop();

    PathManager pathManager;
    void SendReadyLoop();
    bool CheckAllMotorsCurrentPosition();
    bool CheckCurrentPosition(std::shared_ptr<TMotor> motor);
    int writeFailCount;
    void initializePathManager();

};
