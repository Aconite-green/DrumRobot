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
#include "../usbio/SenSor.hpp"
#include "../managers/PathManager.hpp"

using namespace std;

class SendLoopTask
{
public:
    // 생성자 선언
    SendLoopTask(SystemState &systemStateRef,
                 CanManager &canManagerRef,
                 std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef
                 );

    // operator() 함수 선언
    void operator()();

private:
    SystemState &systemState;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors; // 가정된 MaxonMotor 배열

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    Sensor sensor;

    // Perform
    
    void SendLoop();
    void save_to_txt_inputData(const string &csv_file_name);
    void motorInitialize(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);
    
    PathManager pathManager;
    void SendReadyLoop();
    bool CheckAllMotorsCurrentPosition();
    bool CheckTmotorPosition(std::shared_ptr<TMotor> motor);
    bool CheckMaxonPosition(std::shared_ptr<MaxonMotor> motor);
    int writeFailCount;
    void initializePathManager();
    void clearBuffer();
    
};
