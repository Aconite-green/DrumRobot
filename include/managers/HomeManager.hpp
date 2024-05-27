#pragma once

#include <stdio.h>
#include <map>
#include <memory>
#include <string>
#include <functional>
#include <queue>
#include <algorithm>
#include <thread>
#include <cerrno>
#include <cstring>
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
#include <cstdio>     // std::remove를 사용하기 위해 필요
#include <sys/stat.h> // mkdir을 사용하기 위해 필요
#include <cstdlib>


#include "../include/managers/CanManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/tasks/SystemState.hpp"
#include "../include/usbio/SenSor.hpp"
//For Qt
/*
#include "CanManager.hpp"
#include "../motors/CommandParser.hpp"
#include "../motors/Motor.hpp"
#include "../tasks/SystemState.hpp"
#include "../usbio/SenSor.hpp"
*/


using namespace std;

class HomeManager
{
public:
    HomeManager(State &stateRef,
                CanManager &canManagerRef,
                std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    void SendHomeProcess();
    void setMaxonMode(std::string targetMode);
    void MaxonEnable();
    void MaxonDisable();

    std::string m_MotorName="";
private:
    State &state;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    TMotorServoCommandParser tmotorServocmd;
    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    Sensor sensor;

    std::string motorName;
    vector<vector<string>> Priority = {
            {"L_arm1", "R_arm1"},
            {"L_arm2", "R_arm2"},
            {"L_wrist", "R_wrist", "maxonForTest"},
            {"L_arm3", "R_arm3"}};

    vector<vector<shared_ptr<GenericMotor>>> HomingMotorsArr;
    vector<shared_ptr<TMotor>> tMotors;
    vector<shared_ptr<MaxonMotor>> maxonMotors;

    vector<int> sensorsBit;
    vector<float> firstPosition, secondPosition, positionDifference;
    vector<bool> firstSensorTriggered, TriggeredDone;
    vector<double> targetRadians;
    vector<float> midpoints;
    vector<double> directions;
    bool doneSensing;

    void loadHomingInfoFromFile();
    void saveHomingInfoToFile();
    void displayHomingStatus();

    void HomeTmotor();
    void HomeMaxon();
};
