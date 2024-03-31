#pragma once

#include <stdio.h>
#include "../include/managers/CanManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/tasks/SystemState.hpp"
#include "../include/usbio/SenSor.hpp"
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
#include <cstdio> // std::remove를 사용하기 위해 필요
#include <sys/stat.h> // mkdir을 사용하기 위해 필요
#include <cstdlib>

using namespace std;

class HomeManager
{
public:
    HomeManager(State &stateRef,
                CanManager &canManagerRef,
                std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    void mainLoop();
    void SendHomeProcess();

private:
    State &state;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    Sensor sensor;

    std::string motorName;
    vector<vector<string>> Priority = {{"L_arm1", "R_arm1"},
                                       {"L_arm2", "R_arm2"},
                                       {"L_arm3", "R_arm3"},
                                       {"L_wrist", "R_wrist", "maxonForTest"}};

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
    void UpdateHomingStatus();

    void setMaxonMode(std::string targetMode);
    void MaxonEnable();
    void FixMotorPosition(std::shared_ptr<GenericMotor> &motor);
    void MaxonDisable();
    void HomeTmotor();
    void HomeMaxon();
};
