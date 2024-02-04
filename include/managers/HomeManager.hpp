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

using namespace std;

class HomeManager
{
public:
    HomeManager(SystemState &systemStateRef,
                CanManager &canManagerRef,
                std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);

    void mainLoop();

private:
    SystemState &systemState;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    Sensor sensor;
    
    // Home
    void displayHomingStatus();
    void UpdateHomingStatus();

    /*Tmotor*/
    void SetTmotorHome(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames);
    void HomeTMotor(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames);
    vector<float> MoveTMotorToSensorLocation(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames, vector<int> &sensorsBit);
    void RotateTMotor(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames, vector<double> &directions, vector<double> &degrees, vector<float> &midpoints);
    bool PromptUserForHoming(const std::string &motorName);

    /*Maxon*/
    void SetMaxonHome(vector<std::shared_ptr<GenericMotor>> &motors);
    void setMaxonMode(std::string targetMode);
    void motorSettingCmd();
    void MaxonEnable();
    void MaxonQuickStopEnable();
    void FixMotorPosition(std::shared_ptr<GenericMotor> &motor);
};