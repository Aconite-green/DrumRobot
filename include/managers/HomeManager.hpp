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

    vector<int> sensorsBit;
    vector<float> firstPosition, secondPosition, positionDifference;
    vector<bool> firstSensorTriggered, TriggeredDone;
    vector<double> targetRadians;
    vector<float> midpoints;
    vector<double> directions, degrees;
    bool doneSensing;

    void displayHomingStatus();
    void UpdateHomingStatus();

    /* Tmotor */
    void SetTmotorHome(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames);
    void HomeTMotor(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames);
    vector<float> MoveTMotorToSensorLocation(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames, vector<int> &sensorsBit);
    void RotateTMotor(vector<std::shared_ptr<GenericMotor>> &motors, vector<std::string> &motorNames, vector<double> &directions, vector<double> &degrees, vector<float> &midpoints);
    bool PromptUserForHoming(const std::string &motorName);

    /* Maxon */

    void SetMaxonHome(vector<std::shared_ptr<GenericMotor>> &motors);
    void setMaxonMode(std::string targetMode);
    void MaxonEnable();
    void FixMotorPosition(std::shared_ptr<GenericMotor> &motor);
    void MaxonDisable();
};
