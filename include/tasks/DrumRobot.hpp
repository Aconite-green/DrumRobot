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
#include "../include/tasks/TaskUtility.hpp"
#include "../include/usbio/Global.hpp"
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

    // operator() 함수 선언
    void operator()();

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

    // System Initiallize
    void initializeMotors();
    void initializecanManager();
    void DeactivateControlTask();

    // Home
    void homeModeLoop();
    void displayHomingStatus();
    void UpdateHomingStatus();
    void printCurrentPositions();

    /*Tmotor*/
    void SetTmotorHome(vector<std::shared_ptr<GenericMotor>> &motor, vector<std::string> &motorName);
    void HomeTMotor(vector<std::shared_ptr<GenericMotor>> &motor, vector<std::string> &motorName);
    float MoveTMotorToSensorLocation(vector<std::shared_ptr<GenericMotor>> &motor, vector<std::string> &motorName, vector<int> &sensorBit);
    void RotateTMotor(vector<std::shared_ptr<GenericMotor>> &motor, vector<std::string> &motorName, vector<double> &direction, vector<double> &degree, vector<float> &midpoint);

    void SetTmotorHome(std::shared_ptr<GenericMotor> &motor, const std::string &motorName);
    void HomeTMotor(std::shared_ptr<GenericMotor> &motor, const std::string &motorName);
    float MoveTMotorToSensorLocation(std::shared_ptr<GenericMotor> &motor, const std::string &motorName, int sensorBit);
    void RotateTMotor(std::shared_ptr<GenericMotor> &motor, const std::string &motorName, double direction, double degree, float midpoint);
    bool PromptUserForHoming(const std::string &motorName);

    /*Maxon*/
    void SetMaxonHome(std::shared_ptr<GenericMotor> &motor, const std::string &motorName);
    void setMaxonMode(std::string targetMode);
    void motorSettingCmd();
    void MaxonEnable();
    void MaxonQuickStopEnable();
    void FixMotorPosition(std::shared_ptr<GenericMotor> &motor);

    // Perform
    void runModeLoop();
};
