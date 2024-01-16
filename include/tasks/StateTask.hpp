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
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/tasks/TaskUtility.hpp"
#include "../include/usbio/Global.hpp"
#include "../include/managers/TestManager.hpp"

// #include <QObject>

using namespace std;

class StateTask /*: public QObject*/
{
    // Q_OBJECT

    /*signals:
        void stateChanged(Main newState);*/

public:
    // 생성자 선언
    StateTask(SystemState &systemStateRef,
              CanManager &canManagerRef,
              std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef,
              queue<can_frame> &sendBufferRef,
              queue<can_frame> &recieveBufferRef);

    // operator() 함수 선언
    void operator()();

private:
    SystemState &systemState; // 상태 참조
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;
    queue<can_frame> &sendBuffer;
    queue<can_frame> &recieveBuffer;

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    Sensor sensor;
    TestManager testmanager;

    // State Utility
    void displayAvailableCommands() const;
    bool processInput(const std::string &input);
    void idealStateRoutine();
    void checkUserInput();

    // System Initiallize
    void initializeMotors();
    void initializecanManager();
    vector<string> extractIfnamesFromMotors(const map<string, shared_ptr<GenericMotor>> &motors);
    void DeactivateControlTask();
    bool CheckAllMotorsCurrentPosition();
    void printCurrentPositions();
    bool checkMotorPosition(std::shared_ptr<GenericMotor> motor);
    
    // Home
    void homeModeLoop();
    void displayHomingStatus();
    void UpdateHomingStatus();
    void MaxonHMMSetting();

    /*Tmotor*/
    void SetTmotorHome(std::shared_ptr<GenericMotor> &motor, const std::string &motorName);
    void HomeTMotor(std::shared_ptr<GenericMotor> &motor, const std::string &motorName);
    float MoveTMotorToSensorLocation(std::shared_ptr<GenericMotor> &motor, const std::string &motorName, int sensorBit);
    void RotateTMotor(std::shared_ptr<GenericMotor> &motor, const std::string &motorName, double direction, double degree, float midpoint);
    bool PromptUserForHoming(const std::string &motorName);
    

    /*Maxon*/
    void SetMaxonHome(std::shared_ptr<GenericMotor> &motor, const std::string &motorName);
    
    // Tune
    void FixMotorPosition();
    void TuningTmotor(float kp, float kd, float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType);
    void TuningLoopTask();
    void InitializeParameters(const std::string selectedMotor, float &kp, float &kd, float &peakAngle, int &pathType, int &controlType, int &des_vel, int &des_tff, int &direction);
    void TuningMaxonCSP(float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType);
    void TuningMaxonCSV(const std::string selectedMotor, int des_vel, int direction);
    void TuningMaxonCST(const std::string selectedMotor, int des_tff, int direction);
    void MaxonDrumTest(float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType, int des_vel, int direction);
    void MaxonCSPSetting();
    void MaxonCSVSetting();
    void MaxonCSTSetting();
    void MaxonEnable();
    void MaxonQuickStopEnable();

    // Perform
    void runModeLoop();
};
