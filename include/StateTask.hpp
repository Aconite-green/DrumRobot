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
#include "SenSor.hpp"
#include "../include/CanSocketUtils.hpp"
#include "../include/CommandParser.hpp"
#include "../include/ErrorHandle.hpp"
#include "../include/Motor.hpp"
#include "../include/TaskUtility.hpp"
#include "../include/Global.hpp"

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
              CanSocketUtils &canUtilsRef,
              std::map<std::string, std::shared_ptr<TMotor>> &tmotorsRef,
              std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotorsRef);

    // operator() 함수 선언
    void operator()();

private:
    SystemState &systemState; // 상태 참조
    CanSocketUtils &canUtils;
    std::map<std::string, std::shared_ptr<TMotor>> &tmotors; // 모터 배열
    std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors;

    TMotorCommandParser TParser;
    MaxonCommandParser MParser;
    Sensor sensor;

    // State Utility
    void displayAvailableCommands() const;
    bool processInput(const std::string &input);
    void idealStateRoutine();
    void checkUserInput();

    // System Initiallize
    void initializeMotors();
    void initializeCanUtils();
    void ActivateControlTask();
    vector<string> extractIfnamesFromMotors(const map<string, shared_ptr<TMotor>> &tmotors, const map<string, shared_ptr<MaxonMotor>> &maxonMotors);
    void DeactivateControlTask();
    bool CheckTmotorPosition(std::shared_ptr<TMotor> motor);
    bool CheckMaxonPosition(std::shared_ptr<MaxonMotor> motor);
    bool CheckAllMotorsCurrentPosition();

    // Home
    void homeModeLoop();
    void displayHomingStatus();
    void UpdateHomingStatus();
    /*Tmotor*/

    void SetHome(std::shared_ptr<TMotor> &motor, const std::string &motorName);
    void HomeTMotor(std::shared_ptr<TMotor> &motor, const std::string &motorName);
    float MoveTMotorToSensorLocation(std::shared_ptr<TMotor> &motor, const std::string &motorName, int sensorBit);
    void RotateTMotor(std::shared_ptr<TMotor> &motor, const std::string &motorName, double direction, double degree, float midpoint);
    void SendCommandToTMotor(std::shared_ptr<TMotor> &motor, struct can_frame &frame, const std::string &motorName);
    bool PromptUserForHoming(const std::string &motorName);

    /*Maxon*/
    void SetHome(std::shared_ptr<MaxonMotor> &motor, const std::string &motorName);
    void SendCommandToMaxonMotor(std::shared_ptr<MaxonMotor> &motor, struct can_frame &frame, const std::string &motorName);

    // Tune
    void FixMotorPosition();
    void TuningTmotor(float kp, float kd, float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType);
    void TuningMaxon(float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType);
    void TuningLoopTask();
    void InitializeParameters(const std::string selectedMotor, float &kp, float &kd, float &peakAngle, int &pathType);

    // Perform
    void runModeLoop();
};
