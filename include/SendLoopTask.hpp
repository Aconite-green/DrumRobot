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
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChartView>

#include "ChartHandler.hpp"
#include "SystemState.hpp"

using namespace std;
using namespace QtCharts;

class SendLoopTask
{
public:
    // 생성자 선언
    SendLoopTask(SystemState &systemStateRef, CanSocketUtils &canUtilsRef);

    // operator() 함수 선언
    void operator()();

private:
    SystemState &systemState;
    CanSocketUtils &canUtils;
    std::map<std::string, std::shared_ptr<TMotor>, CustomCompare> tmotors; // 모터 배열
    std::map<std::string, std::shared_ptr<MaxonMotor>> maxonMotors;        // 가정된 MaxonMotor 배열

    TMotorCommandParser TParser;
    MaxonCommandParser MParser;

    // System Initiallize
    void initializeTMotors();
    void initializeCanUtils();
    void ActivateControlTask();
    vector<string> extractIfnamesFromMotors(const map<string, shared_ptr<TMotor>, CustomCompare> &motors);

    // SenSor
    int DeviceID = USB2051_32;
    BYTE BoardID = 0x02;
    BYTE total_di;
    int DevNum, res;
    char module_name[15];
    DWORD DIValue = 0, o_dwDICntValue[USBIO_DI_MAX_CHANNEL];

    void SensorLoopTask(queue<int> &sensorBuffer);
    void ActivateSensor();
    void DeactivateSensor();

    // Home
    void SetHome();
    void CheckCurrentPosition(std::shared_ptr<TMotor> motor);
    void MoveMotorToSensorLocation(std::shared_ptr<TMotor> &motor, const std::string &motorName);
    void RotateMotor(std::shared_ptr<TMotor> &motor, const std::string &motorName, double direction);
    void SendCommandToMotor(std::shared_ptr<TMotor> &motor, struct can_frame &frame, const std::string &motorName);
    bool PromptUserForHoming(const std::string &motorName);

    // Tune
    void FixMotorPosition();
};
