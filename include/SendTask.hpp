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
#include "State.hpp"

using namespace std;
using namespace QtCharts;

class SendTask
{
public:
    // 생성자 선언
    SendTask(std::atomic<State> &stateRef);

    // operator() 함수 선언
    void operator()();

private:
    std::atomic<State> &state;
    std::map<std::string, std::shared_ptr<TMotor>, CustomCompare> tmotors;         // 모터 배열
    std::map<std::string, std::shared_ptr<MaxonMotor>> maxonMotors; // 가정된 MaxonMotor 배열

    TMotorCommandParser TParser;
    MaxonCommandParser MParser;
    CanSocketUtils canUtils;


    //System Initiallize
    void initializeTMotors();
    void initializeCanUtils();
    void ActivateControlTask();
    vector<string> extractIfnamesFromMotors(const map<string, shared_ptr<TMotor>, CustomCompare> &motors);

};
