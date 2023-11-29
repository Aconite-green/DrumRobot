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


class StateTask {
public:
    // 생성자 선언
   StateTask(SystemState& systemStateRef);

    // operator() 함수 선언
    void operator()();

private:
    SystemState& systemState; // 상태 참조

    // 추가된 private 함수들
    void displayAvailableCommands() const;
    std::string getStateName() const;
    bool processInput(const std::string& input);
};
