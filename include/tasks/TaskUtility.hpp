#ifndef TASKUTILITY_HPP
#define TASKUTILITY_HPP

#include <stdio.h>
#include "../include/tasks/CanSocketUtils.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
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
#include <fstream>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <chrono>


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for Activate / Deactivate Task
/////////////////////////////////////////////////////////////////////////////////////////////////
void fillCanFrameFromInfo(struct can_frame *frame, const CanFrameInfo &info);
int set_socket_timeout(int hsocket, int timeout_sec, int timeout_usec);
void sendAndReceive(
    int socket,
    const std::string &name,
    struct can_frame &frame,
    std::function<void(const std::string &, bool)> customOutput);
void sendNotRead(
    int socket,
    const std::string &name,
    struct can_frame &frame,
    std::function<void(const std::string &, bool)> customOutput);
void writeAndReadForSync(
    int socket,
    const std::string &name,
    struct can_frame &frame,
    size_t numMaxonMotors,
    std::function<void(const std::string &, bool)> customOutput);

//////////////////////////////////////////////////////////////////////////////////////////
// Functions for PathGenerating
/////////////////////////////////////////////////////////////////////////////////////////

struct CustomCompare
{
    // 연산자 오버로딩을 통해 비교 함수를 정의합니다. 이 함수는 두 문자열을 비교합니다.
    bool operator()(const std::string &lhs, const std::string &rhs) const
    {
        // 우선순위를 지정하는 정적(unordered_map) 맵. 문자열 키에 따라 숫자 우선순위가 지정됩니다.
        static std::unordered_map<std::string, int> priority = {
            {"L_arm1", 0},
            {"L_arm2", 1},
            {"L_arm3", 2},
            {"R_arm1", 3},
            {"R_arm2", 4},
            {"R_arm3", 5},
            {"waist", 6},
        };

        // lhs(왼쪽 항목)의 우선순위를 찾습니다.
        auto lhsPriority = priority.find(lhs);
        // rhs(오른쪽 항목)의 우선순위를 찾습니다.
        auto rhsPriority = priority.find(rhs);

        // 두 항목 모두 우선순위 맵에 있을 경우
        if (lhsPriority != priority.end() && rhsPriority != priority.end())
        {
            // 우선순위를 비교하여 결과를 반환합니다.
            return lhsPriority->second < rhsPriority->second;
        }
        // 왼쪽 항목만 우선순위 맵에 있을 경우
        if (lhsPriority != priority.end())
        {
            // 왼쪽 항목이 우선순위가 더 높다고 간주합니다.
            return true;
        }
        // 오른쪽 항목만 우선순위 맵에 있을 경우
        if (rhsPriority != priority.end())
        {
            // 오른쪽 항목이 우선순위가 더 높다고 간주합니다.
            return false;
        }
        // 두 항목 모두 우선순위 맵에 없을 경우, 문자열 자체를 비교합니다.
        return lhs < rhs;
    }
};

//////////////////////////////////////////////////////////////////////////////////////////
// Functions for SendLoopTask
/////////////////////////////////////////////////////////////////////////////////////////

void handleError(ssize_t bytesWritten, const std::string &interface_name);

//////////////////////////////////////////////////////////////////////////////////////////
// Functions for RecieveLoopTask
/////////////////////////////////////////////////////////////////////////////////////////

int kbhit();

//////////////////////////////////////////////////////////////////////////////////////////
// Functions for Qt
/////////////////////////////////////////////////////////////////////////////////////////
;

#endif // TASKUTILITY_HPP