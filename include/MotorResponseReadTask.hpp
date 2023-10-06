#pragma once
#include "SharedBuffer.hpp"
#include "../include/Motor.hpp"
#include "../include/CommandParser.hpp"
#include <linux/can.h>
#include <map>
#include <memory>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <queue>
#include <sys/types.h>
#include <sys/socket.h>
#include <atomic>
class MotorResponseReadTask
{
public:
    MotorResponseReadTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, const std::map<std::string, int> &sockets, std::atomic<bool>& paused);
    void operator()(SharedBuffer<can_frame> &buffer);

private:
    std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
    const std::map<std::string, int> &sockets;
    std::atomic<bool>& paused;
    std::map<std::string, int> motor_count_per_port;  // 추가: 포트당 모터 개수를 저장할 변수
};
