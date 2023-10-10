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
#include <termios.h>
#include <fcntl.h>
class MotorResponseReadTask
{
public:
    MotorResponseReadTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors,
                          std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors,
                          const std::map<std::string, int> &sockets,
                          std::atomic<bool> &paused,
                          std::atomic<bool> &stop);
    void operator()(SharedBuffer<can_frame> &buffer);

private:
    std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
    std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors;
    const std::map<std::string, int> &sockets;
    std::atomic<bool> &paused;
    std::atomic<bool> &stop;
    std::map<std::string, int> motor_count_per_port;
    int kbhit(void); 
    
};
