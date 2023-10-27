#pragma once
#include <linux/can.h>
#include <iostream>
#include <ctime>
#include <chrono>
#include <cmath>
#include <unistd.h>
#include <queue>
#include <map>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include "../include/SharedBuffer.hpp"
#include "../include/Motor.hpp"
#include <atomic>
class MotorSignalSendTask
{
public:
     MotorSignalSendTask(
          std::map<std::string, std::shared_ptr<TMotor>> &tmotors,
          std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors, 
          const std::map<std::string, int> &sockets,
          std::atomic<bool>& paused,
          std::atomic<bool>& stop);
     void operator()(SharedBuffer<can_frame> &buffer);

private:
     std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
     std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors;
     const std::map<std::string, int> &sockets;
     std::atomic<bool>& paused;
     std::atomic<bool>& stop;
};
