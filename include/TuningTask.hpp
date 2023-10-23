#pragma once
#include <linux/can.h>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <queue>
#include <map>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include "../include/SharedBuffer.hpp"
#include "../include/Motor.hpp"
#include "../include/CommandParser.hpp"
#include "../include/CanSocketUtils.hpp"
#include <atomic>
class TuningTask
{
public:
     TuningTask(
         std::map<std::string, std::shared_ptr<TMotor>> &tmotors,
         std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors,
         const std::map<std::string, int> &sockets);
     void operator()();

     float kp, kd, sine_t;

private:
     std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
     std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors;
     const std::map<std::string, int> &sockets;
     TMotorCommandParser TParser;

     int set_socket_timeout(int hsocket, int timeout_sec, int timeout_usec);
};
