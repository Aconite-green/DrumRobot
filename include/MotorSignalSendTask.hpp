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
#include <termios.h>
#include <fcntl.h>
#include "../include/SharedBuffer.hpp"
#include "../include/Motor.hpp"
#include <atomic>
class MotorSignalSendTask
{
public:
     MotorSignalSendTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, const std::map<std::string, int> &sockets,std::atomic<bool>& paused);
     void operator()(SharedBuffer<can_frame> &buffer);

private:
     std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
     const std::map<std::string, int> &sockets;
     std::atomic<bool>& paused;
     int kbhit(void);
};
