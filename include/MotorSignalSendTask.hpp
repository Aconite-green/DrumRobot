#pragma once
#include "SharedBuffer.hpp"
#include <linux/can.h> // for can_frame

class MotorSignalSendTask
{
public:
     MotorSignalSendTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, const std::map<std::string, int> &sockets);
     void operator()(SharedBuffer<can_frame> &buffer);

private:
     std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
     const std::map<std::string, int>& sockets;
};
