#pragma once
#include "SharedBuffer.hpp"
#include <linux/can.h> // for can_frame

class MotorSignalSendTask {
public:
     void operator()(SharedBuffer<can_frame>& buffer, std::map<std::string, std::shared_ptr<TMotor>>& tmotors);
};
