#pragma once
#include "SharedBuffer.hpp"
#include "../include/Motor.hpp"
#include "../include/CommandParser.hpp"
#include <linux/can.h> 
#include <map>
#include <memory>

class MotorResponseReadTask {
public:
    void operator()(SharedBuffer<can_frame>& buffer, std::map<std::string, std::shared_ptr<TMotor>>& tmotors);
};
