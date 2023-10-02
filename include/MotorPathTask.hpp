#pragma once
#include "SharedBuffer.hpp"
#include "CommandParser.hpp"
#include "Motor.hpp"
#include <map>

class MotorPathTask
{
public:
    void operator()(SharedBuffer<can_frame> &buffer, std::map<std::string, std::shared_ptr<TMotor>> &tmotors);

private:
    TMotorCommandParser Parser;
    can_frame frame;
};
