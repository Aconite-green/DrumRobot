#pragma once
#include "SharedBuffer.hpp"
#include "CommandParser.hpp"
#include "Motor.hpp"
#include <map>

class MotorPathTask
{
public:
    MotorPathTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors);
    void operator()(SharedBuffer<can_frame> &buffer);

private:
    TMotorCommandParser Parser;
    can_frame frame;

    std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
};
