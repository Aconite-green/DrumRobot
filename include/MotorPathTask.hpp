#pragma once
#include "SharedBuffer.hpp"
#include "CommandParser.hpp"
#include "Motor.hpp"
#include <map>
#include <algorithm>
#include <fstream>

class MotorPathTask
{
public:
    MotorPathTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors);
    void operator()(SharedBuffer<can_frame> &buffer);

private:
    TMotorCommandParser Parser;
   

    std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
};
