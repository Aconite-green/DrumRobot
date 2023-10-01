#pragma once
#include "SharedBuffer.hpp"
#include "CommandParser.hpp"
#include "Motor.hpp"

class MotorPathTask
{
public:
    void operator()(SharedBuffer<can_frame> &buffer);

private:
    TMotorCommandParser Parser;
    can_frame frame;
};
