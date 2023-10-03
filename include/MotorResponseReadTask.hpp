#pragma once
#include "SharedBuffer.hpp"
#include "../include/Motor.hpp"
#include "../include/CommandParser.hpp"
#include <linux/can.h>
#include <map>
#include <memory>

class MotorResponseReadTask
{
public:
    MotorResponseReadTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, const std::map<std::string, int> &sockets);
    void operator()(SharedBuffer<can_frame> &buffer);

private:
    std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
    const std::map<std::string, int> &sockets;
};
