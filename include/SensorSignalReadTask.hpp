#pragma once
#include "SharedBuffer.hpp"
#include <map>
#include <memory>
#include "../include/Motor.hpp"

class SensorSignalReadTask
{
public:
    SensorSignalReadTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, std::atomic<bool> &paused);
    void operator()(SharedBuffer<int> &buffer);

private:
    std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
    std::atomic<bool> &paused;
};
