#pragma once
#include "SharedBuffer.hpp"
#include <map>
#include <memory>
#include "../include/Motor.hpp"

class SensorSignalReadTask {
public:
    void operator()(SharedBuffer<int>& buffer, std::map<std::string, std::shared_ptr<TMotor>>& tmotors);
};
