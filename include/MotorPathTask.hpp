#pragma once
#include "SharedBuffer.hpp"

class MotorPathTask {
public:
    void operator()(SharedBuffer& buffer);
};
