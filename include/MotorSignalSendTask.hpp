#pragma once
#include "SharedBuffer.hpp"

class MotorSignalSendTask {
public:
    void operator()(SharedBuffer& buffer);
};
