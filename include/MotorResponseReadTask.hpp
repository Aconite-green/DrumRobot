#pragma once
#include "SharedBuffer.hpp"

class MotorResponseReadTask {
public:
    void operator()(SharedBuffer& buffer);
};
