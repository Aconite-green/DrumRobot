#pragma once
#include "SharedBuffer.hpp"

class SensorSignalReadTask {
public:
    void operator()(SharedBuffer& buffer);
};
