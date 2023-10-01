#pragma once

#include <stdio.h>
#include "../include/CanService.hpp"
#include "../include/CanSocketUtils.hpp"
#include "../include/CommandParser.hpp"
#include "../include/ErrorHandle.hpp"
#include "../include/Motor.hpp"
#include "../include/MotorInterface.hpp"



class InitializeTask {
public:
    void operator()();
};

