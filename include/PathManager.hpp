#pragma once
#include "SharedBuffer.hpp"
#include "CommandParser.hpp"
#include "Motor.hpp"
#include "bin2str.hpp"
#include "RL_assign.hpp"
#include "str2bin.hpp"
#include "qd2sd.hpp"
#include "qd2sd_F.hpp"
#include "IKfun.hpp"
#include "connect.hpp"
#include <map>
#include <algorithm>

class PathManager
{
public:
    PathManager(std::map<std::string, std::shared_ptr<TMotor>> &tmotors);
    void operator()(SharedBuffer<can_frame> &buffer);

private:
    TMotorCommandParser Parser;
   

    std::map<std::string, std::shared_ptr<TMotor>> &tmotors;
};