#include "../include/PathManager.hpp"

PathManager::PathManager(std::map<std::string, std::shared_ptr<TMotor>> &tmotors)
    : tmotors(tmotors)
{
}

void PathManager::operator()(SharedBuffer<can_frame> &buffer)
{

    /*
        struct can_frame frame;

        for (auto &entry : tmotors)
        {
            const std::string &motor_name = entry.first;
            std::shared_ptr<TMotor> &motor = entry.second;
        }


        Parser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, 0, 8, 1, 0);

        buffer.push(frame);
    */
}