#include "../include/MotorPathTask.hpp"
#include "../include/SharedBuffer.hpp"
#include "../include/MotorSignalSendTask.hpp"
#include <string>

MotorSignalSendTask::MotorSignalSendTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, const std::map<std::string, int> &sockets)
: tmotors(tmotors), sockets(sockets)
{
}

void MotorSignalSendTask::operator()(SharedBuffer<can_frame>& buffer){
   
}
