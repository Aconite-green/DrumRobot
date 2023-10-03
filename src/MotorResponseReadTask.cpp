#include "../include/MotorPathTask.hpp"
#include "../include/SharedBuffer.hpp"
#include "../include/MotorResponseReadTask.hpp"
#include <string>


MotorResponseReadTask::MotorResponseReadTask(std::map<std::string, std::shared_ptr<TMotor>> &tmotors, const std::map<std::string, int> &sockets)
: tmotors(tmotors), sockets(sockets)
{
}

void MotorResponseReadTask::operator()(SharedBuffer<can_frame>& buffer){
   
}