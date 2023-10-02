#include "../include/MotorPathTask.hpp"
#include "../include/SharedBuffer.hpp"
#include "../include/MotorSignalSendTask.hpp"
#include <string>

void MotorSignalSendTask::operator()(SharedBuffer<can_frame>& buffer, std::map<std::string, std::shared_ptr<TMotor>>& tmotors){
   
}
