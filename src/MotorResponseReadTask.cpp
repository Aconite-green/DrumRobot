#include "../include/MotorPathTask.hpp"
#include "../include/SharedBuffer.hpp"
#include "../include/MotorResponseReadTask.hpp"
#include <string>

void MotorResponseReadTask::operator()(SharedBuffer<can_frame>& buffer, std::map<std::string, std::shared_ptr<TMotor>>& tmotors) {
   //Read
}
