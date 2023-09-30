#include "../include/InitializeTask.hpp"
#include "../include/SharedBuffer.hpp"
#include <string>

void InitializeTask::operator()(SharedBuffer &buffer)
{

    TMotor tMotorWaist(1, "AK10_9", "Waist");

    CanService canService("can0");

    canService.checkMotor(tMotorWaist);
    canService.enterControlMode(tMotorWaist);
    
    canService.checkMotor(tMotorWaist);
    canService.Exit(tMotorWaist);
}
