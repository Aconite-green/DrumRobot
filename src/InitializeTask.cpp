#include "../include/InitializeTask.hpp"
#include "../include/SharedBuffer.hpp"
#include <string>

void InitializeTask::operator()(SharedBuffer &buffer)
{

    TMotor tMotorWaist(1, "AK10_9", "Waist");

    CanService canService("can0");

    canService.checkMotor(tMotorWaist, tMotorWaist.id);
    canService.enterControlMode(tMotorWaist, tMotorWaist.id);
}
