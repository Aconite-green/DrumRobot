#include "../include/InitializeTask.hpp"
#include "../include/SharedBuffer.hpp"
#include "../include/CanService.hpp"
#include <string>

void InitializeTask::operator()()
{

    TMotor tMotorWaist(1, "AK10_9", "Waist");
    CanService canService("can0");

    canService.checkMotor(tMotorWaist);
    canService.enterControlMode(tMotorWaist);

    canService.checkMotor(tMotorWaist);
    canService.Exit(tMotorWaist);
}
