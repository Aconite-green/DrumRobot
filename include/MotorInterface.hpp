#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <linux/can.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdlib.h>

class MotorInterface {
public:
    int nodeId;
    std::string roboticSection;
    
    virtual void fillCanFrameForCheckMotor(struct can_frame *frame) = 0;
    virtual void fillCanFrameForControlMode(struct can_frame *frame) = 0;
    virtual void fillCanFrameForZeroing(struct can_frame *frame) = 0;
    virtual void fillCanFrameForExit(struct can_frame *frame) = 0;
    virtual void fillCanFrameForQuickStop(struct can_frame *frame) = 0;
    
};

#endif // MOTOR_INTERFACE_H
