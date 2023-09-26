#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <linux/can.h>

class MotorInterface {
public:
    virtual void fillCanFrameForCheckMotor(struct can_frame *frame, int can_id) = 0;
    virtual void fillCanFrameForControlMode(struct can_frame *frame, int can_id) = 0;
    virtual void fillCanFrameForZeroing(struct can_frame *frame, int can_id) = 0;
    virtual void fillCanFrameForExit(struct can_frame *frame, int can_id) = 0;
    
};

#endif // MOTOR_INTERFACE_H
