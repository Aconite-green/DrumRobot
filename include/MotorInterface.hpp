#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <linux/can.h>

class MotorInterface {
public:
    virtual void fillCanFrameForCheckMotor(struct can_frame *frame, int canId) = 0;
    virtual void fillCanFrameForControlMode(struct can_frame *frame, int canId) = 0;
    virtual void fillCanFrameForZeroing(struct can_frame *frame, int canId) = 0;
    virtual void fillCanFrameForExit(struct can_frame *frame, int canId) = 0;
    virtual void fillCanFrameForQuickStop(struct can_frame *frame, int canId) = 0;
    
};

#endif // MOTOR_INTERFACE_H
