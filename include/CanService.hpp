#ifndef CAN_SERVICE_H
#define CAN_SERVICE_H

#include <linux/can.h>
#include <stdio.h>
#include <stdexcept> 
#include "MotorInterface.hpp"
#include "CanSocketUtils.hpp"
#include "Motor.hpp"

class CanService
{
public:
    CanService(const char *ifname) : cansocket(ifname) {}  // 생성자에서 CanSocketUtils 초기화
    
    void enterControlMode(MotorInterface &motor, int can_id);
    void setToZero(MotorInterface &motor, int can_id);
    void checkMotor(MotorInterface &motor, int can_id);
    void Exit(MotorInterface &motor, int can_id);
    void quickStop(MotorInterface &motor, int can_id);
    
    // Maxon
    void enterOperationalMode(MotorInterface &motor, int can_id);
    void setTorqueOffset(MotorInterface &motor, int can_id);
    void enableControl(MotorInterface &motor, int can_id);
    void setTargetPosition(MotorInterface &motor, int can_id, int targetPosition);
    void syncMotor(MotorInterface &motor);

private:
    struct can_frame frame;
    CanSocketUtils cansocket; 
    MaxonMotor* castToMaxonMotor(MotorInterface &motor); // CanSocketUtils 객체
};

#endif // CAN_SERVICE_H
