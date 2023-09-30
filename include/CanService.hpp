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
    
    void enterControlMode(MotorInterface &motor);
    void setToZero(MotorInterface &motor);
    void checkMotor(MotorInterface &motor);
    void Exit(MotorInterface &motor);
    void quickStop(MotorInterface &motor);
    
    // Maxon
    void enterOperationalMode(MotorInterface &motor);
    void setTorqueOffset(MotorInterface &motor);
    void enableControl(MotorInterface &motor);
    void setTargetPosition(MotorInterface &motor, int targetPosition);
    void syncMotor(MotorInterface &motor);

private:
    struct can_frame frame;
    CanSocketUtils cansocket; 
    MaxonMotor* castToMaxonMotor(MotorInterface &motor); // CanSocketUtils 객체
};

#endif // CAN_SERVICE_H
