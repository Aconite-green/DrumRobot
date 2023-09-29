#ifndef CAN_SERVICE_H
#define CAN_SERVICE_H

#include <linux/can.h>
#include <math.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <time.h>
#include <stdlib.h>
#include <linux/can.h>
#include "MotorInterface.hpp"
#include "CanSocketUtils.hpp"
#include "Motor.hpp"

class CanService
{
public:
    void enterControlMode(MotorInterface &motor, int can_id, int socket);
    void setToZero(MotorInterface &motor, int can_id, int socket);
    void checkMotor(MotorInterface &motor, int can_id, int socket);
    void Exit(MotorInterface &motor, int can_id, int socket);
    void quickStop(MotorInterface &motor, int can_id, int socket);
    
    //Maxon
    void enterOperationalMode(MotorInterface &motor, int can_id, int socket);
    void setTorqueOffset(MotorInterface &motor, int can_id, int socket);
    void enableControl(MotorInterface &motor, int can_id, int socket);
    void setTargetPosition(MotorInterface &motor, int can_id, int socket, int targetPosition);
    void syncMotor(MotorInterface &motor, int socket);

private:
    struct can_frame frame;
    CanSocketUtils cansocket;
};

#endif // CAN_SERVICE_H
