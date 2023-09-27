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
#include "CanUtils.hpp"

class CanService
{
public:
    void enterControlMode(MotorInterface &motor, int can_id, int socket)
    {
        struct can_frame frame;
        motor.fillCanFrameForControlMode(&frame, can_id);
        send_frame_and_receive_reply(socket, &frame);
    }

    void setToZero(MotorInterface &motor, int can_id, int socket)
    {
        struct can_frame frame;
        motor.fillCanFrameForZeroing(&frame, can_id);
        send_frame_and_receive_reply(socket, &frame);
    }

    void checkMotor(MotorInterface &motor, int can_id, int socket)
    {
        struct can_frame frame;
        motor.fillCanFrameForCheckMotor(&frame, can_id);
        send_frame_and_receive_reply(socket, &frame);
    }
    void Exit(MotorInterface &motor, int can_id, int socket)
    {
        struct can_frame frame;
        motor.fillCanFrameForExit(&frame, can_id);
        send_frame_and_receive_reply(socket, &frame);
    }
    void quickStop(MotorInterface &motor, int can_id, int socket)
    {
        struct can_frame frame;
        motor.fillCanFrameForQuickStop(&frame, can_id);
        send_frame_and_receive_reply(socket, &frame);
    }

    void enterOperationalMode(MotorInterface &motor, int can_id, int socket)
    {
        struct can_frame frame;
        MaxonMotor *maxonMotor = dynamic_cast<MaxonMotor *>(&motor);
        if (maxonMotor != nullptr)
        {
            maxonMotor->fillCanFrameForOperational(&frame, can_id);
            send_frame_and_receive_reply(socket, &frame);
        }
        else
        {
            printf("dynamic_cast failed");
        }
    }

    void setTorqueOffset(MotorInterface &motor, int can_id, int socket)
    {
        struct can_frame frame;
        MaxonMotor *maxonMotor = dynamic_cast<MaxonMotor *>(&motor);
        if (maxonMotor != nullptr)
        {
            maxonMotor->fillCanFrameForTorqueOffset(&frame, can_id);
            send_frame_and_receive_reply(socket, &frame);
        }
        else
        {
            printf("dynamic_cast failed");
        }
    }

    void enableControl(MotorInterface &motor, int can_id, int socket)
    {
        struct can_frame frame;
        MaxonMotor *maxonMotor = dynamic_cast<MaxonMotor *>(&motor);
        if (maxonMotor != nullptr)
        {
            maxonMotor->fillCanFrameForEnable(&frame, can_id);
            send_frame_and_receive_reply(socket, &frame);
        }
        else
        {
            printf("dynamic_cast failed");
        }
    }

    void setTargetPosition(MotorInterface &motor, int can_id, int socket, int targetPosition)
    {
        struct can_frame frame;
        MaxonMotor *maxonMotor = dynamic_cast<MaxonMotor *>(&motor);
        if (maxonMotor != nullptr)
        {
            maxonMotor->fillCanFrameForTargetPostion(&frame, can_id, targetPosition);
            send_frame_and_receive_reply(socket, &frame);
        }
        else
        {
            printf("dynamic_cast failed");
        }
    }

    void syncMotor(MotorInterface &motor, int socket)
    {
        struct can_frame frame;
        MaxonMotor *maxonMotor = dynamic_cast<MaxonMotor *>(&motor);
        if (maxonMotor != nullptr)
        {
            maxonMotor->fillCanFrameForSync(&frame);
            send_frame_and_receive_reply(socket, &frame);
        }
        else
        {
            printf("dynamic_cast failed");
        }
    }

    
};

#endif // CAN_SERVICE_H
