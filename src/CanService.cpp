#include "../include/CanService.hpp"

void CanService::enterControlMode(MotorInterface &motor, int can_id, int socket)
{
    motor.fillCanFrameForControlMode(&frame, can_id);
    cansocket.send_frame_and_receive_reply(socket, &frame);
}

void CanService::setToZero(MotorInterface &motor, int can_id, int socket)
{
    motor.fillCanFrameForZeroing(&frame, can_id);
    cansocket.send_frame_and_receive_reply(socket, &frame);
}

void CanService::checkMotor(MotorInterface &motor, int can_id, int socket)
{
    motor.fillCanFrameForCheckMotor(&frame, can_id);
    cansocket.send_frame_and_receive_reply(socket, &frame);
}

void CanService::Exit(MotorInterface &motor, int can_id, int socket)
{
    motor.fillCanFrameForExit(&frame, can_id);
    cansocket.send_frame_and_receive_reply(socket, &frame);
}

void CanService::quickStop(MotorInterface &motor, int can_id, int socket)
{
    motor.fillCanFrameForQuickStop(&frame, can_id);
    cansocket.send_frame_and_receive_reply(socket, &frame);
}

void CanService::enterOperationalMode(MotorInterface &motor, int can_id, int socket)
{
    MaxonMotor *maxonMotor = dynamic_cast<MaxonMotor *>(&motor);
    if (maxonMotor != nullptr)
    {
        maxonMotor->fillCanFrameForOperational(&frame, can_id);
        cansocket.send_frame_and_receive_reply(socket, &frame);
    }
    else
    {
        printf("dynamic_cast failed");
    }
}

void CanService::setTorqueOffset(MotorInterface &motor, int can_id, int socket)
{
    MaxonMotor *maxonMotor = dynamic_cast<MaxonMotor *>(&motor);
    if (maxonMotor != nullptr)
    {
        maxonMotor->fillCanFrameForTorqueOffset(&frame, can_id);
        cansocket.send_frame_and_receive_reply(socket, &frame);
    }
    else
    {
        printf("dynamic_cast failed");
    }
}

void CanService::enableControl(MotorInterface &motor, int can_id, int socket)
{
    MaxonMotor *maxonMotor = dynamic_cast<MaxonMotor *>(&motor);
    if (maxonMotor != nullptr)
    {
        maxonMotor->fillCanFrameForEnable(&frame, can_id);
        cansocket.send_frame_and_receive_reply(socket, &frame);
    }
    else
    {
        printf("dynamic_cast failed");
    }
}

void CanService::setTargetPosition(MotorInterface &motor, int can_id, int socket, int targetPosition)
{
    MaxonMotor *maxonMotor = dynamic_cast<MaxonMotor *>(&motor);
    if (maxonMotor != nullptr)
    {
        maxonMotor->fillCanFrameForTargetPostion(&frame, can_id, targetPosition);
        cansocket.send_frame_and_receive_reply(socket, &frame);
    }
    else
    {
        printf("dynamic_cast failed");
    }
}

void CanService::syncMotor(MotorInterface &motor, int socket)
{
    MaxonMotor *maxonMotor = dynamic_cast<MaxonMotor *>(&motor);
    if (maxonMotor != nullptr)
    {
        maxonMotor->fillCanFrameForSync(&frame);
        cansocket.send_frame_and_receive_reply(socket, &frame);
    }
    else
    {
        printf("dynamic_cast failed");
    }
}
