#include "../include/CanService.hpp"



void CanService::enterControlMode(MotorInterface &motor)
{
    motor.fillCanFrameForControlMode(&frame);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::setToZero(MotorInterface &motor)
{
    motor.fillCanFrameForZeroing(&frame);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::checkMotor(MotorInterface &motor)
{
    motor.fillCanFrameForCheckMotor(&frame);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::Exit(MotorInterface &motor)
{
    motor.fillCanFrameForExit(&frame);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::quickStop(MotorInterface &motor)
{
    motor.fillCanFrameForQuickStop(&frame);
    cansocket.send_frame_and_receive_reply(&frame);
}


MaxonMotor* CanService::castToMaxonMotor(MotorInterface &motor) {
    MaxonMotor *maxonMotor = dynamic_cast<MaxonMotor *>(&motor);
    if (maxonMotor == nullptr) {
        throw std::runtime_error("dynamic_cast to MaxonMotor failed");
    }
    return maxonMotor;
}

void CanService::enterOperationalMode(MotorInterface &motor)
{
    MaxonMotor *maxonMotor = castToMaxonMotor(motor);
    maxonMotor->fillCanFrameForOperational(&frame);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::setTorqueOffset(MotorInterface &motor)
{
    MaxonMotor *maxonMotor = castToMaxonMotor(motor);
    maxonMotor->fillCanFrameForTorqueOffset(&frame);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::enableControl(MotorInterface &motor)
{
    MaxonMotor *maxonMotor = castToMaxonMotor(motor);
    maxonMotor->fillCanFrameForEnable(&frame);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::setTargetPosition(MotorInterface &motor, int targetPosition)
{
    MaxonMotor *maxonMotor = castToMaxonMotor(motor);
    maxonMotor->fillCanFrameForTargetPostion(&frame, targetPosition);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::syncMotor(MotorInterface &motor)
{
    MaxonMotor *maxonMotor = castToMaxonMotor(motor);
    maxonMotor->fillCanFrameForSync(&frame);
    cansocket.send_frame_and_receive_reply(&frame);
}



