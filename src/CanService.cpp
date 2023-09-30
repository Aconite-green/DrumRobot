#include "../include/CanService.hpp"



void CanService::enterControlMode(MotorInterface &motor, int can_id)
{
    motor.fillCanFrameForControlMode(&frame, can_id);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::setToZero(MotorInterface &motor, int can_id)
{
    motor.fillCanFrameForZeroing(&frame, can_id);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::checkMotor(MotorInterface &motor, int can_id)
{
    motor.fillCanFrameForCheckMotor(&frame, can_id);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::Exit(MotorInterface &motor, int can_id)
{
    motor.fillCanFrameForExit(&frame, can_id);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::quickStop(MotorInterface &motor, int can_id)
{
    motor.fillCanFrameForQuickStop(&frame, can_id);
    cansocket.send_frame_and_receive_reply(&frame);
}


MaxonMotor* CanService::castToMaxonMotor(MotorInterface &motor) {
    MaxonMotor *maxonMotor = dynamic_cast<MaxonMotor *>(&motor);
    if (maxonMotor == nullptr) {
        throw std::runtime_error("dynamic_cast to MaxonMotor failed");
    }
    return maxonMotor;
}

void CanService::enterOperationalMode(MotorInterface &motor, int can_id)
{
    MaxonMotor *maxonMotor = castToMaxonMotor(motor);
    maxonMotor->fillCanFrameForOperational(&frame, can_id);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::setTorqueOffset(MotorInterface &motor, int can_id)
{
    MaxonMotor *maxonMotor = castToMaxonMotor(motor);
    maxonMotor->fillCanFrameForTorqueOffset(&frame, can_id);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::enableControl(MotorInterface &motor, int can_id)
{
    MaxonMotor *maxonMotor = castToMaxonMotor(motor);
    maxonMotor->fillCanFrameForEnable(&frame, can_id);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::setTargetPosition(MotorInterface &motor, int can_id, int targetPosition)
{
    MaxonMotor *maxonMotor = castToMaxonMotor(motor);
    maxonMotor->fillCanFrameForTargetPostion(&frame, can_id, targetPosition);
    cansocket.send_frame_and_receive_reply(&frame);
}

void CanService::syncMotor(MotorInterface &motor)
{
    MaxonMotor *maxonMotor = castToMaxonMotor(motor);
    maxonMotor->fillCanFrameForSync(&frame);
    cansocket.send_frame_and_receive_reply(&frame);
}



