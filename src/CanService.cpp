#include "../include/CanService.hpp"

void CanService::sendAndReceiveWithTimeout(struct can_frame *frame)
{
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 50000; // 50ms

    if (setsockopt(canSocketUtils.hsocket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
    {
        printf("setsockopt failed\n");
        return;
    }


    printf("Before sending - CAN Frame ID: 0x%X, Data: ", frame->can_id);
    for (int i = 0; i < frame->can_dlc; i++)
    {
        printf("%02X ", frame->data[i]);
    }
    printf("\n");

    int result = write(canSocketUtils.hsocket, frame, sizeof(struct can_frame));
    if (result <= 0)
    {
        printf("send error, errno: %d, strerror: %s\n", errno, strerror(errno));
        return;
    }

    result = read(canSocketUtils.hsocket, frame, sizeof(struct can_frame));
    if (result <= 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            printf("recv timeout\n");
        }
        else
        {
            printf("recv error, errno: %d, strerror: %s\n", errno, strerror(errno));
        }
        return;
    }

    printf("After receiving - CAN Frame ID: 0x%X, Data: ", frame->can_id);
    for (int i = 0; i < frame->can_dlc; i++)
    {
        printf("%02X ", frame->data[i]);
    }
    printf("\n");
}

void CanService::enterControlMode(MotorInterface &motor)
{
    motor.fillCanFrameForControlMode(&frame);
    sendAndReceiveWithTimeout(&frame);
    std::cout << "#" << motor.nodeId << " [" << motor.roboticSection << "] EnterControlMode" << std::endl;
}

void CanService::setToZero(MotorInterface &motor)
{
    motor.fillCanFrameForZeroing(&frame);
    sendAndReceiveWithTimeout(&frame);
    std::cout << "#" << motor.nodeId << " [" << motor.roboticSection << "] setToZero" << std::endl;
}

void CanService::checkMotor(MotorInterface &motor)
{
    motor.fillCanFrameForCheckMotor(&frame);
    sendAndReceiveWithTimeout(&frame);
    std::cout << "#" << motor.nodeId << " [" << motor.roboticSection << "] checkMotor" << std::endl;
}

void CanService::Exit(MotorInterface &motor)
{
    motor.fillCanFrameForExit(&frame);
    sendAndReceiveWithTimeout(&frame);
    std::cout << "#" << motor.nodeId << " [" << motor.roboticSection << "] Exit" << std::endl;
}

void CanService::quickStop(MotorInterface &motor)
{
    motor.fillCanFrameForQuickStop(&frame);
    sendAndReceiveWithTimeout(&frame);
    std::cout << "#" << motor.nodeId << " [" << motor.roboticSection << "] quickStop" << std::endl;
}


MaxonMotor *CanService::castToMaxonMotor(MotorInterface &motor)
{
    MaxonMotor *maxonMotor = dynamic_cast<MaxonMotor *>(&motor);
    if (maxonMotor == nullptr)
    {
        throw std::runtime_error("dynamic_cast to MaxonMotor failed");
    }
    return maxonMotor;
}

void CanService::enterOperationalMode(MotorInterface &motor)
{
    MaxonMotor *maxonMotor = castToMaxonMotor(motor);
    maxonMotor->fillCanFrameForOperational(&frame);
    sendAndReceiveWithTimeout(&frame);
    std::cout << "#" << motor.nodeId << " [" << motor.roboticSection << "] enterOperationalMode" << std::endl;
}

void CanService::setTorqueOffset(MotorInterface &motor)
{
    MaxonMotor *maxonMotor = castToMaxonMotor(motor);
    maxonMotor->fillCanFrameForTorqueOffset(&frame);
    sendAndReceiveWithTimeout(&frame);
    std::cout << "#" << motor.nodeId << " [" << motor.roboticSection << "] setTorqueOffset" << std::endl;
}

void CanService::enableControl(MotorInterface &motor)
{
    MaxonMotor *maxonMotor = castToMaxonMotor(motor);
    maxonMotor->fillCanFrameForEnable(&frame);
    sendAndReceiveWithTimeout(&frame);
    std::cout << "#" << motor.nodeId << " [" << motor.roboticSection << "] enableControl" << std::endl;
}

void CanService::setTargetPosition(MotorInterface &motor, int targetPosition)
{
    MaxonMotor *maxonMotor = castToMaxonMotor(motor);
    maxonMotor->fillCanFrameForTargetPostion(&frame, targetPosition);
    sendAndReceiveWithTimeout(&frame);
    std::cout << "#" << motor.nodeId << " [" << motor.roboticSection << "] setTargetPosition" << std::endl;
}

void CanService::syncMotor(MotorInterface &motor)
{
    MaxonMotor *maxonMotor = castToMaxonMotor(motor);
    maxonMotor->fillCanFrameForSync(&frame);
    sendAndReceiveWithTimeout(&frame);
    std::cout << "#" << motor.nodeId << " [" << motor.roboticSection << "] syncMotor" << std::endl;
}
