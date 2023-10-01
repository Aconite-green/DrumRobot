#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdlib.h>

#include <string>
#include "MotorInterface.hpp"

class TMotor : public MotorInterface
{
public:
   TMotor(int nodeId, const std::string &motorType, const std::string &roboticSection);

    // CanService
    void fillCanFrameForCheckMotor(struct can_frame *frame) override;
    void fillCanFrameForControlMode(struct can_frame *frame) override;
    void fillCanFrameForExit(struct can_frame *frame) override;
    void fillCanFrameForZeroing(struct can_frame *frame) override;
    void fillCanFrameForQuickStop(struct can_frame *frame) override;

private:
    float pMin, pMax;
    float vMin, vMax;
    float kpMin, kpMax;
    float kdMin, kdMax;
    float tMin, tMax;

    std::string motorType;


    void setLimits();
};

class MaxonMotor : public MotorInterface
{
public:
    int canSendId;
    int canReceiveId;

    int pdoId[4];

    

    MaxonMotor(int nodeId, const std::string &roboticSection, std::initializer_list<int> pdoIds);

    // Send all zero(SDO)
    void fillCanFrameForCheckMotor(struct can_frame *frame) override;
    // CSP mode(SDO)
    void fillCanFrameForControlMode(struct can_frame *frame) override;
    // Set pos offset(SDO)
    void fillCanFrameForZeroing(struct can_frame *frame) override;
    // Operational -> Stop(NMT)
    void fillCanFrameForExit(struct can_frame *frame) override;
    // ControlWord Shutdown(PDO)
    void fillCanFrameForQuickStop(struct can_frame *frame) override;

    // pre-operation, Stop -> oprational(MNT)
    void fillCanFrameForOperational(struct can_frame *frame);

    // Set TorqueOffset(SDO)
    void fillCanFrameForTorqueOffset(struct can_frame *frame);

    // ControlWold Enable(PDO)
    void fillCanFrameForEnable(struct can_frame *frame);

    // Set targetPostion(PDO)
    void fillCanFrameForTargetPostion(struct can_frame *frame, int targetPosition);

    // Sync(PDO)
    void fillCanFrameForSync(struct can_frame *frame);
};

#endif
