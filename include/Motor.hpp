#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdlib.h>

#include <string>
#include "MotorInterface.hpp"
#include "CommandParser.hpp"
const int MaxMotors = 10;

class TMotor : public MotorInterface
{
public:
    float pMin, pMax;
    float vMin, vMax;
    float kpMin, kpMax;
    float kdMin, kdMax;
    float tMin, tMax;

    std::string roboticSection;
    std::string motorType;
    int id;

    TMotor(int id, const std::string &motorType, const std::string &roboticSection);
    void setLimits();

    // CanService
    void fillCanFrameForCheckMotor(struct can_frame *frame, int canId) override;
    void fillCanFrameForControlMode(struct can_frame *frame, int canId) override;
    void fillCanFrameForExit(struct can_frame *frame, int canId) override;
    void fillCanFrameForZeroing(struct can_frame *frame, int canId) override;
    void fillCanFrameForQuickStop(struct can_frame *frame, int canId) override;
};

class MaxonMotor : public MotorInterface
{
public:
    int nodeId;
    int canSendId;
    int canReceiveId;

    int pdoId[4];

    std::string roboticSection;

    MaxonMotor(int nodeId, const std::string &roboticSection, std::initializer_list<int> pdoIds);


    // Send all zero(SDO)
    void fillCanFrameForCheckMotor(struct can_frame *frame, int canId) override;
    // CSP mode(SDO)
    void fillCanFrameForControlMode(struct can_frame *frame, int canId) override;
    // Set pos offset(SDO)
    void fillCanFrameForZeroing(struct can_frame *frame, int canId) override;
    // Operational -> Stop(NMT)
    void fillCanFrameForExit(struct can_frame *frame, int canId) override;
    // ControlWord Shutdown(PDO)
    void fillCanFrameForQuickStop(struct can_frame *frame, int canId) override;

    // pre-operation, Stop -> oprational(MNT)
    void fillCanFrameForOperational(struct can_frame *frame, int canId);

    // Set TorqueOffset(SDO)
    void fillCanFrameForTorqueOffset(struct can_frame *frame, int canId);

    // ControlWold Enable(PDO)
    void fillCanFrameForEnable(struct can_frame *frame, int canId);

    // Set targetPostion(PDO)
    void fillCanFrameForTargetPostion(struct can_frame *frame, int canId, int targetPosition);

    // Sync(PDO)
    void fillCanFrameForSync(struct can_frame *frame);
};

#endif
