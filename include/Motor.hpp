#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdlib.h>
#include <array>
#include <cstdint>
#include <string>


struct CanFrameInfo
{
    uint32_t can_id;
    uint8_t can_dlc;
    std::array<uint8_t, 8> data;
};

class TMotor
{
public:
    TMotor(int nodeId, const std::string &motorType, const std::string &interFaceName);
    CanFrameInfo getCanFrameForCheckMotor();
    CanFrameInfo getCanFrameForControlMode();
    CanFrameInfo getCanFrameForExit();
    CanFrameInfo getCanFrameForZeroing();
    CanFrameInfo getCanFrameForQuickStop();
    uint32_t nodeId;

    float pMin, pMax;
    float vMin, vMax;
    float kpMin, kpMax;
    float kdMin, kdMax;
    float tMin, tMax;

    std::string motorType;
    std::string interFaceName;

private:
    void setLimits();
};

class MaxonMotor
{
public:
    uint32_t canSendId;
    uint32_t canReceiveId;

    uint32_t pdoId[4];

    MaxonMotor(int nodeId, std::initializer_list<int> pdoIds);

    // Send all zero(SDO)
    CanFrameInfo getCanFrameForCheckMotor();
    // CSP mode(SDO)
    CanFrameInfo getCanFrameForControlMode();
    // Set pos offset(SDO)
    CanFrameInfo getCanFrameForZeroing();
    // Operational -> Stop(NMT)
    CanFrameInfo getCanFrameForExit();
    // ControlWord Shutdown(PDO)
    CanFrameInfo getCanFrameForQuickStop();
    // pre-operation, Stop -> oprational(MNT)
    CanFrameInfo getCanFrameForOperational();
    // Set TorqueOffset(SDO)
    CanFrameInfo getCanFrameForTorqueOffset();
    // ControlWold Enable(PDO)
    CanFrameInfo getCanFrameForEnable();
    // Set targetPosition(PDO)
    CanFrameInfo getCanFrameForTargetPosition(int targetPosition);
    // Sync(PDO)
    CanFrameInfo getCanFrameForSync();
};

#endif
