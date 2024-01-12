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
#include <vector>

struct CanFrameInfo
{
    uint32_t can_id;
    uint8_t can_dlc;
    std::array<uint32_t, 8> data;
};

class TMotor
{
public:
    TMotor(uint32_t nodeId, const std::string &motorType, const std::string &interFaceName);
    CanFrameInfo getCanFrameForCheckMotor();
    CanFrameInfo getCanFrameForControlMode();
    CanFrameInfo getCanFrameForExit();
    CanFrameInfo getCanFrameForZeroing();
    CanFrameInfo getCanFrameForQuickStop();
    uint32_t nodeId;

    double currentPos;
    float cwDir;
    int sensorBit;
    bool isHomed, isConected;

    float desPos, desVel, desTor, outPos, outVel, outTor;
    float rMin, rMax;

    std::string motorType;
    std::string interFaceName;

private:
};

class MaxonMotor
{
public:
    uint32_t nodeId;
    uint32_t canSendId;
    uint32_t canReceiveId;

    uint32_t txPdoIds[4];
    uint32_t rxPdoIds[4];

    MaxonMotor(uint32_t nodeId,
               const std::vector<uint32_t> &txPdoIds,
               const std::vector<uint32_t> &rxPdoIds,
               const std::string &interFaceName);

    std::string interFaceName;
    double currentPos;
    float cwDir;
    bool isHomed, isConected;
    float rMin, rMax;
    float outPos, outTor;

    /////////////////////////////// SYSTEM

    CanFrameInfo getCanFrameForCheckMotor();

    CanFrameInfo getCanFrameForStop();

    CanFrameInfo getCanFrameForQuickStop();

    CanFrameInfo getCanFrameForOperational();

    CanFrameInfo getCanFrameForEnable();

    CanFrameInfo getCanFrameForSync();

    /////////////////////////////// CSP
    CanFrameInfo getCanFrameForCSPMode();

    CanFrameInfo getCanFrameForTorqueOffset();

    CanFrameInfo getCanFrameForPosOffset();

    CanFrameInfo getCanFrameForTargetPosition(int targetPosition);

    /////////////////////////////// HMM
    CanFrameInfo getCanFrameForHomeMode();

    CanFrameInfo getFlowingErrorWindow();

    CanFrameInfo getHomeoffsetDistance();

    CanFrameInfo getMaxProfileVelocity();

    CanFrameInfo getQuickStopDeceleration();

    CanFrameInfo getSpeedForSwitchSearch();

    CanFrameInfo getSpeedForZeroSearch();

    CanFrameInfo getHomingAcceleration();

    CanFrameInfo getHomePosition();

    CanFrameInfo getHomingMethod();

    CanFrameInfo getStartHoming();

    CanFrameInfo getCanFrameForCurrentThreshold();

    /////////////////////////////// CSV
    CanFrameInfo getCanFrameForCSVMode();

    CanFrameInfo getCanFrameForVelOffset();

    CanFrameInfo getCanFrameForTargetVelocity(int targetVelocity);
    
    /////////////////////////////// CST
    CanFrameInfo getCanFrameForCSTMode();

    CanFrameInfo getCanFrameForTargetTorque(int targetTorque);

};

#endif
