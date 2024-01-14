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

class GenericMotor
{
public:
    uint32_t nodeId;
    double currentPos;
    float cwDir;
    bool isHomed, isConected;
    float rMin, rMax;
    int socket;

    GenericMotor(uint32_t nodeId) : nodeId(nodeId), currentPos(0), cwDir(0), isHomed(false), isConected(false), rMin(0), rMax(0), socket(0) {}
};

struct CanFrameInfo
{
    uint32_t can_id;
    uint8_t can_dlc;
    std::array<uint32_t, 8> data;
};

class TMotor : public GenericMotor
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

class MaxonMotor : public GenericMotor
{
public:
    MaxonMotor(uint32_t nodeId, const std::vector<uint32_t> &txPdoIds, const std::vector<uint32_t> &rxPdoIds, const std::string &interFaceName);
        
    uint32_t nodeId;
    uint32_t canSendId;
    uint32_t canReceiveId;

    std::vector<uint32_t> txPdoIds; // 변경된 부분
    std::vector<uint32_t> rxPdoIds; // 변경된 부분

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

    CanFrameInfo getHomingMethodL();

    CanFrameInfo getHomingMethodR();

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
