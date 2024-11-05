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
#include <queue>
#include <linux/can/raw.h>
#include <iostream>
#include <cmath>
#include <map>
using namespace std;

class GenericMotor
{
public:
    // For CAN communication
    uint32_t nodeId;
    int socket;
    bool isConected;

    // Motors Feature
    float cwDir;
    float rMin, rMax;
    int32_t spd = 300; // ERPM
    int32_t acl = 10000; // ERPA
    std::string myName;
    bool isError = false;
    // Values
    float desPos, desVel, desTor;
    float prePos;
    float currentPos, currentVel, currentTor;
    float coordinatePos;
    // Fixed
    float fixedPos;
    bool isfixed = false;

    // Save CSV
    float pre_spd = 0.0;    

    // Gear ratio
    std::map<std::string, int> R_Ratio = {
        {"AK80_64", 64},
        {"AK70_10", 10},
        {"AK10_9", 9}
    };
    int PolePairs = 21;
    float timingBelt_ratio;

    // For Homing Session
    bool isHomed;

    std::queue<can_frame> recieveBuffer;
    std::queue<can_frame> sendBuffer;

    struct can_frame sendFrame;
    struct can_frame recieveFrame;

    GenericMotor(uint32_t nodeId);
    virtual ~GenericMotor() = default;

    void clearSendBuffer();
    void clearReceiveBuffer();
};

struct TMotorData
{
    float position;
    int32_t spd;
    int32_t acl;
    bool isBrake;
};

class TMotor : public GenericMotor
{
public:
    TMotor(uint32_t nodeId, const std::string &motorType);
    std::string motorType;

    float homeOffset = 0.0;
    bool brake_state;

    // For Homing Session
    bool atFirstSensor, atSecondSensor, atZeroPosition;

    // [A]
    float limitCurrent;

    int errorCnt = 0;

    std::queue<TMotorData> commandBuffer;

    void clearCommandBuffer();

private:
};

struct MaxonData
{
    float position;
    double WristState;
};

class MaxonMotor : public GenericMotor
{
public:
    MaxonMotor(uint32_t nodeId);

    uint32_t canSendId;
    uint32_t canReceiveId;

    uint32_t txPdoIds[4];
    uint32_t rxPdoIds[4];

    float positionValues[4] = {0}; // 포지션 값 저장을 위한 정적 배열
    int posIndex = 0;

    bool stay = false;
    bool hitting = false;
    bool isPositionMode = false;
    bool atPosition = false;
    bool positioning = false;

    bool checked = false;

    unsigned char statusBit;
    float homeOffset = 0.0;
    float bumperLocation = 0.0;
    int errorCnt = 0;

    queue<MaxonData> commandBuffer;
    queue<float> wrist_BackArr;
    void clearCommandBuffer();
    void clearWrist_BackArr();
};

#endif // MOTOR_H