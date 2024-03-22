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
    std::string myName;
    // Values
    float desPos, desVel, desTor;
    float currentPos, currentVel, currentTor;

    // For Homing Session
    bool atFirstSensor, atSecondSensor, atZeroPosition;
    bool isHomed;
    int homeOffset;

    int Kp;
    double Kd;
    std::queue<can_frame> recieveBuffer;

    struct can_frame sendFrame;

    GenericMotor(uint32_t nodeId);
    virtual ~GenericMotor() = default;

    void clearSendBuffer();
    void clearReceiveBuffer();
};

struct TMotorData
{
    float position;
    float velocity;
};

class TMotor : public GenericMotor
{
public:
    TMotor(uint32_t nodeId, const std::string &motorType);
    std::string motorType;

    int sensorBit;
    double homeOffset;
    std::queue<TMotorData> commandBuffer;

    void clearCommandBuffer();

private:
};

struct MaxonData
{
    float position;
    float wristState;
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

    bool hitting = false;

    bool atPosition = false;
    bool positioning = false;
    float targetPos = 0;

    bool checked = false;

    std::queue<MaxonData> commandBuffer;
    void clearCommandBuffer();
};

#endif // MOTOR_H