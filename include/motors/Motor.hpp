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
    std::string interFaceName;
    // Values
    float desPos, desVel, desTor;
    float prePos;
    float currentPos, currentVel, currentTor;
    float coordinatePos;
    int Kp;
    double Kd;
    // For Homing Session

    bool isHomed;

    std::queue<can_frame> recieveBuffer;
    std::queue<can_frame> sendBuffer;

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
    double homeOffset = 0.0;
    double sensorLocation = 0.0;

    // For Homing Session
    bool atFirstSensor, atSecondSensor, atZeroPosition;

    std::queue<TMotorData> commandBuffer;
    std::queue<TMotorData> InRecordBuffer;

    void clearCommandBuffer();
    void clearInRecordBuffer();

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
    // float targetPos = M_PI / 18.0;

    bool checked = false;

    unsigned char statusBit;
    double homeOffset = 0.0;
    double bumperLocation = 0.0;

    queue<MaxonData> commandBuffer;
    queue<MaxonData> InRecordBuffer;
    queue<double> wrist_BackArr;
    void clearCommandBuffer();
    void clearInRecordBuffer();
    void clearWrist_BackArr();
};

#endif // MOTOR_H