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

class GenericMotor
{
public:
    uint32_t nodeId;
    std::string interFaceName;
    double currentPos;
    float cwDir;
    bool isHomed, isConected;
    float rMin, rMax;
    int socket;
    int Kp;
    double Kd;
    std::queue<can_frame> sendBuffer;
    std::queue<can_frame> recieveBuffer;

    GenericMotor(uint32_t nodeId, const std::string &interFaceName) : nodeId(nodeId), interFaceName(interFaceName), currentPos(0), cwDir(0), isHomed(false), isConected(false), rMin(0), rMax(0), socket(0), Kp(0) {}
    virtual ~GenericMotor() = default;

    void clearSendBuffer();
    void clearReceiveBuffer();
};

class TMotor : public GenericMotor
{
public:
    TMotor(uint32_t nodeId, const std::string &motorType);
    std::string motorType;

    int sensorBit;
    float desPos, desVel, desTor, outPos, outVel, outTor;

private:
};

class MaxonMotor : public GenericMotor
{
public:
    MaxonMotor(uint32_t nodeId);

    uint32_t canSendId;
    uint32_t canReceiveId;

    uint32_t txPdoIds[4]; // 변경된 부분
    uint32_t rxPdoIds[4]; // 변경된 부분

    float outPos, outTor;
};

#endif
