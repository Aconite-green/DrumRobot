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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GenericMotor
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class GenericMotor
{
public:
    uint32_t nodeId;
    std::string interFaceName;
    float desPos, desVel, desTor, currentPos, currentVel, currentTor;
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TMotor
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



class TMotor : public GenericMotor
{
public:
    TMotor(uint32_t nodeId, const std::string &motorType);
    std::string motorType;

    int sensorBit;

    //std::queue<TMotorData> sendBuffer;
    //std::queue<can_frame> recieveBuffer;
    // void addTMotorData(float position, float velocity, float kp, float kd, float torqueOffset);

private:
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// maxonMotor
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class MaxonMotor : public GenericMotor
{
public:
    MaxonMotor(uint32_t nodeId);

    //std::queue<MaxonMotorData> sendBuffer;
    //std::queue<can_frame> recieveBuffer;

    //void addMaxonMotorData(int position);

    uint32_t canSendId;
    uint32_t canReceiveId;

    uint32_t txPdoIds[4]; // 변경된 부분
    uint32_t rxPdoIds[4]; // 변경된 부분
};

#endif
