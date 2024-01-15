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


class TMotor : public GenericMotor
{
public:
    TMotor(uint32_t nodeId, const std::string &motorType, const std::string &interFaceName);
    std::string motorType;
    std::string interFaceName;
    /*uint32_t nodeId;
    double currentPos;
    float cwDir;
    bool isHomed, isConected;
    float rMin, rMax;*/

    int sensorBit;
    float desPos, desVel, desTor, outPos, outVel, outTor;

    

private:
};

class MaxonMotor : public GenericMotor
{
public:
    MaxonMotor(uint32_t nodeId, const std::string &interFaceName);
    std::string interFaceName;

    uint32_t canSendId;
    uint32_t canReceiveId;

    uint32_t txPdoIds[4]; // 변경된 부분
    uint32_t rxPdoIds[4]; // 변경된 부분

    /*uint32_t nodeId;
    double currentPos;
    float cwDir;
    bool isHomed, isConected;
    float rMin, rMax;*/

    float outPos, outTor;
};

#endif
