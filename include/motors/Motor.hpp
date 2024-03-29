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
    uint32_t nodeId; 
    std::string interFaceName; 
    float desPos, desVel, desTor; 
    float currentPos, currentVel, currentTor; 
    float cwDir; 
    bool isHomed, isConected; 
    float rMin, rMax; 
    int socket; 
    int Kp; 
    double Kd; 
    std::queue<can_frame> sendBuffer; 
    std::queue<can_frame> recieveBuffer; 

    GenericMotor(uint32_t nodeId);
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

private:
};




class MaxonMotor : public GenericMotor
{
public:
    MaxonMotor(uint32_t nodeId);

    uint32_t canSendId; 
    uint32_t canReceiveId; 

    uint32_t txPdoIds[4]; 
    uint32_t rxPdoIds[4]; 
};

#endif // MOTOR_H