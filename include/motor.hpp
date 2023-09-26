#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdlib.h>

#include <string>
#include "MotorInterface.hpp"
#include "CommandParser.hpp"
const int MaxMotors = 10;

class TMotor : public MotorInterface
{
public:
    float pMin, pMax;
    float vMin, vMax;
    float kpMin, kpMax;
    float kdMin, kdMax;
    float tMin, tMax;

    std::string roboticSection;
    std::string motorType;
    int id;

    
    TMotor(int id, const std::string &motorType, const std::string &roboticSection);
    void setLimits();

    // Passer
    TMotorCommandParser parser;
    // CanService
    void fillCanFrameForCheckMotor(struct can_frame *frame) override;
    void fillCanFrameForControlMode(struct can_frame *frame) override;
    void fillCanFrameForExit(struct can_frame *frame) override;
    void fillCanFrameForZeroing(struct can_frame *frame) override;
};

class MaxonMotor : public MotorInterface
{
public:
    int nodeId;
    int canSendId;
    int canReceiveId;

    int pdoId[4];

    std::string roboticSection;

    MaxonMotor(int nodeId, const std::string &roboticSection, std::initializer_list<int> pdoIds);

    // Passer
    MaxonCommandParser parser;

    // CanService
    void fillCanFrameForControlMode(struct can_frame *frame) override;
    void fillCanFrameForZeroing(struct can_frame *frame) override;
    void fillCanFrameForCheckMotor(struct can_frame *frame) override;
    void fillCanFrameForExit(struct can_frame *frame) override;
};

#endif
