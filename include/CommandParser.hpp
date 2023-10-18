#ifndef COMMANDPARSER_H
#define COMMANDPARSER_H

#include "Motor.hpp"
#include <linux/can.h>
#include <cmath>
#include <tuple>

class TMotor;

class TMotorCommandParser
{

public:
    void parseSendCommand(TMotor &motor, struct can_frame *frame, int canId, int dlc, float p_des, float v_des, float kp, float kd, float t_ff);
    std::tuple<int, float, float, float> parseRecieveCommand(TMotor &motor, struct can_frame *frame);

private:
    int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
};

class MaxonCommandParser
{
public:
    void parseSendCommand(MaxonMotor &motor, struct can_frame *frame, int p_des);
    std::tuple<int, float> parseRecieveCommand(struct can_frame *frame);
    void makeSync(struct can_frame *frame);
};

#endif