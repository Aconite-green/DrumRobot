#ifndef COMMANDPARSER_H
#define COMMANDPARSER_H

#include "Motor.hpp"
#include <linux/can.h>
#include <cmath>

class TMotor;

class TMotorCommandParser
{

public:
    void parseSendCommand(TMotor &motor, struct can_frame *frame, int canId, int dlc, float p_des, float v_des, float kp, float kd, float t_ff);
    void parseRecieveCommand(TMotor &motor, struct can_frame *frame);

private:
    int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
};

class MaxonCommandParser
{
    void parseSendCommand(struct can_frame *frame);
    void parseRecieveCommand(struct can_frame *frame);
};

#endif