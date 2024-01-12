#ifndef COMMANDPARSER_H
#define COMMANDPARSER_H

#include "Motor.hpp"
#include <linux/can.h>
#include <cmath>
#include <tuple>
#include <iostream>

class TMotor;

class TMotorCommandParser
{
public:
    float GLOBAL_P_MIN = -12.5;
    float GLOBAL_P_MAX = 12.5;
    float GLOBAL_KP_MIN = 0;
    float GLOBAL_KP_MAX = 500;
    float GLOBAL_KD_MIN = 0;
    float GLOBAL_KD_MAX = 5;
    float GLOBAL_V_MIN, GLOBAL_V_MAX, GLOBAL_T_MIN, GLOBAL_T_MAX;
    
    void parseSendCommand(TMotor &motor, struct can_frame *frame, int canId, int dlc, float p_des, float v_des, float kp, float kd, float t_ff);
    std::tuple<int, float, float, float> parseRecieveCommand(TMotor &motor, struct can_frame *frame);

private:
    int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);

    void setMotorLimits(TMotor &motor);
};

class MaxonCommandParser
{
public:
    void parsePosCommand(MaxonMotor &motor, struct can_frame *frame, float p_des_radians);
    void parseVelCommand(MaxonMotor &motor, struct can_frame *frame, int des_vel);
    void parseTrqCommand(MaxonMotor &motor, struct can_frame *frame, int des_tff);
    std::tuple<int, float, float> parseRecieveCommand(MaxonMotor &motor,struct can_frame *frame);
    void makeSync(struct can_frame *frame);
};

#endif