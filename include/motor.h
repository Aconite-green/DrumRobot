// motor.h 파일
#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdlib.h>


#define MAX_MOTORS 100

typedef struct {
    float P_MIN, P_MAX;
    float V_MIN, V_MAX;
    float Kp_MIN, Kp_MAX;
    float Kd_MIN, Kd_MAX;
    float T_MIN, T_MAX;
} MotorLimits;

typedef struct {
    const char* type;
    int id;
    MotorLimits limits;
} Motor;

typedef struct {
    int motor_id;
    float p_des;
    float v_des;
    float kp;
    float kd;
    float t_ff;
} MotorCommand;

typedef struct {
    Motor motors[MAX_MOTORS];
    int num_motors;
} MotorContainer;

MotorContainer* get_motor_container();
MotorLimits* get_motor_limits(const char *motortype);
MotorLimits *get_motor_limits_by_id(MotorContainer *container, int id);

#endif 