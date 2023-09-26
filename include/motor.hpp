// motor.h 파일
#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdlib.h>


#include <string>

// 상수를 정의합니다. 실제로 사용할 때는 적절한 값으로 설정해 주세요.
const int MAX_MOTORS = 10;

class Tmotor {
public:
    // Limit 값들
    float P_MIN, P_MAX;
    float V_MIN, V_MAX;
    float Kp_MIN, Kp_MAX;
    float Kd_MIN, Kd_MAX;
    float T_MIN, T_MAX;

    // 모터 정보
    std::string roboticSection;
    std::string motortype;
    int id;

    // 커맨드 정보
    float p_des;
    float v_des;
    float kp;
    float kd;
    float t_ff;

    Tmotor(int id, const std::string& motortype, const std::string& roboticSection);
    void setLimits();

    
};

#endif 