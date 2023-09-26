// motor.c 파일
#include "../include/motor.hpp" // Include header file
#include <iostream>

Tmotor::Tmotor(int id, const std::string& motortype, const std::string& roboticSection): motortype(motortype), id(id), roboticSection(roboticSection) {
    // 공통된 초기값 설정
    P_MIN = -12.5;
    P_MAX = 12.5;
    Kp_MIN = 0;
    Kp_MAX = 500;
    Kd_MIN = 0;
    Kd_MAX = 5;

    // 타입에 따른 초기값 설정
    setLimits();
}

void Tmotor::setLimits() {
    if (motortype == "AK10_9") {
        V_MIN = -50;
        V_MAX = 50;
        T_MIN = -65;
        T_MAX = 65;
    } else if (motortype == "AK70_10") {
        V_MIN = -50;
        V_MAX = 50;
        T_MIN = -25;
        T_MAX = 25;
    } else if (motortype == "AK60_6") {
        V_MIN = -45;
        V_MAX = 45;
        T_MIN = -15;
        T_MAX = 15;
    } else if (motortype == "AK80_6") {
        V_MIN = -76;
        V_MAX = 76;
        T_MIN = -12;
        T_MAX = 12;
    } else if (motortype == "AK80_9") {
        V_MIN = -50;
        V_MAX = 50;
        T_MIN = -18;
        T_MAX = 18;
    } else if (motortype == "AK80_80" || motortype == "AK80_64") {
        V_MIN = -8;
        V_MAX = 8;
        T_MIN = -144;
        T_MAX = 144;
    } else if (motortype == "AK80_8") {
        V_MIN = -37.5;
        V_MAX = 37.5;
        T_MIN = -32;
        T_MAX = 32;
    } else {
        std::cout << "Error: Invalid motor motortype entered!" << std::endl;
    }
}


