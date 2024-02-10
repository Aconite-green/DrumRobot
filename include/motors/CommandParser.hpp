#ifndef COMMANDPARSER_H
#define COMMANDPARSER_H

#include "Motor.hpp"
#include <linux/can.h>
#include <cmath>
#include <tuple>
#include <iostream>

using namespace std;

/**
 * @class TMotorCommandParser
 * @brief TMotor 명령어를 파싱하는 클래스입니다.
 *
 * TMotor에 대한 명령어 송수신 및 해석을 담당합니다. 이 클래스는 TMotor 명령 구성 및
 * 응답 파싱을 위한 메서드를 제공합니다.
 */
class TMotorCommandParser
{
public:
    float GLOBAL_P_MIN = -12.5; ///< 최소 위치 제한.
    float GLOBAL_P_MAX = 12.5; ///< 최대 위치 제한.
    float GLOBAL_KP_MIN = 0; ///< 최소 비례 게인 제한.
    float GLOBAL_KP_MAX = 500; ///< 최대 비례 게인 제한.
    float GLOBAL_KD_MIN = 0; ///< 최소 미분 게인 제한.
    float GLOBAL_KD_MAX = 5; ///< 최대 미분 게인 제한.
    float GLOBAL_V_MIN, GLOBAL_V_MAX, GLOBAL_T_MIN, GLOBAL_T_MAX; ///< 속도 및 토크 제한.

    /**
     * @brief TMotor로 송신할 명령을 파싱합니다.
     * @param motor 명령을 송신할 TMotor 객체.
     * @param frame 송신할 CAN 프레임.
     * @param canId CAN ID.
     * @param dlc 데이터 길이 코드.
     * @param p_des 목표 위치.
     * @param v_des 목표 속도.
     * @param kp 비례 게인.
     * @param kd 미분 게인.
     * @param t_ff 피드포워드 토크.
     */
    void parseSendCommand(TMotor &motor, struct can_frame *frame, int canId, int dlc, float p_des, float v_des, float kp, float kd, float t_ff);

    /**
     * @brief TMotor로부터 수신된 명령을 파싱합니다.
     * @param motor 명령을 수신한 TMotor 객체.
     * @param frame 수신한 CAN 프레임.
     * @return 명령 코드, 위치, 속도, 토크의 튜플.
     */
    std::tuple<int, float, float, float> parseRecieveCommand(TMotor &motor, struct can_frame *frame);

    // 이하 메서드들은 TMotor의 특정 상태 또는 모드를 요청하는 명령을 구성합니다.
    // 각 메서드에 대한 자세한 설명은 생략하나, 실제 코드에는 해당 명령의 목적과 사용 방법에 대해 설명을 추가해야 합니다.
    void getCheck(TMotor &motor, struct can_frame *frame);
    void getControlMode(TMotor &motor, struct can_frame *frame);
    void getExit(TMotor &motor, struct can_frame *frame);
    void getZero(TMotor &motor, struct can_frame *frame);
    void getQuickStop(TMotor &motor, struct can_frame *frame);

private:
    // 내부적으로 사용되는 유틸리티 메서드들입니다. 이러한 메서드들은 클래스의 공개 인터페이스의 일부가 아니며, 주로 데이터 변환에 사용됩니다.
    int floatToUint(float x, float x_min, float x_max, unsigned int bits);
    float uintToFloat(int x_int, float x_min, float x_max, int bits);

    void setMotorLimits(TMotor &motor);
};

/**
 * @class MaxonCommandParser
 * @brief Maxon 모터 명령어를 파싱하는 클래스입니다.
 *
 * Maxon 모터에 대한 명령어 송수신 및 해석을 담당합니다. 이 클래스는 Maxon 모터 명령 구성 및
 * 응답 파싱을 위한 메서드를 제공합니다.
 */
class MaxonCommandParser
{
public:
    // Maxon 모터로부터 수신된 명령을 파싱하는 메서드입니다. 반환된 튜플은 명령 코드, 위치, 속도를 포함합니다.
    std::tuple<int, float, float> parseRecieveCommand(MaxonMotor &motor, struct can_frame *frame);

    // 이하 메서드들은 Maxon 모터의 특정 상태 또는 모드를 요청하는 명령을 구성합니다.
    // 각 메서드에 대한 자세한 설명은 생략하나, 실제 코드에는 해당 명령의 목적과 사용 방법에 대해 설명을 추가해야 합니다.
    void getCheck(MaxonMotor &motor, struct can_frame *frame);
    void getStop(MaxonMotor &motor, struct can_frame *frame);
    void getQuickStop(MaxonMotor &motor, struct can_frame *frame);
    void getOperational(MaxonMotor &motor, struct can_frame *frame);
    void getEnable(MaxonMotor &motor, struct can_frame *frame);
    void getSync(struct can_frame *frame);

    // CSP 모드 관련 명령 구성 메서드.
    void getCSPMode(MaxonMotor &motor, struct can_frame *frame);
    void getTorqueOffset(MaxonMotor &motor, struct can_frame *frame);
    void getPosOffset(MaxonMotor &motor, struct can_frame *frame);
    void getTargetPosition(MaxonMotor &motor, struct can_frame *frame, float p_des_radians);

    // HMM 모드 관련 명령 구성 메서드.
    void getHomeMode(MaxonMotor &motor, struct can_frame *frame);
    void getFlowingErrorWindow(MaxonMotor &motor, struct can_frame *frame);
    void getHomeoffsetDistance(MaxonMotor &motor, struct can_frame *frame, int degree);
    void getHomePosition(MaxonMotor &motor, struct can_frame *frame, int degree);
    void getHomingMethodL(MaxonMotor &motor, struct can_frame *frame);
    void getHomingMethodR(MaxonMotor &motor, struct can_frame *frame);
    void getStartHoming(MaxonMotor &motor, struct can_frame *frame);
    void getCurrentThreshold(MaxonMotor &motor, struct can_frame *frame);

    // CSV 모드 관련 명령 구성 메서드.
    void getCSVMode(MaxonMotor &motor, struct can_frame *frame);
    void getVelOffset(MaxonMotor &motor, struct can_frame *frame);
    void getTargetVelocity(MaxonMotor &motor, struct can_frame *frame, int targetVelocity);

    // CST 모드 관련 명령 구성 메서드.
    void getCSTMode(MaxonMotor &motor, struct can_frame *frame);
    void getTargetTorque(MaxonMotor &motor, struct can_frame *frame,int targetTorque);
};

#endif // COMMANDPARSER_H
