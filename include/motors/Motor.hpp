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

/**
 * @class GenericMotor
 * @brief 모든 모터 타입의 기본이 되는 범용 모터 클래스입니다.
 *
 * 이 클래스는 모터의 기본 속성과 기능을 정의합니다. 모든 특정 모터 타입 클래스는 이 클래스를 상속받습니다.
 */
class GenericMotor
{
public:
    uint32_t nodeId; ///< 모터의 노드 ID.
    std::string interFaceName; ///< 모터가 연결된 인터페이스의 이름.
    float desPos, desVel, desTor; ///< 목표 위치, 속도, 토크.
    float currentPos, currentVel, currentTor; ///< 현재 위치, 속도, 토크.
    float cwDir; ///< 시계 방향 회전을 나타내는 방향 값.
    bool isHomed, isConected; ///< 홈 위치에 있는지, 연결되어 있는지의 상태.
    float rMin, rMax; ///< 회전 범위의 최소, 최대 값.
    int socket; ///< 모터가 연결된 소켓의 식별자.
    int Kp; ///< 비례 제어 게인.
    double Kd; ///< 미분 제어 게인.
    std::queue<can_frame> sendBuffer; ///< 송신 버퍼.
    std::queue<can_frame> recieveBuffer; ///< 수신 버퍼.

    GenericMotor(uint32_t nodeId);
    virtual ~GenericMotor() = default;

    void clearSendBuffer(); ///< 송신 버퍼를 클리어합니다.
    void clearReceiveBuffer(); ///< 수신 버퍼를 클리어합니다.
};

/**
 * @class TMotor
 * @brief TMotor를 위한 클래스입니다. GenericMotor를 상속받습니다.
 *
 * TMotor 특화된 기능과 속성을 정의합니다.
 */
class TMotor : public GenericMotor
{
public:
    TMotor(uint32_t nodeId, const std::string &motorType);
    std::string motorType; ///< 모터의 타입.

    int sensorBit; ///< 센서 비트.

private:
};

/**
 * @class MaxonMotor
 * @brief Maxon 모터를 위한 클래스입니다. GenericMotor를 상속받습니다.
 *
 * Maxon 모터 특화된 기능과 속성을 정의합니다.
 */
class MaxonMotor : public GenericMotor
{
public:
    MaxonMotor(uint32_t nodeId);

    uint32_t canSendId; ///< CAN 송신 ID.
    uint32_t canReceiveId; ///< CAN 수신 ID.

    uint32_t txPdoIds[4]; ///< TX PDO 식별자 배열.
    uint32_t rxPdoIds[4]; ///< RX PDO 식별자 배열.
};

#endif // MOTOR_H