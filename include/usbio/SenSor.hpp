#pragma once

#include "Global.hpp"
#include <stdio.h>
#include <time.h>

using namespace std;

/**
 * @class Sensor
 * @brief 센서와의 통신 및 데이터 수집을 관리하는 클래스입니다.
 *
 * 이 클래스는 USB 디바이스를 통한 센서 데이터의 읽기, 디바이스 연결 및 연결 해제 등의 기능을 제공합니다.
 */
class Sensor
{
public:
    /**
     * @brief Sensor 클래스의 기본 생성자입니다.
     *
     * 센서 초기화 및 필요한 설정을 수행합니다.
     */
    Sensor();

    /**
     * @brief Sensor 클래스의 소멸자입니다.
     *
     * 리소스 해제 및 디바이스 연결 해제를 수행합니다.
     */
    ~Sensor();

    /**
     * @brief 센서로부터 데이터를 읽어옵니다.
     * @return 읽어온 데이터 값입니다.
     */
    DWORD ReadVal();

    /**
     * @brief 성공할 때까지 디바이스와의 연결을 시도합니다.
     * @return 연결 성공 시 true, 그렇지 않으면 false를 반환합니다.
     */
    bool OpenDeviceUntilSuccess();

    /**
     * @brief 디바이스 연결을 해제합니다.
     */
    void closeDevice();

    /**
     * @brief 센서와의 연결을 설정합니다.
     */
    void connect();

    bool connected; ///< 센서 연결 상태를 나타냅니다. 연결되면 true, 그렇지 않으면 false입니다.

private:
    int DeviceID; ///< 디바이스 ID입니다. USB2051_32와 같은 상수를 사용합니다.
    BYTE BoardID; ///< 보드 ID입니다. 특정 디바이스를 식별하는 데 사용됩니다.
    BYTE total_di; ///< 디지털 입력 채널의 총 수입니다.
    int DevNum, res; ///< 디바이스 번호와 결과 코드를 저장합니다.
    char module_name[15]; ///< 모듈 이름을 저장합니다.
    DWORD DIValue; ///< 최근 읽어온 디지털 입력 값입니다.
    DWORD o_dwDICntValue[USBIO_DI_MAX_CHANNEL]; ///< 채널별 디지털 입력 카운트 값을 저장합니다.

    struct timespec start, end; ///< 연산 수행 시간 측정을 위한 시작 및 종료 시간입니다.
    long duration; ///< 연산 수행 시간을 저장합니다. 시간 측정에 사용됩니다.
};
