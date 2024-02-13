#ifndef CAN_SOCKET_UTILS_H
#define CAN_SOCKET_UTILS_H

#include <linux/can.h>
#include <math.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <bits/types.h>
#include <linux/can/raw.h>
#include <sys/time.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <map>
#include <queue>
#include <memory>
#include "Motor.hpp"
#include "CommandParser.hpp"

using namespace std;

/**
 * @class CanManager
 * @brief CAN 통신을 통해 모터와의 연결을 관리하고, 데이터 송수신 및 명령 실행을 담당하는 클래스입니다.
 *
 * 이 클래스는 모터와의 통신을 위한 소켓 설정, 연결 상태 확인, 데이터 송수신 등의 기능을 제공합니다.
 * 모터 제어 명령을 생성하고, CAN 프레임을 통해 모터로 전송하는 역할을 합니다.
 */
class CanManager
{
public:
    static const int ERR_SOCKET_CREATE_FAILURE = -1; ///< @brief 소켓 생성 실패시 반환되는 오류 코드입니다.
    static const int ERR_SOCKET_CONFIGURE_FAILURE = -2; ///< @brief 소켓 설정 실패시 반환되는 오류 코드입니다.

    /**
     * @brief CanManager 클래스의 생성자.
     * @param motorsRef 모터 객체들의 참조를 저장하는 맵. 키는 모터의 이름, 값은 모터 객체에 대한 shared_ptr입니다.
     */
    CanManager(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);
    
    /**
     * @brief CanManager 클래스의 소멸자.
     */
    ~CanManager();

    /**
     * @brief CAN 포트를 초기화하고, 모터와의 통신을 준비합니다.
     */
    void initializeCAN();

    /**
     * @brief 모든 CAN 포트를 재시작합니다.
     */
    void restartCanPorts();

    /**
     * @brief 소켓의 타임아웃을 설정합니다.
     * @param sec 초 단위의 타임아웃 시간.
     * @param usec 마이크로초 단위의 타임아웃 시간.
     */
    void setSocketsTimeout(int sec, int usec);

    /**
     * @brief 모든 CAN 포트의 상태를 확인하고, 연결 상태를 업데이트합니다.
     */
    void checkCanPortsStatus();

    /**
     * @brief 모터와의 통신을 위한 소켓을 설정합니다.
     */
    void setMotorsSocket();

    /**
     * @brief 지정된 모터와의 연결 상태를 확인합니다.
     * @param motor 연결 상태를 확인할 모터의 shared_ptr.
     * @return 연결이 성공적이면 true, 실패하면 false를 반환합니다.
     */
    bool checkConnection(std::shared_ptr<GenericMotor> motor);

    /**
     * @brief 모든 모터와의 연결 상태를 확인합니다.
     * @return 모든 모터와의 연결이 성공적이면 true, 하나라도 실패하면 false를 반환합니다.
     */
    bool checkAllMotors();

    /**
     * @brief 지정된 모터로 데이터를 송신하고 응답을 수신합니다.
     * @param motor 데이터 송수신할 모터의 shared_ptr.
     * @param frame 송신할 CAN 프레임.
     * @return 데이터 송수신이 성공적이면 true, 실패하면 false를 반환합니다.
     */
    bool sendAndRecv(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);

    /**
     * @brief 지정된 모터의 송신 버퍼에서 데이터를 송신합니다.
     * @param motor 데이터를 송신할 모터의 shared_ptr.
     * @return 데이터 송신이 성공적이면 true, 실패하면 false를 반환합니다.
     */
    bool sendFromBuff(std::shared_ptr<GenericMotor> &motor);

    /**
     * @brief 지정된 모터의 수신 버퍼로 데이터를 수신합니다.
     * @param motor 데이터를 수신할 모터의 shared_ptr.
     * @param readCount 읽을 데이터 프레임의 수.
     * @return 데이터 수신이 성공적이면 true, 실패하면 false를 반환합니다.
     */
    bool recvToBuff(std::shared_ptr<GenericMotor> &motor, int readCount);

    /**
     * @brief 모터로 데이터 프레임을 송신합니다.
     * @param motor 데이터 프레임을 송신할 모터의 shared_ptr.
     * @param frame 송신할 CAN 프레임.
     * @return 데이터 프레임 송신이 성공적이면 true, 실패하면 false를 반환합니다.
     */
    bool txFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);

    /**
     * @brief 모터로부터 데이터 프레임을 수신합니다.
     * @param motor 데이터 프레임을 수신할 모터의 shared_ptr.
     * @param frame 수신할 CAN 프레임.
     * @return 데이터 프레임 수신이 성공적이면 true, 실패하면 false를 반환합니다.
     */
    bool rxFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);

    /**
     * @brief 모든 소켓에서 CAN 프레임을 읽습니다.
     */
    void readFramesFromAllSockets();

    /**
     * @brief 수신된 CAN 프레임을 적절한 모터에 분배합니다.
     */
    void distributeFramesToMotors();

    /**
     * @brief 읽기 버퍼를 비웁니다.
     */
    void clearReadBuffers();

    std::map<std::string, int> sockets; ///< 모터와 통신하는 소켓의 맵.
    std::map<std::string, bool> isConnected; ///< 모터의 연결 상태를 나타내는 맵.
    int maxonCnt=0; ///< 연결된 Maxon 모터의 수.

private:
    std::vector<std::string> ifnames; ///< 사용 가능한 인터페이스 이름 목록.
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors; ///< 연결된 모터들의 참조를 저장하는 맵.

    TMotorCommandParser tmotorcmd; ///< T 모터 명령 파서.
    MaxonCommandParser maxoncmd; ///< Maxon 모터 명령 파서.

    std::map<int, std::vector<can_frame>> tempFrames; ///< 임시 프레임 저장소.

    bool getCanPortStatus(const char *port); ///< 특정 CAN 포트의 상태를 반환합니다.
    void activateCanPort(const char *port); ///< 특정 CAN 포트를 활성화합니다.
    void list_and_activate_available_can_ports(); ///< 사용 가능한 모든 CAN 포트를 활성화합니다.
    void deactivateCanPort(const char *port); ///< 특정 CAN 포트를 비활성화합니다.

    int createSocket(const std::string &ifname); ///< 특정 인터페이스에 대한 소켓을 생성합니다.
    int setSocketTimeout(int socket, int sec, int usec); ///< 소켓의 타임아웃을 설정합니다.

    void clearCanBuffer(int canSocket); ///< 특정 CAN 소켓의 버퍼를 비웁니다.
};

#endif // CAN_SOCKET_UTILS_H
